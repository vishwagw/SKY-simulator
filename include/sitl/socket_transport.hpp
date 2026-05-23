#pragma once

// Platform socket includes
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "Ws2_32.lib")
   using socket_t = SOCKET;
#  define INVALID_SOCK INVALID_SOCKET
#  define SOCK_ERR     SOCKET_ERROR
#  define sock_errno   WSAGetLastError()
#  define SOCK_WOULDBLOCK WSAEWOULDBLOCK
#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <netinet/tcp.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#  include <fcntl.h>
#  include <errno.h>
   using socket_t = int;
#  define INVALID_SOCK (-1)
#  define SOCK_ERR     (-1)
#  define sock_errno   errno
#  define SOCK_WOULDBLOCK EAGAIN
#  define closesocket  close
#endif

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <optional>
#include <functional>
#include <atomic>
#include <mutex>

namespace dronesim::sitl {

// ---------------------------------------------------------------------------
// Transport mode
// ---------------------------------------------------------------------------
enum class TransportMode { UDP, TCP };

// ---------------------------------------------------------------------------
// SocketTransport
//
// Single object per firmware connection. Thread-safe send; recv is
// intended to be called from the physics thread (Godot _integrate_forces).
//
// UDP:  bind local port, sendto remote addr, recvfrom (stateless)
// TCP:  connect as client OR listen+accept as server (firmware-dependent)
// ---------------------------------------------------------------------------
class SocketTransport {
public:
    struct Config {
        TransportMode mode{TransportMode::UDP};
        std::string   local_addr{"0.0.0.0"};
        uint16_t      local_port{14550};
        std::string   remote_addr{"127.0.0.1"};
        uint16_t      remote_port{14550};
        bool          tcp_server{false}; // true = listen, false = connect
        int           recv_buf_bytes{65536};
        int           send_buf_bytes{65536};
    };

    explicit SocketTransport(Config cfg) noexcept : _cfg(std::move(cfg)) {}
    ~SocketTransport() { close(); }

    // Non-copyable, movable
    SocketTransport(const SocketTransport&)            = delete;
    SocketTransport& operator=(const SocketTransport&) = delete;

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------
    [[nodiscard]] bool open() noexcept {
#ifdef _WIN32
        WSADATA wsa{};
        if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) return false;
#endif
        if (_cfg.mode == TransportMode::UDP)
            return _open_udp();
        else
            return _open_tcp();
    }

    void close() noexcept {
        if (_sock != INVALID_SOCK) {
            ::closesocket(_sock);
            _sock = INVALID_SOCK;
        }
        if (_accept_sock != INVALID_SOCK) {
            ::closesocket(_accept_sock);
            _accept_sock = INVALID_SOCK;
        }
#ifdef _WIN32
        WSACleanup();
#endif
        _connected = false;
    }

    [[nodiscard]] bool is_open()      const noexcept { return _sock != INVALID_SOCK; }
    [[nodiscard]] bool is_connected() const noexcept { return _connected; }
    [[nodiscard]] TransportMode mode() const noexcept { return _cfg.mode; }

    // -----------------------------------------------------------------------
    // Non-blocking send — returns false on unrecoverable error
    // -----------------------------------------------------------------------
    bool send(const uint8_t* data, size_t len) noexcept {
        if (!is_open()) return false;
        std::lock_guard<std::mutex> lk(_send_mtx);

        socket_t sock = (_cfg.mode == TransportMode::TCP && _cfg.tcp_server)
                        ? _accept_sock : _sock;
        if (sock == INVALID_SOCK) return false;

        if (_cfg.mode == TransportMode::UDP) {
            return ::sendto(sock,
                reinterpret_cast<const char*>(data), static_cast<int>(len), 0,
                reinterpret_cast<const sockaddr*>(&_remote_addr),
                sizeof(_remote_addr)) != SOCK_ERR;
        } else {
            // TCP — loop until all bytes sent
            size_t sent = 0;
            while (sent < len) {
                int n = ::send(sock,
                    reinterpret_cast<const char*>(data + sent),
                    static_cast<int>(len - sent), 0);
                if (n == SOCK_ERR) {
                    if (sock_errno == SOCK_WOULDBLOCK) continue;
                    _connected = false;
                    return false;
                }
                sent += static_cast<size_t>(n);
            }
            return true;
        }
    }

    bool send(const std::vector<uint8_t>& buf) noexcept {
        return send(buf.data(), buf.size());
    }

    // -----------------------------------------------------------------------
    // Non-blocking recv — returns byte count (0 = nothing yet, <0 = error)
    // For UDP: populates _last_remote_addr so we can reply
    // -----------------------------------------------------------------------
    int recv(uint8_t* buf, int max_len) noexcept {
        if (!is_open()) return -1;

        // TCP server: poll accept on main socket first
        if (_cfg.mode == TransportMode::TCP && _cfg.tcp_server) {
            _poll_accept();
            if (_accept_sock == INVALID_SOCK) return 0;
        }

        socket_t sock = (_cfg.mode == TransportMode::TCP && _cfg.tcp_server)
                        ? _accept_sock : _sock;
        if (sock == INVALID_SOCK) return 0;

        if (_cfg.mode == TransportMode::UDP) {
            socklen_t slen = sizeof(_last_remote_addr);
            int n = ::recvfrom(sock,
                reinterpret_cast<char*>(buf), max_len, 0,
                reinterpret_cast<sockaddr*>(&_last_remote_addr), &slen);
            if (n == SOCK_ERR) {
                return (sock_errno == SOCK_WOULDBLOCK) ? 0 : -1;
            }
            // For UDP: update remote target to whoever sent the last packet
            std::memcpy(&_remote_addr, &_last_remote_addr, sizeof(_remote_addr));
            return n;
        } else {
            int n = ::recv(sock, reinterpret_cast<char*>(buf), max_len, 0);
            if (n == SOCK_ERR) {
                if (sock_errno == SOCK_WOULDBLOCK) return 0;
                _connected = false;
                return -1;
            }
            if (n == 0) { _connected = false; return -1; } // graceful close
            return n;
        }
    }

    // -----------------------------------------------------------------------
    // TCP client: attempt non-blocking reconnect (call each tick if !connected)
    // -----------------------------------------------------------------------
    bool try_reconnect() noexcept {
        if (_connected) return true;
        if (_cfg.mode != TransportMode::TCP || _cfg.tcp_server) return false;
        // Re-create socket and attempt connect
        if (_sock != INVALID_SOCK) { ::closesocket(_sock); _sock = INVALID_SOCK; }
        return _open_tcp();
    }

    const Config& config() const noexcept { return _cfg; }

private:
    Config   _cfg;
    socket_t _sock{INVALID_SOCK};
    socket_t _accept_sock{INVALID_SOCK};
    std::atomic<bool> _connected{false};
    std::mutex _send_mtx;

    sockaddr_in _remote_addr{};
    sockaddr_in _last_remote_addr{};

    // -----------------------------------------------------------------------
    bool _open_udp() noexcept {
        _sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (_sock == INVALID_SOCK) return false;
        _set_nonblocking(_sock);
        _set_buf_sizes(_sock);

        sockaddr_in local{};
        local.sin_family      = AF_INET;
        local.sin_port        = htons(_cfg.local_port);
        local.sin_addr.s_addr = inet_addr(_cfg.local_addr.c_str());
        if (::bind(_sock, reinterpret_cast<const sockaddr*>(&local),
                   sizeof(local)) == SOCK_ERR) {
            ::closesocket(_sock); _sock = INVALID_SOCK; return false;
        }

        _remote_addr.sin_family      = AF_INET;
        _remote_addr.sin_port        = htons(_cfg.remote_port);
        _remote_addr.sin_addr.s_addr = inet_addr(_cfg.remote_addr.c_str());
        _connected = true;
        return true;
    }

    bool _open_tcp() noexcept {
        _sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (_sock == INVALID_SOCK) return false;
        _set_nonblocking(_sock);
        _set_buf_sizes(_sock);

        // Disable Nagle for low-latency
        int flag = 1;
        ::setsockopt(_sock, IPPROTO_TCP, TCP_NODELAY,
                     reinterpret_cast<const char*>(&flag), sizeof(flag));

        if (_cfg.tcp_server) {
            int reuse = 1;
            ::setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR,
                         reinterpret_cast<const char*>(&reuse), sizeof(reuse));
            sockaddr_in local{};
            local.sin_family      = AF_INET;
            local.sin_port        = htons(_cfg.local_port);
            local.sin_addr.s_addr = inet_addr(_cfg.local_addr.c_str());
            if (::bind(_sock, reinterpret_cast<const sockaddr*>(&local),
                       sizeof(local)) == SOCK_ERR) {
                ::closesocket(_sock); _sock = INVALID_SOCK; return false;
            }
            ::listen(_sock, 1);
            // _accept_sock populated later by _poll_accept()
            _connected = false; // wait for client
        } else {
            sockaddr_in remote{};
            remote.sin_family      = AF_INET;
            remote.sin_port        = htons(_cfg.remote_port);
            remote.sin_addr.s_addr = inet_addr(_cfg.remote_addr.c_str());
            int r = ::connect(_sock,
                reinterpret_cast<const sockaddr*>(&remote), sizeof(remote));
#ifdef _WIN32
            bool pending = (r == SOCK_ERR && WSAGetLastError() == WSAEWOULDBLOCK);
#else
            bool pending = (r == SOCK_ERR && errno == EINPROGRESS);
#endif
            if (r == 0) {
                _connected = true;
            } else if (pending) {
                // Async connect — resolved next poll
                _connected = false;
            } else {
                ::closesocket(_sock); _sock = INVALID_SOCK; return false;
            }
        }
        return true;
    }

    void _poll_accept() noexcept {
        if (_accept_sock != INVALID_SOCK) return;
        sockaddr_in client{};
        socklen_t slen = sizeof(client);
        socket_t s = ::accept(_sock,
            reinterpret_cast<sockaddr*>(&client), &slen);
        if (s != INVALID_SOCK) {
            _accept_sock = s;
            _set_nonblocking(_accept_sock);
            _set_buf_sizes(_accept_sock);
            _connected = true;
        }
    }

    void _set_nonblocking(socket_t s) noexcept {
#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(s, FIONBIO, &mode);
#else
        int flags = fcntl(s, F_GETFL, 0);
        fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
    }

    void _set_buf_sizes(socket_t s) noexcept {
        ::setsockopt(s, SOL_SOCKET, SO_RCVBUF,
            reinterpret_cast<const char*>(&_cfg.recv_buf_bytes), sizeof(int));
        ::setsockopt(s, SOL_SOCKET, SO_SNDBUF,
            reinterpret_cast<const char*>(&_cfg.send_buf_bytes), sizeof(int));
    }
};

} // namespace dronesim::sitl
