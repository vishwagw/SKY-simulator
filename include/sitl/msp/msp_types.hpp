#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <optional>
#include <array>

// ---------------------------------------------------------------------------
// MultiWii Serial Protocol v2 (MSP v2)
// Wire format:
//   '$' 'X' direction  flag  function(2) size(2) payload  CRC8
// direction: '<' = request/command,  '>' = response,  '!' = error
// CRC: DVB-S2 (CRC8 with poly 0xD5)
// ---------------------------------------------------------------------------
namespace dronesim::sitl::msp {

// ---------------------------------------------------------------------------
// MSP function codes used in SITL
// ---------------------------------------------------------------------------
static constexpr uint16_t MSP_API_VERSION   = 1;
static constexpr uint16_t MSP_FC_VARIANT    = 2;
static constexpr uint16_t MSP_FC_VERSION    = 3;
static constexpr uint16_t MSP_STATUS        = 101;
static constexpr uint16_t MSP_RAW_IMU       = 102;
static constexpr uint16_t MSP_ATTITUDE      = 108;
static constexpr uint16_t MSP_ALTITUDE      = 109;
static constexpr uint16_t MSP_RC            = 105;
static constexpr uint16_t MSP_SET_MOTOR     = 214;
static constexpr uint16_t MSP2_MOTOR_OUTPUT = 0x1FF;

// ---------------------------------------------------------------------------
// CRC8 DVB-S2
// ---------------------------------------------------------------------------
inline uint8_t crc8_dvb_s2(uint8_t crc, uint8_t b) noexcept {
    crc ^= b;
    for (int i = 0; i < 8; ++i)
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    return crc;
}

inline uint8_t crc8_dvb_s2_buf(const uint8_t* buf, size_t n, uint8_t init = 0) noexcept {
    uint8_t crc = init;
    for (size_t i = 0; i < n; ++i) crc = crc8_dvb_s2(crc, buf[i]);
    return crc;
}

// ---------------------------------------------------------------------------
// Parsed MSP v2 frame
// ---------------------------------------------------------------------------
struct Frame {
    char     direction{'>'};   // '<' request, '>' response
    uint8_t  flag{};
    uint16_t function{};
    std::vector<uint8_t> payload;
};

// ---------------------------------------------------------------------------
// Build a MSP v2 response frame
// ---------------------------------------------------------------------------
inline std::vector<uint8_t> make_response(
    uint16_t function, const uint8_t* payload, uint16_t size)
{
    std::vector<uint8_t> buf;
    buf.reserve(8 + size);
    buf.push_back('$');
    buf.push_back('X');
    buf.push_back('>');
    buf.push_back(0);  // flag
    buf.push_back(static_cast<uint8_t>(function & 0xFF));
    buf.push_back(static_cast<uint8_t>(function >> 8));
    buf.push_back(static_cast<uint8_t>(size & 0xFF));
    buf.push_back(static_cast<uint8_t>(size >> 8));
    for (uint16_t i = 0; i < size; ++i) buf.push_back(payload[i]);

    // CRC over header bytes 3..7 + payload
    uint8_t crc = 0;
    for (size_t i = 3; i < buf.size(); ++i) crc = crc8_dvb_s2(crc, buf[i]);
    buf.push_back(crc);
    return buf;
}

template<typename T>
inline std::vector<uint8_t> make_response(uint16_t fn, const T& payload) {
    return make_response(fn,
        reinterpret_cast<const uint8_t*>(&payload), sizeof(T));
}

// ---------------------------------------------------------------------------
// MSP v2 parser
// ---------------------------------------------------------------------------
class Parser {
    enum class State { IDLE, GOT_DOLLAR, GOT_X, GOT_DIR, GOT_FLAG,
                       FN0, FN1, SIZE0, SIZE1, PAYLOAD, CRC };
public:
    std::optional<Frame> parse_byte(uint8_t b) noexcept {
        switch (_s) {
        case State::IDLE:
            if (b == '$') _s = State::GOT_DOLLAR;
            break;
        case State::GOT_DOLLAR:
            _s = (b == 'X') ? State::GOT_X : State::IDLE;
            break;
        case State::GOT_X:
            _f.direction = static_cast<char>(b);
            _s = State::GOT_DIR;
            _crc = 0;
            _crc = crc8_dvb_s2(_crc, b);
            break;
        case State::GOT_DIR:
            _f.flag = b; _crc = crc8_dvb_s2(_crc, b); _s = State::GOT_FLAG; break;
        case State::GOT_FLAG:
            _f.function = b; _crc = crc8_dvb_s2(_crc, b); _s = State::FN0; break;
        case State::FN0:
            _f.function |= static_cast<uint16_t>(b) << 8; _crc = crc8_dvb_s2(_crc, b); _s = State::FN1; break;
        case State::FN1:
            _size = b; _crc = crc8_dvb_s2(_crc, b); _s = State::SIZE0; break;
        case State::SIZE0:
            _size |= static_cast<uint16_t>(b) << 8; _crc = crc8_dvb_s2(_crc, b);
            _f.payload.clear();
            _f.payload.reserve(_size);
            _idx = 0;
            _s = (_size > 0) ? State::SIZE1 : State::CRC;
            break;
        case State::SIZE1: // reused as PAYLOAD state
            _f.payload.push_back(b); _crc = crc8_dvb_s2(_crc, b);
            if (++_idx >= _size) _s = State::CRC;
            break;
        case State::PAYLOAD:
            _f.payload.push_back(b); _crc = crc8_dvb_s2(_crc, b);
            if (++_idx >= _size) _s = State::CRC;
            break;
        case State::CRC:
            _s = State::IDLE;
            if (b == _crc) {
                Frame f = std::move(_f);
                _f = {};
                return f;
            }
            break;
        }
        return std::nullopt;
    }

    std::vector<Frame> parse(const uint8_t* buf, int len) noexcept {
        std::vector<Frame> out;
        for (int i = 0; i < len; ++i) {
            auto f = parse_byte(static_cast<uint8_t>(buf[i]));
            if (f) out.push_back(std::move(*f));
        }
        return out;
    }

private:
    State   _s{State::IDLE};
    Frame   _f{};
    uint8_t _crc{};
    uint16_t _size{}, _idx{};
};

// ---------------------------------------------------------------------------
// Payload structs (little-endian)
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
struct RawImuPayload {
    int16_t accx{}, accy{}, accz{};   // 1/512 g
    int16_t gyrx{}, gyry{}, gyrz{};   // 1/16.4 deg/s (LSB from 2000dps gyro)
    int16_t magx{}, magy{}, magz{};   // raw
};

struct AttitudePayload {
    int16_t angx{}; // roll  × 10 deg
    int16_t angy{}; // pitch × 10 deg
    int16_t heading{}; // yaw deg
};

struct AltitudePayload {
    int32_t alt_cm{};          // cm MSL
    int16_t vario_cms{};       // cm/s
    float   baro_alt_m{};
};

struct MotorPayload {
    uint16_t motor[8]{};       // 1000–2000 µs PWM equivalents
};

struct ApiVersionPayload {
    uint8_t protocol{0};
    uint8_t api_major{1};
    uint8_t api_minor{42};
};

struct FcVariantPayload {
    char     identifier[4]{'B','T','F','L'};
};

struct FcVersionPayload {
    uint8_t major{4};
    uint8_t minor{4};
    uint8_t patch{0};
};
#pragma pack(pop)

} // namespace dronesim::sitl::msp
