#include "sitl/sitl_manager.hpp"
#include <godot_cpp/variant/dictionary.hpp>
#include <chrono>

using namespace godot;
using namespace dronesim;
using namespace dronesim::sitl;

// ============================================================================
SITLManager::SITLManager() : _adapter(_origin) {}

SITLManager::~SITLManager() {
    if (_ap)  _ap->disconnect();
    if (_px4) _px4->disconnect();
    if (_bf)  _bf->disconnect();
}

// ============================================================================
void SITLManager::_ready() {
    _build_bridges();
}

void SITLManager::_process(double /*delta*/) {
    // Reconnect TCP bridges that dropped
    if (_bf && _en_bf && !_bf->is_connected())
        _bf->connect();
}

// ============================================================================
bool SITLManager::physics_tick(
    const RigidBodyState& body,
    const Vec3d&          accel_body,
    const Vec3d&          wind_world,
    const Atmosphere&     atm,
    const IMUReading&     imu,
    const BaroReading&    baro,
    const std::optional<GPSReading>& gps,
    ActuatorOutput&       out
) noexcept {
    if (!_built) return false;

    // Build firmware state packet
    SimState state = _adapter.build(
        body, accel_body, wind_world, atm, imu, baro, gps);

    auto now_ms = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    bool got_any = false;

    // Betaflight first (highest priority / most latency-sensitive)
    if (_bf && _en_bf) {
        ActuatorOutput bf_out;
        if (_bf->tick(state, bf_out)) {
            _last_rx_bf = now_ms;
            if (_forced_firmware == -1 || _forced_firmware == 2) {
                out = bf_out;
                got_any = true;
            }
        }
    }

    // PX4
    if (_px4 && _en_px4) {
        ActuatorOutput px4_out;
        if (_px4->tick(state, px4_out)) {
            _last_rx_px4 = now_ms;
            if ((_forced_firmware == -1 && !got_any) || _forced_firmware == 1) {
                out = px4_out;
                got_any = true;
            }
        }
    }

    // ArduPilot
    if (_ap && _en_ap) {
        ActuatorOutput ap_out;
        if (_ap->tick(state, ap_out)) {
            _last_rx_ap = now_ms;
            if ((_forced_firmware == -1 && !got_any) || _forced_firmware == 0) {
                out = ap_out;
                got_any = true;
            }
        }
    }

    return got_any;
}

// ============================================================================
void SITLManager::_build_bridges() {
    if (_ap)  { _ap->disconnect();  _ap.reset();  }
    if (_px4) { _px4->disconnect(); _px4.reset(); }
    if (_bf)  { _bf->disconnect();  _bf.reset();  }

    _adapter.set_origin(_origin);

    if (_en_ap) {
        auto cfg = _make_udp_config(
            static_cast<uint16_t>(_port_ap),
            static_cast<uint16_t>(_port_ap));
        _ap = std::make_unique<ArduPilotBridge>(cfg);
        _ap->connect();
    }

    if (_en_px4) {
        auto cfg = _make_udp_config(
            static_cast<uint16_t>(_port_px4),
            static_cast<uint16_t>(_port_px4));
        _px4 = std::make_unique<PX4Bridge>(cfg);
        _px4->connect();
    }

    if (_en_bf) {
        // Betaflight SITL listens on TCP; we connect as client
        auto cfg = _make_tcp_client_config(static_cast<uint16_t>(_port_bf));
        _bf = std::make_unique<BetaflightBridge>(cfg);
        _bf->connect(); // non-blocking; may not connect immediately
    }

    _built = true;
}

// ============================================================================
SocketTransport::Config SITLManager::_make_udp_config(
    uint16_t local_port, uint16_t remote_port) const noexcept
{
    SocketTransport::Config cfg;
    cfg.mode        = TransportMode::UDP;
    cfg.local_addr  = "0.0.0.0";
    cfg.local_port  = local_port;
    cfg.remote_addr = "127.0.0.1";
    cfg.remote_port = (remote_port == 0) ? local_port : remote_port;
    return cfg;
}

SocketTransport::Config SITLManager::_make_tcp_client_config(
    uint16_t remote_port) const noexcept
{
    SocketTransport::Config cfg;
    cfg.mode        = TransportMode::TCP;
    cfg.tcp_server  = false;
    cfg.remote_addr = "127.0.0.1";
    cfg.remote_port = remote_port;
    cfg.local_port  = 0;
    return cfg;
}

// ============================================================================
// GDScript API
// ============================================================================
void SITLManager::set_active_firmware(String name) {
    if (name == "ardupilot")   _forced_firmware = 0;
    else if (name == "px4")    _forced_firmware = 1;
    else if (name == "betaflight") _forced_firmware = 2;
    else                       _forced_firmware = -1; // auto
}

String SITLManager::get_active_firmware() const {
    switch (_forced_firmware) {
        case 0: return "ardupilot";
        case 1: return "px4";
        case 2: return "betaflight";
        default: return "auto";
    }
}

Dictionary SITLManager::get_sitl_status() const {
    auto now_ms = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    auto ms_since = [&](uint64_t t) -> int64_t {
        return (t == 0) ? -1 : static_cast<int64_t>(now_ms - t);
    };

    Dictionary d;
    // ArduPilot
    Dictionary ap;
    ap["connected"]   = _ap && _ap->is_connected();
    ap["port"]        = _port_ap;
    ap["last_rx_ms"]  = ms_since(_last_rx_ap);
    ap["enabled"]     = _en_ap;
    d["ardupilot"]    = ap;

    // PX4
    Dictionary px4;
    px4["connected"]  = _px4 && _px4->is_connected();
    px4["port"]       = _port_px4;
    px4["last_rx_ms"] = ms_since(_last_rx_px4);
    px4["enabled"]    = _en_px4;
    d["px4"]          = px4;

    // Betaflight
    Dictionary bf;
    bf["connected"]   = _bf && _bf->is_connected();
    bf["port"]        = _port_bf;
    bf["last_rx_ms"]  = ms_since(_last_rx_bf);
    bf["enabled"]     = _en_bf;
    d["betaflight"]   = bf;

    return d;
}

void SITLManager::reconnect_ardupilot()  { if (_ap)  { _ap->disconnect();  _build_bridges(); } }
void SITLManager::reconnect_px4()        { if (_px4) { _px4->disconnect(); _build_bridges(); } }
void SITLManager::reconnect_betaflight() { if (_bf)  { _bf->disconnect();  _build_bridges(); } }

// ============================================================================
// Property accessors
// ============================================================================
void SITLManager::set_enabled_ardupilot(bool v)  { _en_ap  = v; if (_built) _build_bridges(); }
bool SITLManager::get_enabled_ardupilot() const   { return _en_ap; }
void SITLManager::set_enabled_px4(bool v)         { _en_px4 = v; if (_built) _build_bridges(); }
bool SITLManager::get_enabled_px4() const          { return _en_px4; }
void SITLManager::set_enabled_betaflight(bool v)  { _en_bf  = v; if (_built) _build_bridges(); }
bool SITLManager::get_enabled_betaflight() const   { return _en_bf; }

void SITLManager::set_transport_mode(int m) { _transport_mode = m; }
int  SITLManager::get_transport_mode() const { return _transport_mode; }

void SITLManager::set_port_ardupilot(int p)  { _port_ap  = p; }
int  SITLManager::get_port_ardupilot() const  { return _port_ap; }
void SITLManager::set_port_px4(int p)         { _port_px4 = p; }
int  SITLManager::get_port_px4() const         { return _port_px4; }
void SITLManager::set_port_betaflight(int p)  { _port_bf  = p; }
int  SITLManager::get_port_betaflight() const  { return _port_bf; }

void SITLManager::set_gps_origin_lat(double v) { _origin.lat_deg  = v; }
double SITLManager::get_gps_origin_lat() const  { return _origin.lat_deg; }
void SITLManager::set_gps_origin_lon(double v) { _origin.lon_deg  = v; }
double SITLManager::get_gps_origin_lon() const  { return _origin.lon_deg; }
void SITLManager::set_gps_origin_alt(double v) { _origin.alt_msl_m = v; }
double SITLManager::get_gps_origin_alt() const  { return _origin.alt_msl_m; }

// ============================================================================
// Binding
// ============================================================================
void SITLManager::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_active_firmware","name"),  &SITLManager::set_active_firmware);
    ClassDB::bind_method(D_METHOD("get_active_firmware"),         &SITLManager::get_active_firmware);
    ClassDB::bind_method(D_METHOD("get_sitl_status"),             &SITLManager::get_sitl_status);
    ClassDB::bind_method(D_METHOD("reconnect_ardupilot"),         &SITLManager::reconnect_ardupilot);
    ClassDB::bind_method(D_METHOD("reconnect_px4"),               &SITLManager::reconnect_px4);
    ClassDB::bind_method(D_METHOD("reconnect_betaflight"),        &SITLManager::reconnect_betaflight);

    // Enable toggles
    ClassDB::bind_method(D_METHOD("set_enabled_ardupilot","v"),  &SITLManager::set_enabled_ardupilot);
    ClassDB::bind_method(D_METHOD("get_enabled_ardupilot"),      &SITLManager::get_enabled_ardupilot);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL,"enabled_ardupilot"),
                 "set_enabled_ardupilot","get_enabled_ardupilot");

    ClassDB::bind_method(D_METHOD("set_enabled_px4","v"),        &SITLManager::set_enabled_px4);
    ClassDB::bind_method(D_METHOD("get_enabled_px4"),            &SITLManager::get_enabled_px4);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL,"enabled_px4"),
                 "set_enabled_px4","get_enabled_px4");

    ClassDB::bind_method(D_METHOD("set_enabled_betaflight","v"), &SITLManager::set_enabled_betaflight);
    ClassDB::bind_method(D_METHOD("get_enabled_betaflight"),     &SITLManager::get_enabled_betaflight);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL,"enabled_betaflight"),
                 "set_enabled_betaflight","get_enabled_betaflight");

    // Ports
    ClassDB::bind_method(D_METHOD("set_port_ardupilot","p"),     &SITLManager::set_port_ardupilot);
    ClassDB::bind_method(D_METHOD("get_port_ardupilot"),         &SITLManager::get_port_ardupilot);
    ADD_PROPERTY(PropertyInfo(Variant::INT,"port_ardupilot",PROPERTY_HINT_RANGE,"1024,65535"),
                 "set_port_ardupilot","get_port_ardupilot");

    ClassDB::bind_method(D_METHOD("set_port_px4","p"),           &SITLManager::set_port_px4);
    ClassDB::bind_method(D_METHOD("get_port_px4"),               &SITLManager::get_port_px4);
    ADD_PROPERTY(PropertyInfo(Variant::INT,"port_px4",PROPERTY_HINT_RANGE,"1024,65535"),
                 "set_port_px4","get_port_px4");

    ClassDB::bind_method(D_METHOD("set_port_betaflight","p"),    &SITLManager::set_port_betaflight);
    ClassDB::bind_method(D_METHOD("get_port_betaflight"),        &SITLManager::get_port_betaflight);
    ADD_PROPERTY(PropertyInfo(Variant::INT,"port_betaflight",PROPERTY_HINT_RANGE,"1024,65535"),
                 "set_port_betaflight","get_port_betaflight");

    // GPS origin
    ClassDB::bind_method(D_METHOD("set_gps_origin_lat","v"), &SITLManager::set_gps_origin_lat);
    ClassDB::bind_method(D_METHOD("get_gps_origin_lat"),     &SITLManager::get_gps_origin_lat);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"gps_origin_lat"),
                 "set_gps_origin_lat","get_gps_origin_lat");

    ClassDB::bind_method(D_METHOD("set_gps_origin_lon","v"), &SITLManager::set_gps_origin_lon);
    ClassDB::bind_method(D_METHOD("get_gps_origin_lon"),     &SITLManager::get_gps_origin_lon);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"gps_origin_lon"),
                 "set_gps_origin_lon","get_gps_origin_lon");

    ClassDB::bind_method(D_METHOD("set_gps_origin_alt","v"), &SITLManager::set_gps_origin_alt);
    ClassDB::bind_method(D_METHOD("get_gps_origin_alt"),     &SITLManager::get_gps_origin_alt);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"gps_origin_alt"),
                 "set_gps_origin_alt","get_gps_origin_alt");
}
