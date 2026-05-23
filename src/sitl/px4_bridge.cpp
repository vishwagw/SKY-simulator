#include "sitl/firmware_bridge.hpp"
#include "sitl/mavlink/mavlink_types.hpp"
#include <cmath>
#include <cstring>

using namespace dronesim;
using namespace dronesim::sitl;
using namespace dronesim::sitl::mavlink;

// ---------------------------------------------------------------------------
bool PX4Bridge::tick(const SimState& s, ActuatorOutput& out) noexcept {
    if (!_transport.is_connected()) return false;

    _send_hil_sensor(s);
    if ((_tick % GPS_DIVIDER) == 0) _send_hil_gps(s);
    if ((_tick % HB_DIVIDER)  == 0) _send_heartbeat();
    ++_tick;

    int n = _transport.recv(_recv_buf, sizeof(_recv_buf));
    if (n <= 0) return false;

    bool got_actuator = false;
    for (auto& frame : _parser.parse(_recv_buf, n)) {
        if (frame.msg_id == MSG_ID_HIL_ACTUATOR_CONTROLS) {
            got_actuator = _parse_hil_actuator_controls(frame, out);
        }
    }
    return got_actuator;
}

// ---------------------------------------------------------------------------
// PX4 HIL_SENSOR — identical wire format to ArduPilot; different timing
// PX4 expects ~250 Hz updates for the IMU loop to stay healthy
// ---------------------------------------------------------------------------
void PX4Bridge::_send_hil_sensor(const SimState& s) noexcept {
    HilSensorPayload p{};
    p.time_usec = s.timestamp_us;

    p.xacc  = static_cast<float>(s.accel_ms2.x);
    p.yacc  = static_cast<float>(s.accel_ms2.y);
    p.zacc  = static_cast<float>(s.accel_ms2.z);
    p.xgyro = static_cast<float>(s.gyro_rads.x);
    p.ygyro = static_cast<float>(s.gyro_rads.y);
    p.zgyro = static_cast<float>(s.gyro_rads.z);

    // PX4 HIL uses SI: mag in Gauss
    p.xmag = static_cast<float>(s.mag_ut.x * 1e-4f);
    p.ymag = static_cast<float>(s.mag_ut.y * 1e-4f);
    p.zmag = static_cast<float>(s.mag_ut.z * 1e-4f);

    p.abs_pressure  = static_cast<float>(s.pressure_hpa);
    p.diff_pressure = 0.0f;
    p.pressure_alt  = static_cast<float>(s.pressure_alt_m);
    p.temperature   = static_cast<float>(s.temperature_c);

    // PX4 uses fields_updated bitmask to know what changed:
    // bits: gyro=0x7, accel=0x38, mag=0x1C0, baro=0xE00, diff=0x7000
    p.fields_updated = (1 << 0) | (1 << 1) | (1 << 2)   // gyro
                     | (1 << 3) | (1 << 4) | (1 << 5)   // accel
                     | (1 << 9) | (1 << 10)| (1 << 11)  // baro
                     | (1 << 6) | (1 << 7) | (1 << 8);  // mag

    auto buf = make_hil_sensor(p, _seq);
    _transport.send(buf);
}

// ---------------------------------------------------------------------------
void PX4Bridge::_send_hil_gps(const SimState& s) noexcept {
    HilGpsPayload p{};
    p.time_usec = s.timestamp_us;
    p.fix_type  = static_cast<uint8_t>(s.fix_type);
    p.lat = static_cast<int32_t>(s.lat_deg * 1e7);
    p.lon = static_cast<int32_t>(s.lon_deg * 1e7);
    p.alt = static_cast<int32_t>(s.alt_msl_m * 1000.0);
    p.eph = static_cast<uint16_t>(s.eph_m * 100.0);
    p.epv = static_cast<uint16_t>(s.epv_m * 100.0);
    double gs = std::sqrt(s.vel_ned_ms.x*s.vel_ned_ms.x +
                          s.vel_ned_ms.y*s.vel_ned_ms.y);
    p.vel = static_cast<uint16_t>(gs * 100.0);
    p.vn  = static_cast<int16_t>(s.vel_ned_ms.x * 100.0);
    p.ve  = static_cast<int16_t>(s.vel_ned_ms.y * 100.0);
    p.vd  = static_cast<int16_t>(s.vel_ned_ms.z * 100.0);
    p.satellites_visible = 12;

    auto buf = make_hil_gps(p, _seq);
    _transport.send(buf);
}

// ---------------------------------------------------------------------------
void PX4Bridge::_send_heartbeat() noexcept {
    auto buf = make_heartbeat(_seq);
    _transport.send(buf);
}

// ---------------------------------------------------------------------------
// PX4 sends HIL_ACTUATOR_CONTROLS with 16 float channels in [-1, +1].
// Mapping to [0,1] throttle:  motor channels are in [0, 1] in PX4 SITL.
// ---------------------------------------------------------------------------
bool PX4Bridge::_parse_hil_actuator_controls(
    const mavlink::Frame& f, ActuatorOutput& out) const noexcept
{
    if (f.payload_len < sizeof(HilActuatorControlsPayload)) return false;

    HilActuatorControlsPayload p{};
    std::memcpy(&p, f.payload.data(), sizeof(p));

    out.n_channels = 8;
    out.source = ActuatorOutput::Source::PX4;
    for (int i = 0; i < 8; ++i) {
        // PX4 motor outputs are [0,1] in HIL mode (not [-1,+1] as for servos)
        double v = static_cast<double>(p.controls[i]);
        out.channels[i] = (v < 0.0) ? 0.0 : (v > 1.0 ? 1.0 : v);
        // Back-convert to PWM for logging / display
        out.pwm_us[i] = static_cast<uint16_t>(1100 + out.channels[i] * 840);
    }
    return true;
}
