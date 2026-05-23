#include "sitl/firmware_bridge.hpp"
#include "sitl/mavlink/mavlink_types.hpp"
#include <cmath>
#include <cstring>

using namespace dronesim;
using namespace dronesim::sitl;
using namespace dronesim::sitl::mavlink;

// ---------------------------------------------------------------------------
bool ArduPilotBridge::tick(const SimState& s, ActuatorOutput& out) noexcept {
    if (!_transport.is_connected()) return false;

    // ---- Send sensor data to ArduPilot ------------------------------------
    _send_hil_sensor(s);
    if ((_tick % GPS_DIVIDER) == 0) _send_hil_gps(s);
    if ((_tick % HB_DIVIDER)  == 0) _send_heartbeat();
    ++_tick;

    // ---- Receive SERVO_OUTPUT_RAW -----------------------------------------
    int n = _transport.recv(_recv_buf, sizeof(_recv_buf));
    if (n <= 0) return false;

    bool got_actuator = false;
    for (auto& frame : _parser.parse(_recv_buf, n)) {
        if (frame.msg_id == MSG_ID_SERVO_OUTPUT_RAW) {
            got_actuator = _parse_servo_output(frame, out);
        }
        // Swallow HEARTBEAT, STATUSTEXT, etc. silently
    }
    return got_actuator;
}

// ---------------------------------------------------------------------------
void ArduPilotBridge::_send_hil_sensor(const SimState& s) noexcept {
    HilSensorPayload p{};
    p.time_usec     = s.timestamp_us;

    // Body-frame accelerometer (m/s²)
    p.xacc = static_cast<float>(s.accel_ms2.x);
    p.yacc = static_cast<float>(s.accel_ms2.y);
    p.zacc = static_cast<float>(s.accel_ms2.z);

    // Body-frame gyroscope (rad/s)
    p.xgyro = static_cast<float>(s.gyro_rads.x);
    p.ygyro = static_cast<float>(s.gyro_rads.y);
    p.zgyro = static_cast<float>(s.gyro_rads.z);

    // Magnetometer in body frame (Gauss — AP expects Gauss)
    // For SITL we rotate NED mag to body (simplified — no declination)
    p.xmag = static_cast<float>(s.mag_ut.x * 0.01f); // µT → mGauss→Gauss rough
    p.ymag = static_cast<float>(s.mag_ut.y * 0.01f);
    p.zmag = static_cast<float>(s.mag_ut.z * 0.01f);

    p.abs_pressure  = static_cast<float>(s.pressure_hpa);
    p.diff_pressure = 0.5f * 1.225f * static_cast<float>(
        s.vel_ned_ms.x*s.vel_ned_ms.x +
        s.vel_ned_ms.y*s.vel_ned_ms.y) / 100.0f; // dynamic q in hPa
    p.pressure_alt  = static_cast<float>(s.pressure_alt_m);
    p.temperature   = static_cast<float>(s.temperature_c);
    p.fields_updated = 0x1FFF;

    auto buf = make_hil_sensor(p, _seq);
    _transport.send(buf);
}

// ---------------------------------------------------------------------------
void ArduPilotBridge::_send_hil_gps(const SimState& s) noexcept {
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
    p.cog = static_cast<uint16_t>(
        std::atan2(s.vel_ned_ms.y, s.vel_ned_ms.x) * RAD2DEG * 100.0 + 36000.0) % 36000;
    p.satellites_visible = static_cast<uint8_t>(s.sats);

    auto buf = make_hil_gps(p, _seq);
    _transport.send(buf);
}

// ---------------------------------------------------------------------------
void ArduPilotBridge::_send_hil_state(const SimState& s) noexcept {
    HilStateQuaternionPayload p{};
    p.time_usec = s.timestamp_us;
    // MAVLink quaternion convention: w, x, y, z
    p.attitude_quaternion[0] = static_cast<float>(s.orientation.w);
    p.attitude_quaternion[1] = static_cast<float>(s.orientation.x);
    p.attitude_quaternion[2] = static_cast<float>(s.orientation.y);
    p.attitude_quaternion[3] = static_cast<float>(s.orientation.z);
    p.rollspeed  = static_cast<float>(s.angular_vel_rads.x);
    p.pitchspeed = static_cast<float>(s.angular_vel_rads.y);
    p.yawspeed   = static_cast<float>(s.angular_vel_rads.z);
    // NED position expressed as absolute lat/lon/alt
    p.lat = static_cast<int32_t>(s.lat_deg * 1e7);
    p.lon = static_cast<int32_t>(s.lon_deg * 1e7);
    p.alt = static_cast<int32_t>(s.alt_msl_m * 1000.0);
    p.vx  = static_cast<int16_t>(s.vel_ned_ms.x * 100.0);
    p.vy  = static_cast<int16_t>(s.vel_ned_ms.y * 100.0);
    p.vz  = static_cast<int16_t>(s.vel_ned_ms.z * 100.0);
    p.xacc = static_cast<int16_t>(s.accel_ms2.x / 9.80665 * 1000.0);
    p.yacc = static_cast<int16_t>(s.accel_ms2.y / 9.80665 * 1000.0);
    p.zacc = static_cast<int16_t>(s.accel_ms2.z / 9.80665 * 1000.0);

    auto buf = make_hil_state_quaternion(p, _seq);
    _transport.send(buf);
}

// ---------------------------------------------------------------------------
void ArduPilotBridge::_send_heartbeat() noexcept {
    auto buf = make_heartbeat(_seq);
    _transport.send(buf);
}

// ---------------------------------------------------------------------------
bool ArduPilotBridge::_parse_servo_output(
    const mavlink::Frame& f, ActuatorOutput& out) const noexcept
{
    if (f.payload_len < sizeof(ServoOutputRawPayload)) return false;

    ServoOutputRawPayload p{};
    std::memcpy(&p, f.payload.data(), sizeof(p));

    const uint16_t channels[8] = {
        p.servo1_raw, p.servo2_raw, p.servo3_raw, p.servo4_raw,
        p.servo5_raw, p.servo6_raw, p.servo7_raw, p.servo8_raw
    };
    out.n_channels = 8;
    out.source = ActuatorOutput::Source::ArduPilot;
    for (int i = 0; i < 8; ++i) {
        out.pwm_us[i]   = channels[i];
        out.channels[i] = pwm_to_norm(channels[i], 1100, 1940);
    }
    return true;
}
