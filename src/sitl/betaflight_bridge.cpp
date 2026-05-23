#include "sitl/firmware_bridge.hpp"
#include "sitl/msp/msp_types.hpp"
#include <cmath>
#include <cstring>

using namespace dronesim;
using namespace dronesim::sitl;
using namespace dronesim::sitl::msp;

// ---------------------------------------------------------------------------
bool BetaflightBridge::tick(const SimState& s, ActuatorOutput& out) noexcept {
    // TCP client — attempt reconnect each tick if not connected
    if (!_transport.is_connected()) {
        _transport.try_reconnect();
        return false;
    }

    _last_state = s;

    // Receive MSP requests from Betaflight (it polls us)
    int n = _transport.recv(_recv_buf, sizeof(_recv_buf));
    if (n <= 0) return false;

    bool got_motor = false;
    for (auto& frame : _parser.parse(_recv_buf, n)) {
        if (frame.direction == '<') {
            // Betaflight sent a request → we respond
            if (_handle_request(frame, s)) {
                // MSP_SET_MOTOR is a command (no response expected) — it sets motors
                if (frame.function == MSP_SET_MOTOR &&
                    frame.payload.size() >= sizeof(MotorPayload))
                {
                    MotorPayload mp{};
                    std::memcpy(&mp, frame.payload.data(), sizeof(mp));
                    out.n_channels = 4; // quad
                    out.source = ActuatorOutput::Source::Betaflight;
                    for (int i = 0; i < 4; ++i) {
                        out.pwm_us[i]   = mp.motor[i];
                        out.channels[i] = pwm_to_norm(mp.motor[i], 1000, 2000);
                    }
                    got_motor = true;
                }
            }
        }
    }
    return got_motor;
}

// ---------------------------------------------------------------------------
// Respond to Betaflight MSP poll requests
// Returns true if the frame was a SET command (motors etc.)
// ---------------------------------------------------------------------------
bool BetaflightBridge::_handle_request(const msp::Frame& req, const SimState& s) noexcept {
    switch (req.function) {

    case MSP_API_VERSION: {
        ApiVersionPayload p{};
        _transport.send(make_response(MSP_API_VERSION, p));
        break;
    }
    case MSP_FC_VARIANT: {
        FcVariantPayload p{};
        _transport.send(make_response(MSP_FC_VARIANT, p));
        break;
    }
    case MSP_FC_VERSION: {
        FcVersionPayload p{};
        _transport.send(make_response(MSP_FC_VERSION, p));
        break;
    }

    case MSP_RAW_IMU: {
        RawImuPayload p{};
        // Convert m/s² to raw ADC units (±8g range, 512 LSB/g)
        const double accel_lsb = 512.0 / 9.80665;
        p.accx = static_cast<int16_t>(s.accel_ms2.x * accel_lsb);
        p.accy = static_cast<int16_t>(s.accel_ms2.y * accel_lsb);
        p.accz = static_cast<int16_t>(s.accel_ms2.z * accel_lsb);
        // Convert rad/s to raw ADC units (±2000 dps, 16.4 LSB/dps)
        const double gyro_lsb = 16.4 * (180.0 / PI);
        p.gyrx = static_cast<int16_t>(s.gyro_rads.x * gyro_lsb);
        p.gyry = static_cast<int16_t>(s.gyro_rads.y * gyro_lsb);
        p.gyrz = static_cast<int16_t>(s.gyro_rads.z * gyro_lsb);
        _transport.send(make_response(MSP_RAW_IMU, p));
        break;
    }

    case MSP_ATTITUDE: {
        AttitudePayload p{};
        Vec3d rpy = s.orientation.to_euler_rpy();
        p.angx    = static_cast<int16_t>(rpy.x * RAD2DEG * 10.0); // ×10 deg
        p.angy    = static_cast<int16_t>(rpy.y * RAD2DEG * 10.0);
        // Heading: convert yaw to 0-360
        double hdg = rpy.z * RAD2DEG;
        if (hdg < 0) hdg += 360.0;
        p.heading = static_cast<int16_t>(hdg);
        _transport.send(make_response(MSP_ATTITUDE, p));
        break;
    }

    case MSP_ALTITUDE: {
        AltitudePayload p{};
        p.alt_cm    = static_cast<int32_t>(s.alt_msl_m * 100.0);
        p.vario_cms = static_cast<int16_t>(-s.vel_ned_ms.z * 100.0); // NED vd→up
        p.baro_alt_m = static_cast<float>(s.pressure_alt_m);
        _transport.send(make_response(MSP_ALTITUDE, p));
        break;
    }

    case MSP_RC: {
        // Return neutral RC values so arming conditions pass
        uint8_t rc_buf[32]{};
        // 8 channels at 1500 µs (uint16_t each)
        for (int i = 0; i < 8; ++i) {
            uint16_t v = (i == 2) ? 1000 : 1500; // throttle low
            rc_buf[i*2]   = static_cast<uint8_t>(v & 0xFF);
            rc_buf[i*2+1] = static_cast<uint8_t>(v >> 8);
        }
        _transport.send(make_response(MSP_RC, rc_buf, 16));
        break;
    }

    case MSP_STATUS: {
        // 11-byte minimal status (armingFlags=0 → disarmed for initial handshake)
        uint8_t st[11]{};
        uint16_t cycle = 125; // 8 kHz loop in µs
        std::memcpy(st, &cycle, 2);
        _transport.send(make_response(MSP_STATUS, st, sizeof(st)));
        break;
    }

    case MSP_SET_MOTOR:
        // SET_MOTOR is parsed in tick() — no response needed
        return true;

    default:
        // Send empty response for unrecognised requests
        _transport.send(make_response(req.function, nullptr, 0));
        break;
    }
    return false;
}
