#pragma once
#include "core/math_types.hpp"
#include <array>
#include <cstdint>

namespace dronesim::sitl {

// ---------------------------------------------------------------------------
// SimState — what the sim sends to firmware (sensor readings)
// ---------------------------------------------------------------------------
struct SimState {
    uint64_t timestamp_us{};

    // IMU (body frame)
    Vec3d accel_ms2{};     // m/s² — includes gravity component
    Vec3d gyro_rads{};     // rad/s

    // Magnetometer (world NED frame, micro-Tesla)
    Vec3d mag_ut{0.0, 0.0, -50.0}; // approximate mid-lat downward component

    // Barometer
    double pressure_hpa{1013.25};
    double temperature_c{25.0};
    double pressure_alt_m{};

    // GPS (absolute position — zeroed at origin by default)
    double lat_deg{-35.363261};  // Canberra RAAF (ArduPilot default origin)
    double lon_deg{149.165230};
    double alt_msl_m{584.0};
    Vec3d  vel_ned_ms{};         // m/s
    int    fix_type{3};
    int    sats{12};
    double eph_m{0.5};
    double epv_m{1.0};

    // Attitude (for HIL_STATE_QUATERNION)
    Quat   orientation{};
    Vec3d  angular_vel_rads{};
    Vec3d  position_ned_m{};

    // Airspeed (derived)
    double true_airspeed_ms{};
};

// ---------------------------------------------------------------------------
// ActuatorOutput — what firmware sends back to the sim
// ---------------------------------------------------------------------------
struct ActuatorOutput {
    // Normalised [0,1] per motor — sim maps to omega_cmd
    static constexpr int MAX_CHANNELS = 16;
    std::array<double, MAX_CHANNELS> channels{};
    int n_channels{4};

    // Raw µs PWM values (from Betaflight MSP or SERVO_OUTPUT_RAW)
    std::array<uint16_t, MAX_CHANNELS> pwm_us{};

    // Which firmware provided this
    enum class Source { ArduPilot, PX4, Betaflight, None } source{Source::None};
};

// ---------------------------------------------------------------------------
// Helper — PWM µs → normalised [0,1]
// pwm range 1100–1940 µs is typical BLHeli range
// ---------------------------------------------------------------------------
inline double pwm_to_norm(uint16_t pwm_us,
                          uint16_t min_us = 1100,
                          uint16_t max_us = 1940) noexcept
{
    if (pwm_us <= min_us) return 0.0;
    if (pwm_us >= max_us) return 1.0;
    return static_cast<double>(pwm_us - min_us) /
           static_cast<double>(max_us - min_us);
}

} // namespace dronesim::sitl
