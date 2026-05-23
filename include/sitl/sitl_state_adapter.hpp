#pragma once
#include "sitl/sitl_state.hpp"
#include "core/math_types.hpp"
#include "aero/atmosphere.hpp"
#include "sensors/sensor_suite.hpp"
#include <chrono>
#include <cmath>

namespace dronesim::sitl {

// ---------------------------------------------------------------------------
// SITLStateAdapter
//
// Translates the Godot/physics-engine body state (from DroneBody) into
// a SimState ready to be sent to firmware bridges.
// Also accumulates GPS origin offset for converting world-frame positions
// to absolute lat/lon/alt.
// ---------------------------------------------------------------------------
class SITLStateAdapter {
public:
    struct OriginConfig {
        double lat_deg{-35.363261};   // ArduPilot/PX4 default (Canberra RAAF)
        double lon_deg{149.165230};
        double alt_msl_m{584.0};
    };

    explicit SITLStateAdapter(OriginConfig origin = {}) noexcept
        : _origin(origin)
        , _epoch(std::chrono::steady_clock::now())
    {}

    // -----------------------------------------------------------------------
    // Primary conversion — call every physics tick
    // body_vel_world: world-frame velocity (Godot Y-up)
    // accel_body: true linear acceleration in body frame (from BET wrench/mass)
    // -----------------------------------------------------------------------
    [[nodiscard]] SimState build(
        const RigidBodyState& body,
        const Vec3d&          accel_body_ms2,     // body frame, no gravity removal
        const Vec3d&          wind_world,
        const Atmosphere&     atm,
        const IMUReading&     imu,
        const BaroReading&    baro,
        const std::optional<GPSReading>& gps_opt
    ) noexcept {
        SimState s;

        // Timestamp
        auto now = std::chrono::steady_clock::now();
        s.timestamp_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - _epoch).count());

        // ---- IMU (already in body frame from SensorSuite) -----------------
        s.accel_ms2 = imu.accel_body;
        s.gyro_rads = imu.gyro_body;

        // ---- Atmosphere ---------------------------------------------------
        double alt_world = body.position.y; // Godot Y-up
        const AtmosphericState atm_s = atm.at_altitude(alt_world);
        s.pressure_hpa  = atm_s.pressure / 100.0;
        s.temperature_c = atm_s.temperature - 273.15;
        s.pressure_alt_m = baro.altitude_m;

        // ---- GPS ----------------------------------------------------------
        // Convert world-frame NED displacement to absolute lat/lon
        // (flat-earth approximation valid within ~100 km of origin)
        //  Δlat ≈ Δnorth / R_earth
        //  Δlon ≈ Δeast  / (R_earth · cos(lat))
        const double R_earth = 6371000.0;
        const double lat_rad = _origin.lat_deg * DEG2RAD;
        // World frame: X = east, Y = up, Z = south (Godot right-handed Y-up)
        // NED: N = -Z_world, E = +X_world, D = -Y_world
        double delta_north = -body.position.z;
        double delta_east  =  body.position.x;
        double delta_down  = -body.position.y;  // above MSL

        s.lat_deg   = _origin.lat_deg + (delta_north / R_earth) * RAD2DEG;
        s.lon_deg   = _origin.lon_deg + (delta_east  / (R_earth * std::cos(lat_rad))) * RAD2DEG;
        s.alt_msl_m = _origin.alt_msl_m - delta_down;

        // NED velocity
        s.vel_ned_ms.x = -body.velocity.z; // N = -Z_world
        s.vel_ned_ms.y =  body.velocity.x; // E = +X_world
        s.vel_ned_ms.z = -body.velocity.y; // D = -Y_world

        // GPS noise / fix
        if (gps_opt) {
            s.fix_type = gps_opt->fix_type;
            s.sats     = 12;
        } else {
            s.fix_type = 3; // still report 3D fix between updates
        }
        s.eph_m = 0.5;
        s.epv_m = 1.0;

        // ---- Attitude (body orientation) ----------------------------------
        s.orientation      = body.orientation;
        s.angular_vel_rads = body.angular_velocity;
        s.position_ned_m   = { delta_north, delta_east, -body.position.y };

        // ---- Airspeed -----------------------------------------------------
        Vec3d vel_air = body.velocity - wind_world;
        s.true_airspeed_ms = vel_air.norm();

        // ---- Mag (simple NED → body rotation) ----------------------------
        // Hardcoded mid-latitude vertical component (pointing down in NED)
        Vec3d mag_ned{0.0, 0.0, -50.0}; // µT, roughly correct downward
        // Rotate to body frame: mag_body = R_world_to_body · mag_ned_as_world
        // Convert NED to world (Y-up): world_x = ned_e, world_y = -ned_d, world_z = -ned_n
        Vec3d mag_world{ mag_ned.y, -mag_ned.z, -mag_ned.x };
        s.mag_ut = body.orientation.conjugate().rotate(mag_world);

        return s;
    }

    // Shift GPS origin (call when sim resets position)
    void set_origin(OriginConfig o) noexcept { _origin = o; }
    const OriginConfig& origin() const noexcept { return _origin; }

private:
    OriginConfig _origin;
    std::chrono::steady_clock::time_point _epoch;
};

} // namespace dronesim::sitl
