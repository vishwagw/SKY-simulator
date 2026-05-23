#pragma once

// Godot headers
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/core/class_db.hpp>

// Simulator subsystems
#include "core/math_types.hpp"
#include "aero/atmosphere.hpp"
#include "aero/blade_element.hpp"
#include "aero/aero_effects.hpp"
#include "control/flight_controller.hpp"
#include "sensors/sensor_suite.hpp"

#include <array>
#include <memory>

namespace dronesim {

// -------------------------------------------------------------------------
// Telemetry snapshot — exported as a Dictionary to GDScript / C#
// -------------------------------------------------------------------------
struct TelemetryFrame {
    double altitude{};
    double ground_speed{};
    double vertical_speed{};
    double roll_deg{}, pitch_deg{}, yaw_deg{};
    double roll_rate{}, pitch_rate{}, yaw_rate{};
    double total_thrust{};
    double battery_voltage{};
    double power_draw{};
    bool   vrs_active{};
    double vrs_severity{};
    double ground_effect_factor{};
    double air_density{};
    Vec3d  wind_world{};
};

// -------------------------------------------------------------------------
// DroneBody
//
// Exposes the following properties to the Godot editor:
//   - rotor_radius, n_blades, motor_kv, max_voltage
//   - pid_rate_roll/pitch/yaw (p, i, d)
//   - turbulence_intensity, wind_x/y/z
//   - mass (mirrors RigidBody3D.mass)
//
// Control inputs (call from GDScript or networking layer):
//   set_attitude_setpoint(roll, pitch, yaw_rate, throttle)
//   set_rotor_throttles(PackedFloat64Array)  -- bypass FC for direct control
// -------------------------------------------------------------------------
class DroneBody : public godot::RigidBody3D {
    GDCLASS(DroneBody, godot::RigidBody3D)

public:
    DroneBody();
    ~DroneBody() override = default;

    // -----------------------------------------------------------------------
    // Godot lifecycle
    // -----------------------------------------------------------------------
    void _ready()                           override;
    void _physics_process(double delta)     override;
    void _integrate_forces(
        godot::PhysicsDirectBodyState3D* state) override;

    // -----------------------------------------------------------------------
    // GDScript-callable control API
    // -----------------------------------------------------------------------
    void set_attitude_setpoint(double roll, double pitch,
                               double yaw_rate, double throttle);
    void set_rotor_throttles(godot::PackedFloat64Array throttles);
    void arm();
    void disarm();

    // -----------------------------------------------------------------------
    // Property accessors (wired to GDCLASS binding)
    // -----------------------------------------------------------------------
    void   set_rotor_radius(double r);
    double get_rotor_radius() const;

    void   set_n_rotors(int n);
    int    get_n_rotors() const;

    void   set_motor_kv(double kv);
    double get_motor_kv() const;

    void   set_max_voltage(double v);
    double get_max_voltage() const;

    void   set_turbulence_intensity(double i);
    double get_turbulence_intensity() const;

    void   set_wind(godot::Vector3 w);
    godot::Vector3 get_wind() const;

    void   set_ground_height(double h);
    double get_ground_height() const;

    // Returns a Dictionary of telemetry for HUD/logging
    godot::Dictionary get_telemetry() const;

    // PID gain setters
    void set_rate_roll_pid(double p, double i, double d);
    void set_rate_pitch_pid(double p, double i, double d);
    void set_rate_yaw_pid(double p, double i, double d);

    static void _bind_methods();

private:
    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------
    void _rebuild_rotors();
    void _build_quad_x_layout();

    [[nodiscard]] RigidBodyState _godot_to_sim_state(
        godot::PhysicsDirectBodyState3D* gstate) const noexcept;

    void _apply_wrench_to_godot(
        godot::PhysicsDirectBodyState3D* gstate,
        const Wrench& w) noexcept;

    [[nodiscard]] static Vec3d _gv3_to_sim(godot::Vector3 v) noexcept {
        return { static_cast<double>(v.x),
                 static_cast<double>(v.y),
                 static_cast<double>(v.z) };
    }
    [[nodiscard]] static godot::Vector3 _sim_to_gv3(Vec3d v) noexcept {
        return { static_cast<float>(v.x),
                 static_cast<float>(v.y),
                 static_cast<float>(v.z) };
    }

    // -----------------------------------------------------------------------
    // Subsystems
    // -----------------------------------------------------------------------
    Atmosphere                       _atm;
    std::unique_ptr<RotorArray>      _rotors;
    std::unique_ptr<AeroEffectsBundle> _effects;
    std::unique_ptr<FlightController> _fc;
    std::unique_ptr<MixerMatrix>     _mixer;
    std::unique_ptr<SensorSuite>     _sensors;

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------
    bool   _armed{false};
    FlightController::Setpoints _setpoints{};
    TelemetryFrame              _telem{};

    // Config cache (editor properties)
    double _rotor_radius{0.127};
    int    _n_rotors{4};
    double _motor_kv{920.0};
    double _max_voltage{14.8};
    double _turbulence_intensity{0.0};
    double _ground_height{0.0};
    Vec3d  _wind_world{};

    FlightController::Gains _fc_gains{};

    // Direct throttle override (bypass FC when set)
    bool                _direct_throttle_mode{false};
    std::vector<double> _direct_throttles;
};

} // namespace dronesim
