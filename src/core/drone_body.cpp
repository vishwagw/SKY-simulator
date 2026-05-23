#include "core/drone_body.hpp"
#include <godot_cpp/classes/physics_direct_body_state3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <cmath>

using namespace godot;
using namespace dronesim;

// ============================================================================
// Construction / Godot lifecycle
// ============================================================================
DroneBody::DroneBody() {
    _rotors  = std::make_unique<RotorArray>();
    _effects = std::make_unique<AeroEffectsBundle>(_rotor_radius);
    _fc      = std::make_unique<FlightController>(_fc_gains);
    _mixer   = std::make_unique<MixerMatrix>(MixerMatrix::quad_x());
    _sensors = std::make_unique<SensorSuite>();
}

void DroneBody::_ready() {
    _rebuild_rotors();
    // Match RigidBody3D mass to our sim mass
    set_mass(static_cast<real_t>(1.5));
    // Disable Godot's built-in gravity — we handle it ourselves
    set_gravity_scale(0.0);
    // Enable continuous collision detection for small-body accuracy
    set_continuous_cd(true);
}

void DroneBody::_physics_process(double /*delta*/) {
    // Nothing in the GDScript tick — all physics happens in _integrate_forces
}

void DroneBody::_integrate_forces(PhysicsDirectBodyState3D* gstate) {
    const double dt = gstate->get_step();
    if (dt <= 0.0) return;

    // ---- Sample atmosphere --------------------------------------------------
    const double alt = static_cast<double>(gstate->get_transform().origin.y);
    _atm.set_turbulence({ _turbulence_intensity, 200.0 });
    _atm.set_wind_global(_wind_world);
    const Vec3d wind_w = _atm.sample_wind(alt, dt);

    // ---- Build sim state from Godot -----------------------------------------
    RigidBodyState body = _godot_to_sim_state(gstate);

    // ---- Flight controller --------------------------------------------------
    std::vector<double> throttles;
    if (_direct_throttle_mode) {
        throttles = _direct_throttles;
    } else if (_armed) {
        auto [tau_r, tau_p, tau_y, thr] = _fc->update(
            _setpoints, body.orientation, body.angular_velocity, dt);

        // Normalise torque demands (crude scaling — tune per airframe)
        const double torque_scale = 0.3;
        throttles = _mixer->mix(
            thr,
            tau_r * torque_scale,
            tau_p * torque_scale,
            tau_y * torque_scale
        );
    } else {
        throttles.assign(_rotors->size(), 0.0);
    }
    _rotors->set_throttles(throttles);

    // ---- Aerodynamics -------------------------------------------------------
    Wrench aero_wrench = _rotors->solve_all(body, _atm, wind_w, dt);

    // ---- Ground Effect ------------------------------------------------------
    const double h_agl = std::max(alt - _ground_height, 0.0);
    double ge_factor = _effects->ground_effect.effective_multiplier(h_agl);
    aero_wrench.force.y *= ge_factor;   // thrust boost near ground

    // ---- Vortex Ring State --------------------------------------------------
    const double descent_rate  = -body.velocity.y;  // positive = descending
    const double lateral_speed = std::sqrt(body.velocity.x*body.velocity.x
                                         + body.velocity.z*body.velocity.z);
    // Use first rotor's induced velocity as representative hover Vc
    double vc = 0.0;
    if (!_rotors->states().empty()) {
        const auto& rs = _rotors->states()[0];
        const AtmosphericState atm_s = _atm.at_altitude(alt);
        const double A = PI * _rotor_radius * _rotor_radius;
        vc = std::sqrt(std::max(rs.thrust, 0.0) / (2.0 * atm_s.density * A));
    }
    auto vrs = _effects->vrs.evaluate(vc, descent_rate, lateral_speed, dt);
    if (vrs.active) {
        aero_wrench.force.y *= vrs.thrust_factor;
    }

    // ---- Gravity (applied explicitly so we can account for density alt) -----
    const double g = 9.80665;
    Vec3d gravity_f{0, -g * body.mass, 0};
    aero_wrench.force += gravity_f;

    // ---- Aerodynamic body drag (linear + quadratic) -------------------------
    const AtmosphericState atm_s = _atm.at_altitude(alt);
    const double drag_lin  = 0.02;  // kg/s   — skin friction approx
    const double drag_quad = 0.15;  // kg/m    — form drag coefficient
    Vec3d vel_rel = body.velocity - wind_w;
    double v2     = vel_rel.norm2();
    if (v2 > 1e-6) {
        Vec3d drag_f = -(drag_lin + drag_quad * atm_s.density * std::sqrt(v2))
                       * vel_rel;
        aero_wrench.force += drag_f;
    }

    // ---- Apply to Godot physics engine --------------------------------------
    _apply_wrench_to_godot(gstate, aero_wrench);

    // ---- Sensor simulation --------------------------------------------------
    Vec3d accel_bf = body.orientation.conjugate().rotate(aero_wrench.force / body.mass);
    _sensors->imu.measure(accel_bf, body.angular_velocity, body.orientation, dt);
    _sensors->baro.measure(atm_s, alt, dt);
    _sensors->gps.measure(body.position, body.velocity, dt);

    // ---- Telemetry update ---------------------------------------------------
    Vec3d rpy = body.orientation.to_euler_rpy();
    _telem.altitude           = alt;
    _telem.vertical_speed     = body.velocity.y;
    _telem.ground_speed       = lateral_speed;
    _telem.roll_deg           = rpy.x * RAD2DEG;
    _telem.pitch_deg          = rpy.y * RAD2DEG;
    _telem.yaw_deg            = rpy.z * RAD2DEG;
    _telem.roll_rate          = body.angular_velocity.x;
    _telem.pitch_rate         = body.angular_velocity.y;
    _telem.yaw_rate           = body.angular_velocity.z;
    _telem.total_thrust       = aero_wrench.force.norm();
    _telem.vrs_active         = vrs.active;
    _telem.vrs_severity       = vrs.severity;
    _telem.ground_effect_factor = ge_factor;
    _telem.air_density        = atm_s.density;
    _telem.wind_world         = wind_w;

    double total_power = 0;
    for (const auto& rs : _rotors->states()) total_power += rs.power;
    _telem.power_draw = total_power;
    _telem.battery_voltage = _max_voltage; // placeholder — add battery model
}

// ============================================================================
// Control API
// ============================================================================
void DroneBody::arm()   { _armed = true;  _fc->reset(); _direct_throttle_mode = false; }
void DroneBody::disarm(){ _armed = false; }

void DroneBody::set_attitude_setpoint(double roll, double pitch,
                                      double yaw_rate, double throttle) {
    _setpoints.roll_rad      = clamp(roll,      -0.785, 0.785); // ±45°
    _setpoints.pitch_rad     = clamp(pitch,     -0.785, 0.785);
    _setpoints.yaw_rate_rads = clamp(yaw_rate,  -3.14,  3.14);
    _setpoints.thrust_norm   = clamp(throttle,   0.0,   1.0);
    _direct_throttle_mode    = false;
}

void DroneBody::set_rotor_throttles(PackedFloat64Array throttles) {
    _direct_throttles.resize(static_cast<size_t>(throttles.size()));
    for (int i = 0; i < throttles.size(); ++i)
        _direct_throttles[static_cast<size_t>(i)] = throttles[i];
    _direct_throttle_mode = true;
}

// ============================================================================
// Private helpers
// ============================================================================
void DroneBody::_rebuild_rotors() {
    _rotors = std::make_unique<RotorArray>();
    _effects = std::make_unique<AeroEffectsBundle>(_rotor_radius);

    if (_n_rotors == 4) {
        _build_quad_x_layout();
        _mixer = std::make_unique<MixerMatrix>(MixerMatrix::quad_x());
    } else if (_n_rotors == 6) {
        // Hex-X would go here — similar pattern
        _mixer = std::make_unique<MixerMatrix>(MixerMatrix::hex_x());
    }
}

void DroneBody::_build_quad_x_layout() {
    const double arm = _rotor_radius * 2.1;  // arm length heuristic
    struct MotorPlacement { double x; double z; int dir; };
    const MotorPlacement layout[4] = {
        { arm,  arm, -1},  // FL CCW
        { arm, -arm,  1},  // FR CW
        {-arm,  arm,  1},  // RL CW
        {-arm, -arm, -1},  // RR CCW
    };
    for (auto& p : layout) {
        RotorConfig cfg;
        cfg.radius    = _rotor_radius;
        cfg.motor_kv  = _motor_kv;
        cfg.max_voltage = _max_voltage;
        cfg.position  = { p.x, 0, p.z };
        cfg.spin_dir  = p.dir;
        _rotors->add_rotor(std::move(cfg));
    }
}

RigidBodyState DroneBody::_godot_to_sim_state(
    PhysicsDirectBodyState3D* gs) const noexcept
{
    RigidBodyState s;
    Transform3D xf = gs->get_transform();
    s.position = _gv3_to_sim(xf.origin);
    s.velocity = _gv3_to_sim(gs->get_linear_velocity());
    // Angular velocity from Godot is in local (body) frame
    s.angular_velocity = _gv3_to_sim(gs->get_angular_velocity());
    s.mass = static_cast<double>(get_mass());

    // Convert Godot Basis to quaternion
    Basis b = xf.basis;
    // Shepperd method
    double tr = b[0][0] + b[1][1] + b[2][2];
    if (tr > 0) {
        double s4 = std::sqrt(tr + 1.0) * 2;
        s.orientation = { (b[2][1]-b[1][2])/s4, (b[0][2]-b[2][0])/s4,
                          (b[1][0]-b[0][1])/s4, 0.25*s4 };
    } else if ((b[0][0]>b[1][1]) && (b[0][0]>b[2][2])) {
        double s4 = std::sqrt(1.0+b[0][0]-b[1][1]-b[2][2])*2;
        s.orientation = { 0.25*s4, (b[0][1]+b[1][0])/s4,
                          (b[0][2]+b[2][0])/s4, (b[2][1]-b[1][2])/s4 };
    } else if (b[1][1] > b[2][2]) {
        double s4 = std::sqrt(1.0+b[1][1]-b[0][0]-b[2][2])*2;
        s.orientation = { (b[0][1]+b[1][0])/s4, 0.25*s4,
                          (b[1][2]+b[2][1])/s4, (b[0][2]-b[2][0])/s4 };
    } else {
        double s4 = std::sqrt(1.0+b[2][2]-b[0][0]-b[1][1])*2;
        s.orientation = { (b[0][2]+b[2][0])/s4, (b[1][2]+b[2][1])/s4,
                          0.25*s4, (b[1][0]-b[0][1])/s4 };
    }
    s.orientation = s.orientation.normalized();
    return s;
}

void DroneBody::_apply_wrench_to_godot(
    PhysicsDirectBodyState3D* gs, const Wrench& w) noexcept
{
    // Forces in body frame — rotate to world
    RigidBodyState body = _godot_to_sim_state(gs);
    Vec3d force_world  = body.orientation.rotate(w.force);
    Vec3d torque_world = body.orientation.rotate(w.torque);

    gs->apply_central_force(_sim_to_gv3(force_world));
    gs->apply_torque(_sim_to_gv3(torque_world));
}

// ============================================================================
// Dictionary telemetry export
// ============================================================================
Dictionary DroneBody::get_telemetry() const {
    Dictionary d;
    d["altitude"]             = _telem.altitude;
    d["ground_speed"]         = _telem.ground_speed;
    d["vertical_speed"]       = _telem.vertical_speed;
    d["roll_deg"]             = _telem.roll_deg;
    d["pitch_deg"]            = _telem.pitch_deg;
    d["yaw_deg"]              = _telem.yaw_deg;
    d["roll_rate"]            = _telem.roll_rate;
    d["pitch_rate"]           = _telem.pitch_rate;
    d["yaw_rate"]             = _telem.yaw_rate;
    d["total_thrust"]         = _telem.total_thrust;
    d["power_draw"]           = _telem.power_draw;
    d["vrs_active"]           = _telem.vrs_active;
    d["vrs_severity"]         = _telem.vrs_severity;
    d["ground_effect_factor"] = _telem.ground_effect_factor;
    d["air_density"]          = _telem.air_density;
    d["wind"]                 = _sim_to_gv3(_telem.wind_world);
    return d;
}

// ============================================================================
// Property accessors
// ============================================================================
void   DroneBody::set_rotor_radius(double r)  { _rotor_radius = r; if (is_inside_tree()) _rebuild_rotors(); }
double DroneBody::get_rotor_radius() const     { return _rotor_radius; }

void   DroneBody::set_n_rotors(int n)          { _n_rotors = n; if (is_inside_tree()) _rebuild_rotors(); }
int    DroneBody::get_n_rotors() const         { return _n_rotors; }

void   DroneBody::set_motor_kv(double kv)      { _motor_kv = kv; if (is_inside_tree()) _rebuild_rotors(); }
double DroneBody::get_motor_kv() const         { return _motor_kv; }

void   DroneBody::set_max_voltage(double v)    { _max_voltage = v; if (is_inside_tree()) _rebuild_rotors(); }
double DroneBody::get_max_voltage() const      { return _max_voltage; }

void   DroneBody::set_turbulence_intensity(double i) { _turbulence_intensity = std::max(0.0,i); }
double DroneBody::get_turbulence_intensity() const   { return _turbulence_intensity; }

void   DroneBody::set_wind(Vector3 w)          { _wind_world = _gv3_to_sim(w); }
Vector3 DroneBody::get_wind() const            { return _sim_to_gv3(_wind_world); }

void   DroneBody::set_ground_height(double h)  { _ground_height = h; }
double DroneBody::get_ground_height() const    { return _ground_height; }

void DroneBody::set_rate_roll_pid(double p, double i, double d) {
    _fc_gains.rate_roll_p = p; _fc_gains.rate_roll_i = i; _fc_gains.rate_roll_d = d;
    _fc->set_gains(_fc_gains);
}
void DroneBody::set_rate_pitch_pid(double p, double i, double d) {
    _fc_gains.rate_pitch_p = p; _fc_gains.rate_pitch_i = i; _fc_gains.rate_pitch_d = d;
    _fc->set_gains(_fc_gains);
}
void DroneBody::set_rate_yaw_pid(double p, double i, double d) {
    _fc_gains.rate_yaw_p = p; _fc_gains.rate_yaw_i = i; _fc_gains.rate_yaw_d = d;
    _fc->set_gains(_fc_gains);
}

// ============================================================================
// GDClass binding
// ============================================================================
void DroneBody::_bind_methods() {
    // Control
    ClassDB::bind_method(D_METHOD("arm"),    &DroneBody::arm);
    ClassDB::bind_method(D_METHOD("disarm"), &DroneBody::disarm);
    ClassDB::bind_method(D_METHOD("set_attitude_setpoint","roll","pitch","yaw_rate","throttle"),
                         &DroneBody::set_attitude_setpoint);
    ClassDB::bind_method(D_METHOD("set_rotor_throttles","throttles"),
                         &DroneBody::set_rotor_throttles);
    ClassDB::bind_method(D_METHOD("get_telemetry"), &DroneBody::get_telemetry);

    // PID
    ClassDB::bind_method(D_METHOD("set_rate_roll_pid","p","i","d"),  &DroneBody::set_rate_roll_pid);
    ClassDB::bind_method(D_METHOD("set_rate_pitch_pid","p","i","d"), &DroneBody::set_rate_pitch_pid);
    ClassDB::bind_method(D_METHOD("set_rate_yaw_pid","p","i","d"),   &DroneBody::set_rate_yaw_pid);

    // Properties
    ClassDB::bind_method(D_METHOD("set_rotor_radius","r"), &DroneBody::set_rotor_radius);
    ClassDB::bind_method(D_METHOD("get_rotor_radius"),     &DroneBody::get_rotor_radius);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"rotor_radius",PROPERTY_HINT_RANGE,"0.05,0.5,0.001"),
                 "set_rotor_radius","get_rotor_radius");

    ClassDB::bind_method(D_METHOD("set_n_rotors","n"), &DroneBody::set_n_rotors);
    ClassDB::bind_method(D_METHOD("get_n_rotors"),     &DroneBody::get_n_rotors);
    ADD_PROPERTY(PropertyInfo(Variant::INT,"n_rotors",PROPERTY_HINT_ENUM,"4,6"),
                 "set_n_rotors","get_n_rotors");

    ClassDB::bind_method(D_METHOD("set_motor_kv","kv"), &DroneBody::set_motor_kv);
    ClassDB::bind_method(D_METHOD("get_motor_kv"),      &DroneBody::get_motor_kv);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"motor_kv",PROPERTY_HINT_RANGE,"100,3000,1"),
                 "set_motor_kv","get_motor_kv");

    ClassDB::bind_method(D_METHOD("set_max_voltage","v"), &DroneBody::set_max_voltage);
    ClassDB::bind_method(D_METHOD("get_max_voltage"),     &DroneBody::get_max_voltage);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"max_voltage",PROPERTY_HINT_RANGE,"7.4,44.4,0.1"),
                 "set_max_voltage","get_max_voltage");

    ClassDB::bind_method(D_METHOD("set_turbulence_intensity","i"), &DroneBody::set_turbulence_intensity);
    ClassDB::bind_method(D_METHOD("get_turbulence_intensity"),     &DroneBody::get_turbulence_intensity);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"turbulence_intensity",PROPERTY_HINT_RANGE,"0,5,0.01"),
                 "set_turbulence_intensity","get_turbulence_intensity");

    ClassDB::bind_method(D_METHOD("set_wind","wind"), &DroneBody::set_wind);
    ClassDB::bind_method(D_METHOD("get_wind"),        &DroneBody::get_wind);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3,"wind"),
                 "set_wind","get_wind");

    ClassDB::bind_method(D_METHOD("set_ground_height","h"), &DroneBody::set_ground_height);
    ClassDB::bind_method(D_METHOD("get_ground_height"),     &DroneBody::get_ground_height);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"ground_height",PROPERTY_HINT_RANGE,"-500,5000,0.1"),
                 "set_ground_height","get_ground_height");
}
