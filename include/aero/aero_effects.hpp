#pragma once
#include "core/math_types.hpp"
#include "aero/atmosphere.hpp"

namespace dronesim {

// -------------------------------------------------------------------------
// Ground Effect (IGE — In-Ground-Effect)
//
// Based on Cheeseman & Bennett (1955) empirical model:
//   T_IGE / T_OGE = 1 / (1 - (R/4h)²)
//
// Valid for h/R in [0.25, 4.0].  Returns a thrust multiplier.
// -------------------------------------------------------------------------
class GroundEffectModel {
public:
    explicit GroundEffectModel(double rotor_radius) noexcept
        : _R(rotor_radius) {}

    // height_above_ground: distance from rotor disk to ground plane (m)
    [[nodiscard]] double thrust_multiplier(double height_above_ground) const noexcept {
        const double h = std::max(height_above_ground, _R * 0.25);
        const double ratio = _R / (4.0 * h);
        const double denom = 1.0 - ratio * ratio;
        if (denom < 1e-6) return 1.0; // degenerate, clamp
        return clamp(1.0 / denom, 1.0, 2.5); // physically bounded
    }

    // Smoothly blend out as h/R > 3 (effect negligible beyond)
    [[nodiscard]] double effective_multiplier(double height_above_ground) const noexcept {
        const double h_R = height_above_ground / _R;
        if (h_R > 3.0) return 1.0;
        const double raw   = thrust_multiplier(height_above_ground);
        const double blend = clamp(1.0 - (h_R - 0.25) / 2.75, 0.0, 1.0);
        return 1.0 + (raw - 1.0) * blend;
    }

private:
    double _R;
};

// -------------------------------------------------------------------------
// Vortex Ring State (VRS) detector & thrust degradation model
//
// VRS onset conditions (Leishman 2000):
//   - Descent rate   ≥ 0.3 · Vc  (Vc = hover induced velocity)
//   - Ascent rate    ≤ 1.5 · Vc
//   - Lateral speed  ≤ 0.5 · Vc  (nearly vertical descent required)
//
// Inside the VRS envelope, thrust is reduced by up to 30% with random
// fluctuations (simulating the unsteady nature of the phenomenon).
// -------------------------------------------------------------------------
class VortexRingStateModel {
public:
    struct VRSState {
        double severity{0.0};    // 0 = none, 1 = full VRS
        double thrust_factor{1.0};
        bool   active{false};
    };

    // hover_induced_vel: √(T/(2ρA)) from BET output (m/s)
    [[nodiscard]] VRSState evaluate(
        double hover_induced_vel,  // Vc
        double descent_rate,       // positive = descending (m/s)
        double lateral_speed,      // m/s
        double dt
    ) noexcept {
        const double Vc = std::max(hover_induced_vel, 0.1);

        // Normalised descent rate into rotor disk
        const double mu_d = descent_rate / Vc;
        const double mu_l = lateral_speed / Vc;

        // Check VRS envelope: 0.3 ≤ μd ≤ 1.5, μl ≤ 0.5
        double onset = 0.0;
        if (mu_d > 0.3 && mu_d < 1.5 && mu_l < 0.5) {
            // Severity peaks at μd ≈ 0.9 (approximately hover sink rate)
            double d_factor = 1.0 - std::abs(mu_d - 0.9) / 0.6;
            double l_factor = 1.0 - mu_l / 0.5;
            onset = clamp(d_factor * l_factor, 0.0, 1.0);
        }

        // Low-pass filter for hysteresis (VRS doesn't snap in/out)
        const double tau_in  = 0.5;  // s — build-up time
        const double tau_out = 1.2;  // s — recovery time (longer)
        const double tau     = (onset > _severity) ? tau_in : tau_out;
        _severity += (onset - _severity) * (dt / (tau + dt));

        // Thrust degradation: up to 30% reduction at full VRS
        // Add low-frequency noise to simulate buffeting
        _noise_phase += dt * 2.3; // ~2.3 Hz characteristic VRS frequency
        double noise  = 0.5 * std::sin(_noise_phase) + 0.5 * std::sin(_noise_phase * 1.7);
        double factor = 1.0 - _severity * (0.25 + 0.05 * noise);

        return {
            _severity,
            clamp(factor, 0.5, 1.0),
            _severity > 0.1
        };
    }

private:
    double _severity{0.0};
    double _noise_phase{0.0};
};

// -------------------------------------------------------------------------
// AeroEffectsBundle — combined interface passed to DroneBody
// -------------------------------------------------------------------------
struct AeroEffectsBundle {
    GroundEffectModel   ground_effect;
    VortexRingStateModel vrs;

    explicit AeroEffectsBundle(double rotor_radius)
        : ground_effect(rotor_radius) {}
};

} // namespace dronesim
