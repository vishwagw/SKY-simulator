#pragma once
#include <cmath>
#include <algorithm>
#include <numbers>

namespace dronesim {

// -------------------------------------------------------------------------
// Constants
// -------------------------------------------------------------------------
inline constexpr double PI      = std::numbers::pi;
inline constexpr double TWO_PI  = 2.0 * PI;
inline constexpr double DEG2RAD = PI / 180.0;
inline constexpr double RAD2DEG = 180.0 / PI;

// -------------------------------------------------------------------------
// Scalar helpers
// -------------------------------------------------------------------------
template<typename T>
[[nodiscard]] inline T clamp(T v, T lo, T hi) noexcept {
    return std::clamp(v, lo, hi);
}

template<typename T>
[[nodiscard]] inline T lerp(T a, T b, T t) noexcept {
    return a + t * (b - a);
}

template<typename T>
[[nodiscard]] inline T sign(T v) noexcept {
    return (v > T{0}) - (v < T{0});
}

// -------------------------------------------------------------------------
// Vec3d — double-precision 3-vector (physics internal representation)
// -------------------------------------------------------------------------
struct Vec3d {
    double x{}, y{}, z{};

    [[nodiscard]] Vec3d operator+(Vec3d o) const noexcept { return {x+o.x, y+o.y, z+o.z}; }
    [[nodiscard]] Vec3d operator-(Vec3d o) const noexcept { return {x-o.x, y-o.y, z-o.z}; }
    [[nodiscard]] Vec3d operator*(double s) const noexcept { return {x*s, y*s, z*s}; }
    [[nodiscard]] Vec3d operator/(double s) const noexcept { double r=1.0/s; return {x*r,y*r,z*r}; }
    Vec3d& operator+=(Vec3d o) noexcept { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3d& operator-=(Vec3d o) noexcept { x-=o.x; y-=o.y; z-=o.z; return *this; }
    Vec3d& operator*=(double s) noexcept { x*=s; y*=s; z*=s; return *this; }

    [[nodiscard]] double dot(Vec3d o)  const noexcept { return x*o.x + y*o.y + z*o.z; }
    [[nodiscard]] double norm2()       const noexcept { return dot(*this); }
    [[nodiscard]] double norm()        const noexcept { return std::sqrt(norm2()); }
    [[nodiscard]] Vec3d  normalized()  const noexcept { double n=norm(); return n>1e-12?*this/n:Vec3d{}; }

    [[nodiscard]] Vec3d cross(Vec3d o) const noexcept {
        return { y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x };
    }

    [[nodiscard]] static Vec3d zero()    noexcept { return {}; }
    [[nodiscard]] static Vec3d up()      noexcept { return {0,1,0}; }
    [[nodiscard]] static Vec3d forward() noexcept { return {0,0,-1}; }
};

[[nodiscard]] inline Vec3d operator*(double s, Vec3d v) noexcept { return v*s; }

// -------------------------------------------------------------------------
// Wrench — force + torque pair applied to rigid body
// -------------------------------------------------------------------------
struct Wrench {
    Vec3d force{};
    Vec3d torque{};

    Wrench& operator+=(Wrench o) noexcept {
        force  += o.force;
        torque += o.torque;
        return *this;
    }
    [[nodiscard]] Wrench operator+(Wrench o) const noexcept {
        return { force+o.force, torque+o.torque };
    }
};

// -------------------------------------------------------------------------
// Quaternion (Hamilton convention, w-last storage)
// -------------------------------------------------------------------------
struct Quat {
    double x{}, y{}, z{}, w{1};

    [[nodiscard]] static Quat identity() noexcept { return {0,0,0,1}; }

    [[nodiscard]] Quat conjugate() const noexcept { return {-x,-y,-z,w}; }

    [[nodiscard]] Vec3d rotate(Vec3d v) const noexcept {
        // Rodrigues-style quaternion rotation
        Vec3d qv{x,y,z};
        Vec3d t = 2.0 * qv.cross(v);
        return v + w*t + qv.cross(t);
    }

    [[nodiscard]] Vec3d to_euler_rpy() const noexcept {
        // roll-pitch-yaw (ZYX extrinsic)
        double sinr_cosp = 2*(w*x + y*z);
        double cosr_cosp = 1 - 2*(x*x + y*y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2*(w*y - z*x);
        double pitch = std::abs(sinp)>=1 ? std::copysign(PI/2, sinp) : std::asin(sinp);

        double siny_cosp = 2*(w*z + x*y);
        double cosy_cosp = 1 - 2*(y*y + z*z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return {roll, pitch, yaw};
    }

    [[nodiscard]] static Quat from_axis_angle(Vec3d axis, double angle) noexcept {
        double ha = angle * 0.5;
        double s  = std::sin(ha);
        return {axis.x*s, axis.y*s, axis.z*s, std::cos(ha)};
    }

    [[nodiscard]] Quat operator*(Quat o) const noexcept {
        return {
            w*o.x + x*o.w + y*o.z - z*o.y,
            w*o.y - x*o.z + y*o.w + z*o.x,
            w*o.z + x*o.y - y*o.x + z*o.w,
            w*o.w - x*o.x - y*o.y - z*o.z
        };
    }

    [[nodiscard]] Quat normalized() const noexcept {
        double n = std::sqrt(x*x+y*y+z*z+w*w);
        return {x/n,y/n,z/n,w/n};
    }
};

// -------------------------------------------------------------------------
// RigidBodyState — snapshot passed between subsystems
// -------------------------------------------------------------------------
struct RigidBodyState {
    Vec3d position{};         // world frame, metres
    Quat  orientation{};      // world-to-body rotation
    Vec3d velocity{};         // world frame, m/s
    Vec3d angular_velocity{}; // body frame, rad/s
    double mass{1.5};         // kg
};

} // namespace dronesim
