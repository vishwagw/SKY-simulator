#pragma once
#include "sitl/socket_transport.hpp"
#include "sitl/sitl_state.hpp"
#include <memory>
#include <string>
#include <chrono>

namespace dronesim::sitl {

// ---------------------------------------------------------------------------
// FirmwareBridge — abstract interface every firmware adapter implements
// ---------------------------------------------------------------------------
class FirmwareBridge {
public:
    virtual ~FirmwareBridge() = default;

    // Called once at sim startup
    virtual bool connect() noexcept = 0;

    // Called every physics tick (from Godot _integrate_forces thread):
    //   1. send SimState → firmware
    //   2. non-blocking recv firmware → parse actuator output
    //   3. populate `out` only if a new frame was received
    // Returns true if a new actuator frame was received this tick
    virtual bool tick(const SimState& state, ActuatorOutput& out) noexcept = 0;

    virtual void disconnect() noexcept = 0;

    [[nodiscard]] virtual bool is_connected() const noexcept = 0;
    [[nodiscard]] virtual const char* firmware_name() const noexcept = 0;
    [[nodiscard]] virtual uint16_t port() const noexcept = 0;
};

// ===========================================================================
// ArduPilotBridge
//
// Protocol: MAVLink v2 over UDP (default port 14550)
// Sim → AP: HIL_SENSOR + HIL_GPS + HIL_STATE_QUATERNION (400 Hz / 5 Hz / 100 Hz)
// AP → Sim: SERVO_OUTPUT_RAW (8 channels, 1000–2000 µs)
//
// ArduPilot SITL command:
//   ./ArduCopter --model quad --speedup 1 --sim-address 127.0.0.1
// ===========================================================================
class ArduPilotBridge : public FirmwareBridge {
public:
    explicit ArduPilotBridge(SocketTransport::Config cfg) noexcept
        : _transport(std::move(cfg)) {}

    bool connect() noexcept override { return _transport.open(); }
    void disconnect() noexcept override { _transport.close(); }
    [[nodiscard]] bool is_connected() const noexcept override { return _transport.is_connected(); }
    [[nodiscard]] const char* firmware_name() const noexcept override { return "ArduPilot"; }
    [[nodiscard]] uint16_t port() const noexcept override { return _transport.config().local_port; }

    bool tick(const SimState& s, ActuatorOutput& out) noexcept override;

private:
    SocketTransport _transport;
    mavlink::Parser _parser;
    uint8_t         _seq{};
    uint64_t        _tick{};

    // Rate dividers
    static constexpr int SENSOR_RATE = 1;    // every tick
    static constexpr int GPS_DIVIDER  = 20;  // every 20 ticks (~5 Hz at 100 Hz physics)
    static constexpr int HB_DIVIDER   = 100; // heartbeat 1 Hz at 100 Hz

    void _send_hil_sensor(const SimState& s) noexcept;
    void _send_hil_gps(const SimState& s)    noexcept;
    void _send_hil_state(const SimState& s)  noexcept;
    void _send_heartbeat()                   noexcept;

    [[nodiscard]] bool _parse_servo_output(
        const mavlink::Frame& f, ActuatorOutput& out) const noexcept;

    uint8_t _recv_buf[4096]{};
};

// ===========================================================================
// PX4Bridge
//
// Protocol: MAVLink v2 over UDP (default port 14560)
// Sim → PX4: HIL_SENSOR + HIL_GPS (both at 250 Hz) + heartbeat
// PX4 → Sim: HIL_ACTUATOR_CONTROLS (normalised [-1,+1] / [0,1])
//
// PX4 SITL command:
//   make px4_sitl_default none_iris
//   export PX4_SIM_HOST_ADDR=127.0.0.1 && ./build/px4_sitl_default/bin/px4
// ===========================================================================
class PX4Bridge : public FirmwareBridge {
public:
    explicit PX4Bridge(SocketTransport::Config cfg) noexcept
        : _transport(std::move(cfg)) {}

    bool connect() noexcept override { return _transport.open(); }
    void disconnect() noexcept override { _transport.close(); }
    [[nodiscard]] bool is_connected() const noexcept override { return _transport.is_connected(); }
    [[nodiscard]] const char* firmware_name() const noexcept override { return "PX4"; }
    [[nodiscard]] uint16_t port() const noexcept override { return _transport.config().local_port; }

    bool tick(const SimState& s, ActuatorOutput& out) noexcept override;

private:
    SocketTransport _transport;
    mavlink::Parser _parser;
    uint8_t         _seq{};
    uint64_t        _tick{};

    static constexpr int GPS_DIVIDER = 16;
    static constexpr int HB_DIVIDER  = 100;

    void _send_hil_sensor(const SimState& s) noexcept;
    void _send_hil_gps(const SimState& s)    noexcept;
    void _send_heartbeat()                   noexcept;

    [[nodiscard]] bool _parse_hil_actuator_controls(
        const mavlink::Frame& f, ActuatorOutput& out) const noexcept;

    uint8_t _recv_buf[4096]{};
};

// ===========================================================================
// BetaflightBridge
//
// Protocol: MSP v2 over TCP (Betaflight SITL listens on port 5760)
// Betaflight polls the sim with MSP requests; sim responds.
// Betaflight → Sim: MSP_SET_MOTOR (8 × uint16 PWM µs)
// Sim → BF:         MSP_RAW_IMU, MSP_ATTITUDE, MSP_ALTITUDE responses
//
// Betaflight SITL command:
//   ./obj/main/betaflight_SITL.elf
// ===========================================================================
class BetaflightBridge : public FirmwareBridge {
public:
    explicit BetaflightBridge(SocketTransport::Config cfg) noexcept
        : _transport(std::move(cfg)) {}

    bool connect() noexcept override {
        bool ok = _transport.open();
        if (!ok && _transport.config().mode == TransportMode::TCP)
            return true; // non-blocking connect pending
        return ok;
    }
    void disconnect() noexcept override { _transport.close(); }
    [[nodiscard]] bool is_connected() const noexcept override { return _transport.is_connected(); }
    [[nodiscard]] const char* firmware_name() const noexcept override { return "Betaflight"; }
    [[nodiscard]] uint16_t port() const noexcept override { return _transport.config().remote_port; }

    bool tick(const SimState& s, ActuatorOutput& out) noexcept override;

private:
    SocketTransport _transport;
    msp::Parser     _parser;
    SimState        _last_state{};

    bool _handle_request(const msp::Frame& req, const SimState& s) noexcept;

    uint8_t _recv_buf[4096]{};
};

} // namespace dronesim::sitl
