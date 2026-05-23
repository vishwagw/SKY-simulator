## drone_controller.gd
## Attach to a Node that is a sibling of the DroneBody node.
## Reads Xbox/PS gamepad axes + keyboard fallback and feeds
## attitude setpoints to the C++ DroneBody.

extends Node

@export var drone: DroneBody
@export var hud_label: RichTextLabel

## Input mapping (remap in Project → Input Map)
const ACTION_ARM       = "drone_arm"
const ACTION_DISARM    = "drone_disarm"
const AXIS_THROTTLE    = "drone_throttle"   # left stick Y  (inverted)
const AXIS_YAW         = "drone_yaw"        # left stick X
const AXIS_PITCH       = "drone_pitch"      # right stick Y
const AXIS_ROLL        = "drone_roll"       # right stick X

## Feel parameters (tune in editor)
@export_range(0.0, 1.0, 0.01) var throttle_expo: float = 0.3
@export_range(0.0, 1.0, 0.01) var stick_expo:    float = 0.4
@export_range(0.0, 45.0, 0.5) var max_angle_deg: float = 30.0
@export_range(0.0, 360.0, 1.0)var max_yaw_rate_deg: float = 120.0

var _throttle_smooth: float = 0.0
var _armed: bool = false

func _ready() -> void:
	assert(drone != null, "Assign DroneBody node in inspector")
	_update_hud({})

func _process(delta: float) -> void:
	# --- Arm / disarm --------------------------------------------------------
	if Input.is_action_just_pressed(ACTION_ARM) and not _armed:
		drone.arm()
		_armed = true
	if Input.is_action_just_pressed(ACTION_DISARM) and _armed:
		drone.disarm()
		_armed = false

	if not _armed:
		_update_hud(drone.get_telemetry())
		return

	# --- Read sticks ----------------------------------------------------------
	var raw_thr:   float = _get_axis(AXIS_THROTTLE, KEY_W, KEY_S, false)
	var raw_yaw:   float = _get_axis(AXIS_YAW,      KEY_A, KEY_D, false)
	var raw_pitch: float = _get_axis(AXIS_PITCH,     KEY_UP, KEY_DOWN, true)
	var raw_roll:  float = _get_axis(AXIS_ROLL,      KEY_LEFT, KEY_RIGHT, false)

	# --- Expo curves (reduces sensitivity near centre) -----------------------
	var thr   = _expo(raw_thr,   throttle_expo)
	var yaw   = _expo(raw_yaw,   stick_expo)
	var pitch = _expo(raw_pitch, stick_expo)
	var roll  = _expo(raw_roll,  stick_expo)

	# Map to physical units
	var throttle_norm = clamp((thr + 1.0) * 0.5, 0.0, 1.0)  # stick [-1,1] → [0,1]
	var yaw_rate = deg_to_rad(yaw   * max_yaw_rate_deg)
	var pitch_sp = deg_to_rad(pitch * max_angle_deg)
	var roll_sp  = deg_to_rad(roll  * max_angle_deg)

	# Smooth throttle to avoid step inputs
	_throttle_smooth = lerp(_throttle_smooth, throttle_norm, 1.0 - exp(-10.0 * delta))

	drone.set_attitude_setpoint(roll_sp, pitch_sp, yaw_rate, _throttle_smooth)

	_update_hud(drone.get_telemetry())

# ============================================================================
# HUD
# ============================================================================
func _update_hud(telem: Dictionary) -> void:
	if hud_label == null: return
	if telem.is_empty():
		hud_label.text = "[b]DISARMED[/b]"
		return

	var vrs_str = "[color=red]VRS![/color]" if telem.get("vrs_active", false) else "clear"
	var ge = telem.get("ground_effect_factor", 1.0)
	var ge_str = ("[color=yellow]%.2f×[/color]" % ge) if ge > 1.02 else ("%.2f×" % ge)

	hud_label.text = (
		"[b]ALT[/b]  %.1f m    [b]VS[/b] %+.1f m/s    [b]GS[/b] %.1f m/s\n" % [
			telem.get("altitude",0), telem.get("vertical_speed",0), telem.get("ground_speed",0)
		] +
		"[b]R[/b] %+5.1f°  [b]P[/b] %+5.1f°  [b]Y[/b] %+6.1f°\n" % [
			telem.get("roll_deg",0), telem.get("pitch_deg",0), telem.get("yaw_deg",0)
		] +
		"[b]Thrust[/b] %.1f N    [b]Power[/b] %.0f W    [b]ρ[/b] %.3f kg/m³\n" % [
			telem.get("total_thrust",0), telem.get("power_draw",0), telem.get("air_density",1.225)
		] +
		"[b]GE[/b] %s    [b]VRS[/b] %s    [b]Wind[/b] %s\n" % [
			ge_str, vrs_str, str(telem.get("wind", Vector3.ZERO))
		]
	)

# ============================================================================
# Helpers
# ============================================================================
func _get_axis(action: String, key_pos: Key, key_neg: Key, invert: bool) -> float:
	# Try gamepad axis first
	var v = Input.get_axis(action + "_neg", action + "_pos")
	# Keyboard fallback
	if abs(v) < 0.05:
		if Input.is_key_pressed(key_pos): v =  1.0
		elif Input.is_key_pressed(key_neg): v = -1.0
	return -v if invert else v

func _expo(x: float, e: float) -> float:
	# Expo curve: blends linear with cubic
	return lerp(x, x * x * x, e)
