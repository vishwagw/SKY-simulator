## sitl_hud.gd
## Attach to a CanvasLayer child. Displays real-time SITL connection state
## for all three firmware bridges alongside flight telemetry.

extends Control

@export var sitl_manager: SITLManager
@export var drone: DroneBody

@onready var _panel: RichTextLabel = $RichTextLabel

const TICK = 0.1  # update interval seconds
var _accum: float = 0.0

func _process(delta: float) -> void:
    _accum += delta
    if _accum < TICK: return
    _accum = 0.0
    _refresh()

func _refresh() -> void:
    if not sitl_manager or not drone: return
    var status: Dictionary = sitl_manager.get_sitl_status()
    var telem:  Dictionary = drone.get_telemetry()
    var active: String     = sitl_manager.get_active_firmware()

    var text = "[font_size=13]"

    # ---- Firmware status row -----------------------------------------------
    text += "[b]SITL[/b]  active=[b]%s[/b]\n" % active.to_upper()
    for fw in ["ardupilot", "px4", "betaflight"]:
        var s: Dictionary = status.get(fw, {})
        var conn:    bool  = s.get("connected", false)
        var enabled: bool  = s.get("enabled",   false)
        var port:    int   = s.get("port",       0)
        var rx_ms:   int   = s.get("last_rx_ms", -1)

        var icon  = "[color=green]●[/color]" if conn  else "[color=gray]○[/color]"
        var en_tag = "" if enabled else "[color=gray](off)[/color]"
        var rx_tag = ""
        if conn and rx_ms >= 0:
            if   rx_ms < 100:  rx_tag = " [color=green]%.0fms[/color]" % rx_ms
            elif rx_ms < 500:  rx_tag = " [color=yellow]%.0fms[/color]" % rx_ms
            else:               rx_tag = " [color=red]%.0fms![/color]" % rx_ms

        text += "  %s [b]%s[/b] :%d%s%s\n" % [icon, fw.to_upper(), port, rx_tag, en_tag]

    text += "\n"

    # ---- Flight telemetry --------------------------------------------------
    if not telem.is_empty():
        text += "[b]ALT[/b] %.1fm  [b]VS[/b] %+.1fm/s  [b]GS[/b] %.1fm/s\n" % [
            telem.get("altitude", 0),
            telem.get("vertical_speed", 0),
            telem.get("ground_speed",   0)
        ]
        text += "[b]R[/b]%+.1f°  [b]P[/b]%+.1f°  [b]Y[/b]%+.1f°\n" % [
            telem.get("roll_deg",  0),
            telem.get("pitch_deg", 0),
            telem.get("yaw_deg",   0)
        ]
        var vrs_str = "[color=red] VRS[/color]" if telem.get("vrs_active", false) else ""
        var ge = telem.get("ground_effect_factor", 1.0)
        var ge_str = (" [color=yellow]GE×%.2f[/color]" % ge) if ge > 1.02 else ""
        text += "[b]Thrust[/b] %.1fN  [b]P[/b] %.0fW  ρ=%.3fkg/m³%s%s\n" % [
            telem.get("total_thrust", 0),
            telem.get("power_draw",   0),
            telem.get("air_density",  1.225),
            vrs_str, ge_str
        ]

    text += "[/font_size]"
    _panel.text = text
