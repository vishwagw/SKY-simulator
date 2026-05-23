## battery_hud.gd
## Attach to a Control node. Displays a comprehensive battery dashboard:
##   - Pack voltage (terminal vs OCV), current, power
##   - SoC bar with colour gradient
##   - Per-cell voltage bars (up to 12S)
##   - Peukert efficiency, temperature, cell imbalance
##   - Estimated flight time remaining
##   - Low-voltage and cutoff warnings

extends Control

@export var drone: DroneBody

# ---- Layout config ---------------------------------------------------------
@export var cell_bar_width:  int = 18
@export var cell_bar_height: int = 60
@export var cell_bar_gap:    int = 4

var _font: Font
const CELL_MIN_V = 3.27   # 0% OCV
const CELL_MAX_V = 4.20   # 100% OCV
const CELL_WARN_V = 3.50
const CELL_CRIT_V = 3.30

func _ready() -> void:
    _font = ThemeDB.fallback_font

func _process(_delta: float) -> void:
    queue_redraw()

func _draw() -> void:
    if not drone: return
    var t: Dictionary = drone.get_telemetry()
    if t.is_empty(): return

    var x: int = 10
    var y: int = 10
    var line_h: int = 18

    # ---- Pack summary row --------------------------------------------------
    var v_term: float  = t.get("battery_voltage", 0.0)
    var v_ocv:  float  = t.get("battery_voltage_ocv", 0.0)
    var amps:   float  = t.get("battery_current", 0.0)
    var watts:  float  = t.get("power_draw", 0.0)
    var soc:    float  = t.get("battery_soc", 1.0)
    var soc_pct: float = soc * 100.0
    var temp:   float  = t.get("battery_temp_c", 25.0)
    var mah:    float  = t.get("battery_mah_used", 0.0)
    var ft_min: float  = t.get("battery_flight_time_min", 0.0)
    var peff:   float  = t.get("battery_peukert_eff", 1.0)
    var imb_mv: float  = t.get("battery_cell_imbalance_mv", 0.0)
    var low_warn: bool = t.get("battery_low_warn", false)
    var cutoff:   bool = t.get("battery_cutoff", false)

    # Background panel
    var cells_arr = t.get("cell_voltages", PackedFloat64Array())
    var n_cells: int = cells_arr.size()
    var panel_w: int = max(260, n_cells * (cell_bar_width + cell_bar_gap) + 20)
    var panel_h: int = 200
    draw_rect(Rect2(x - 6, y - 6, panel_w, panel_h),
              Color(0, 0, 0, 0.72), true, 0)
    draw_rect(Rect2(x - 6, y - 6, panel_w, panel_h),
              Color(0.2, 0.8, 0.4, 0.3), false, 1)

    # Title
    _draw_text("BATTERY", x, y, Color(0.4, 1.0, 0.6), 12)
    y += line_h

    # Voltage row
    var v_color = Color.GREEN
    if low_warn: v_color = Color.YELLOW
    if cutoff:   v_color = Color.RED
    _draw_text("%.2fV  OCV %.2fV" % [v_term, v_ocv], x, y, v_color)
    y += line_h

    # Current / Power
    _draw_text("%.1fA  %.0fW  %.0fmAh" % [amps, watts, mah], x, y, Color.WHITE)
    y += line_h

    # SoC bar
    var bar_w: int = panel_w - 20
    var bar_h: int = 12
    draw_rect(Rect2(x, y, bar_w, bar_h), Color(0.15, 0.15, 0.15), true)
    var fill_w: int = int(bar_w * soc)
    var soc_col = _soc_color(soc)
    draw_rect(Rect2(x, y, fill_w, bar_h), soc_col, true)
    draw_rect(Rect2(x, y, bar_w, bar_h), Color(0.4, 0.4, 0.4), false, 1)
    _draw_text("%.0f%%" % soc_pct, x + bar_w / 2 - 12, y - 1, Color.WHITE, 11)
    y += bar_h + 4

    # Flight time + temp
    var ft_str = "∞" if ft_min > 999 else ("%.1fmin" % ft_min)
    _draw_text("EFT %s  T %.0f°C  Pekrt %.2f" % [ft_str, temp, peff], x, y, Color(0.8, 0.9, 1.0), 11)
    y += line_h

    # Cell imbalance
    var imb_col = Color.GREEN if imb_mv < 10 else (Color.YELLOW if imb_mv < 30 else Color.RED)
    _draw_text("Imbal %.0fmV" % imb_mv, x, y, imb_col, 11)
    y += line_h + 2

    # ---- Per-cell voltage bars --------------------------------------------
    if n_cells > 0:
        var bar_top: int = y
        for i in range(n_cells):
            var cv: float = cells_arr[i]
            var frac: float = clamp((cv - CELL_MIN_V) / (CELL_MAX_V - CELL_MIN_V), 0.0, 1.0)
            var bx: int = x + i * (cell_bar_width + cell_bar_gap)

            # Background
            draw_rect(Rect2(bx, bar_top, cell_bar_width, cell_bar_height),
                      Color(0.1, 0.1, 0.1), true)
            # Fill from bottom
            var fill_h: int = int(cell_bar_height * frac)
            var cell_col: Color = _cell_color(cv)
            draw_rect(Rect2(bx, bar_top + cell_bar_height - fill_h,
                            cell_bar_width, fill_h), cell_col, true)
            # Border — red if critical
            var border_col = Color.RED if cv < CELL_CRIT_V else Color(0.3, 0.6, 0.3)
            draw_rect(Rect2(bx, bar_top, cell_bar_width, cell_bar_height),
                      border_col, false, 1)
            # Voltage label below bar
            _draw_text("%.2f" % cv,
                       bx - 1, bar_top + cell_bar_height + 2,
                       cell_col, 9)

    # ---- Warning overlays -------------------------------------------------
    if cutoff:
        _draw_text("⚠ BATT CUTOFF", x, 8, Color.RED, 14)
    elif low_warn:
        _draw_text("LOW BATT", x + panel_w - 80, 8, Color.YELLOW, 12)

# ============================================================================
func _draw_text(text: String, px: int, py: int,
                color: Color = Color.WHITE, size: int = 12) -> void:
    draw_string(_font, Vector2(px, py + size), text,
                HORIZONTAL_ALIGNMENT_LEFT, -1, size, color)

func _soc_color(soc: float) -> Color:
    if soc > 0.5:
        return Color(lerp(1.0, 0.0, (soc - 0.5) * 2.0), 1.0, 0.0)
    else:
        return Color(1.0, lerp(0.0, 1.0, soc * 2.0), 0.0)

func _cell_color(v: float) -> Color:
    if v >= 3.7:  return Color(0.2, 0.9, 0.3)
    if v >= CELL_WARN_V: return Color(1.0, 0.85, 0.0)
    if v >= CELL_CRIT_V: return Color(1.0, 0.4, 0.0)
    return Color(1.0, 0.1, 0.1)
