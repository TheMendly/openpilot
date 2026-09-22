"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from dataclasses import dataclass

import pyray as rl

from cereal import custom
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached

State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
VTargetSource = custom.IntelligentCruiseButtonManagement.VTargetSource

# Wide enough that neither line has to shrink at the sizes below; clamped against the
# viewport so a small screen gets a proportionate badge instead of a wall.
BADGE_WIDTH = 380
BADGE_MAX_WIDTH_FRAC = 0.4
BADGE_HEIGHT = 95
BADGE_MARGIN = 18
BADGE_ROUNDNESS = 0.30
BADGE_SEGMENTS = 8
BADGE_PAD_X = 22
LINE_GAP = 4
DOT_RADIUS = 9
DOT_GAP = 14


@dataclass(frozen=True)
class BadgeFontSizes:
  title: int = 34
  detail: int = 28
  # Shrink floor: the detail line carries a variable lead distance, so it has to be
  # allowed to give ground rather than spill out of the rounded box.
  detail_min: int = 20


BADGE_FONT_SIZES = BadgeFontSizes()

GREEN = rl.Color(70, 220, 120, 235)
AMBER = rl.Color(255, 180, 60, 235)
RED = rl.Color(255, 80, 70, 240)


def _fit_font_size(font: rl.Font, text: str, size: int, min_size: int, max_width: float) -> int:
  """Shrink until the text fits. measure_text_cached memoises per (font, text, size)."""
  while size > min_size and measure_text_cached(font, text, size).x > max_width:
    size -= 2
  return size


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()

    # Pseudo-ACC status badge. Everything shown here comes straight from the ICBM
    # state published by selfdrived, so the HUD cannot disagree with the car.
    self._visible = False
    self._label = ""
    self._accent = GREEN
    self._urgent = False
    self._v_target = 0
    self._lead_distance = 0.0

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    self._update_pseudo_acc()

    # The badge and the right blind spot icon share the upper right corner: the icon
    # sits 100px down by default, which the badge now reaches past.
    self.blind_spot_indicators.set_right_y_offset(BADGE_MARGIN + BADGE_HEIGHT + 10 if self._visible else 0)

  def _update_pseudo_acc(self) -> None:
    sm = ui_state.sm
    self._visible = False

    if not (sm.alive['selfdriveStateSP'] and sm.valid['selfdriveStateSP']):
      return

    icbm = sm['selfdriveStateSP'].intelligentCruiseButtonManagement

    # Only surface the badge while pseudo-ACC is actually governing the set speed
    # or warning about it. Plain ICBM and cars without it stay untouched.
    if not (icbm.cancel or icbm.brakeRequired or icbm.atSpeedFloor or
            icbm.vTargetSource != VTargetSource.cruise):
      return

    self._visible = True
    self._v_target = max(0, icbm.vTarget)
    self._urgent = bool(icbm.cancel or icbm.brakeRequired)

    if icbm.cancel:
      self._label, self._accent = "TAKE OVER", RED
    elif icbm.brakeRequired:
      self._label, self._accent = "BRAKE", RED
    elif icbm.atSpeedFloor:
      self._label, self._accent = "MIN SPEED", AMBER
    elif icbm.state == State.decreasing:
      self._label, self._accent = "SLOWING", AMBER
    elif icbm.state == State.increasing:
      self._label, self._accent = "RESUMING", GREEN
    else:
      self._label, self._accent = "FOLLOWING", GREEN

    self._lead_distance = 0.0
    if sm.alive['radarState'] and sm.valid['radarState']:
      lead = sm['radarState'].leadOne
      if lead.status:
        self._lead_distance = max(0.0, lead.dRel)

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self._draw_pseudo_acc_badge(rect)

  def _draw_pseudo_acc_badge(self, rect: rl.Rectangle) -> None:
    if not self._visible:
      return

    # Urgent states pulse so they read at a glance without stealing the alert bar.
    accent = self._accent
    if self._urgent and int(rl.get_time() * 3.0) % 2 == 0:
      accent = rl.Color(accent.r, accent.g, accent.b, 110)

    # Compact translucent badge in the upper right, clear of the road path, the
    # speedometer, the steering wheel and the bottom alerts.
    width = min(BADGE_WIDTH, rect.width * BADGE_MAX_WIDTH_FRAC)
    x = rect.x + rect.width - width - BADGE_MARGIN
    y = rect.y + BADGE_MARGIN
    badge = rl.Rectangle(x, y, width, BADGE_HEIGHT)

    rl.draw_rectangle_rounded(badge, BADGE_ROUNDNESS, BADGE_SEGMENTS, rl.Color(0, 0, 0, 140))
    rl.draw_rectangle_rounded_lines_ex(badge, BADGE_ROUNDNESS, BADGE_SEGMENTS, 2.0, accent)

    unit = "km/h" if ui_state.is_metric else "mph"
    detail = f"{self._v_target} {unit}"
    if self._lead_distance > 0.0:
      detail += f"  ·  {round(self._lead_distance)} m"

    font = self._font_semi_bold
    title_x_off = BADGE_PAD_X + DOT_RADIUS * 2 + DOT_GAP
    title_avail = width - title_x_off - BADGE_PAD_X
    detail_avail = width - BADGE_PAD_X * 2

    title_size = _fit_font_size(font, self._label, BADGE_FONT_SIZES.title,
                                BADGE_FONT_SIZES.detail_min, title_avail)
    detail_size = _fit_font_size(font, detail, BADGE_FONT_SIZES.detail,
                                 BADGE_FONT_SIZES.detail_min, detail_avail)

    title_m = measure_text_cached(font, self._label, title_size)
    detail_m = measure_text_cached(font, detail, detail_size)

    # Centre the two line block rather than pinning fixed offsets: the text scale
    # differs between device classes, so hardcoded y offsets are only ever right on one.
    block_h = title_m.y + LINE_GAP + detail_m.y
    title_y = y + (BADGE_HEIGHT - block_h) / 2
    detail_y = title_y + title_m.y + LINE_GAP

    rl.draw_circle(int(x + BADGE_PAD_X + DOT_RADIUS), int(title_y + title_m.y / 2), float(DOT_RADIUS), accent)
    rl.draw_text_ex(font, self._label, rl.Vector2(x + title_x_off, title_y), title_size, 0, rl.WHITE)
    rl.draw_text_ex(font, detail, rl.Vector2(x + BADGE_PAD_X, detail_y), detail_size, 0,
                    rl.Color(255, 255, 255, 200))

  def _has_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.detected
