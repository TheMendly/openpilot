"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from cereal import custom
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state

State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
VTargetSource = custom.IntelligentCruiseButtonManagement.VTargetSource

BADGE_WIDTH = 214
BADGE_HEIGHT = 62
BADGE_MARGIN = 18

GREEN = rl.Color(70, 220, 120, 235)
AMBER = rl.Color(255, 180, 60, 235)
RED = rl.Color(255, 80, 70, 240)


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
    x = rect.x + rect.width - BADGE_WIDTH - BADGE_MARGIN
    y = rect.y + BADGE_MARGIN
    badge = rl.Rectangle(x, y, BADGE_WIDTH, BADGE_HEIGHT)

    rl.draw_rectangle_rounded(badge, 0.35, 8, rl.Color(0, 0, 0, 140))
    rl.draw_rectangle_rounded_lines_ex(badge, 0.35, 8, 2.0, accent)
    rl.draw_circle(int(x + 20), int(y + 22), 6.0, accent)

    unit = "km/h" if ui_state.is_metric else "mph"
    detail = f"{self._v_target} {unit}"
    if self._lead_distance > 0.0:
      detail += f"  ·  {round(self._lead_distance)} m"

    rl.draw_text_ex(self._font_semi_bold, self._label, rl.Vector2(x + 36, y + 10), 22, 0, rl.WHITE)
    rl.draw_text_ex(self._font_semi_bold, detail, rl.Vector2(x + 20, y + 36), 20, 0,
                    rl.Color(255, 255, 255, 200))

  def _has_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.detected
