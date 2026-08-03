"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()

    # Bayon pseudo-ACC lead indicator. This deliberately follows
    # longitudinalPlan.hasLead, the same signal used by ICBM.
    self._lead_active = False
    self._lead_distance = 0.0
    self._lead_relative_speed = 0.0

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

    sm = ui_state.sm
    plan_valid = sm.alive['longitudinalPlan'] and sm.valid['longitudinalPlan']
    self._lead_active = plan_valid and sm['longitudinalPlan'].hasLead
    self._lead_distance = 0.0
    self._lead_relative_speed = 0.0

    if self._lead_active:
      lead = sm['radarState'].leadOne
      if lead.status:
        self._lead_distance = max(0.0, lead.dRel)
        self._lead_relative_speed = lead.vRel

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self._draw_lead_indicator(rect)

  def _draw_lead_indicator(self, rect: rl.Rectangle) -> None:
    if not self._lead_active:
      return

    # Compact translucent badge in the upper-right corner so it stays clear of
    # the road path, speedometer, steering wheel, and bottom alerts.
    width = 150
    height = 44
    margin = 18
    x = rect.x + rect.width - width - margin
    y = rect.y + margin
    badge = rl.Rectangle(x, y, width, height)

    # vRel is negative while approaching the lead vehicle.
    closing_speed = max(0.0, -self._lead_relative_speed)
    if closing_speed >= 5.0:
      accent = rl.Color(255, 80, 70, 230)
    elif closing_speed >= 2.0:
      accent = rl.Color(255, 180, 60, 230)
    else:
      accent = rl.Color(70, 220, 120, 230)

    rl.draw_rectangle_rounded(badge, 0.45, 8, rl.Color(0, 0, 0, 125))
    rl.draw_rectangle_rounded_lines_ex(badge, 0.45, 8, 2.0, accent)
    rl.draw_circle(int(x + 17), int(y + height / 2), 6.0, accent)

    distance_text = f"{round(self._lead_distance)} m" if self._lead_distance > 0.0 else "-- m"
    rl.draw_text_ex(self._font_semi_bold, "LEAD", rl.Vector2(x + 30, y + 4), 18, 0, rl.Color(255, 255, 255, 190))
    rl.draw_text_ex(self._font_semi_bold, distance_text, rl.Vector2(x + 30, y + 20), 22, 0, rl.WHITE)

  def _has_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.detected
