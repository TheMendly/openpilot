"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import D_MIN, \
  coast_decel_authority, follow_target_speed, lookahead_time, plan_speed_at, required_decel, \
  time_gap_for_personality

# Lead acceptance. Vision-only leads flicker a lot in town (parked cars, oncoming
# traffic), and every false positive here becomes a real set-speed drop, so a lead
# has to be believable and stable before it moves the target.
LEAD_PROB_MIN = 0.5
LEAD_MAX_DIST = 150.0  # m
LEAD_MAX_LATERAL = 2.5  # m
LEAD_ACQUIRE_T = 0.35  # s of continuously valid lead before we act on it
LEAD_RELEASE_T = 0.80  # s of continuously invalid lead before we let go

# Target ramp. Losing a lead must not fire a burst of RES+.
A_UP_MAX = 0.4  # m/s^2

# Escalation thresholds.
BRAKE_REQUIRED_T = 0.50  # s the coast authority must be exceeded before warning
HARD_DECEL_A = -1.5  # m/s^2, far beyond anything coasting can deliver
HARD_DECEL_T = 0.30  # s
STOP_CANCEL_DIST = 60.0  # m, only trust shouldStop for a lead this close
FLOOR_CANCEL_MARGIN = 0.83  # m/s (~3 km/h) below the stock cruise floor
FLOOR_CANCEL_T = 0.50  # s


class Source:
  """Why the current target was chosen. Diagnostic only, surfaced on the HUD."""
  cruise = 0  # longitudinalPlanSP: driver set speed, speed limit, curve
  plan = 1  # longitudinal plan trajectory
  lead = 2  # explicit follow law


class PseudoAcc:
  """Turns the longitudinal plan and the lead into a stock cruise set speed.

  Pure logic: it receives messages selfdrived already subscribes to, so it owns no
  sockets. The output is a *target set speed*, never an acceleration - the stock
  cruise stays in charge of the throttle at all times, and the only way this class
  can slow the car is by asking for a lower set speed or by giving up entirely.
  """

  def __init__(self):
    self.v_target_sp = 0.0
    self.reset()

  def reset(self) -> None:
    self.v_target_ms = 0.0
    self.source = Source.cruise

    self.lead_active = False
    self.lead_valid_t = 0.0
    self.lead_invalid_t = 0.0
    # Last *valid* lead measurement. Held through the release grace period so a
    # momentary dropout does not present a zeroed lead as a stopped car ahead.
    self.lead_d_rel = 0.0
    self.lead_v_lead = 0.0
    self.lead_v_rel = 0.0

    self.a_req = 0.0
    self.a_coast = 0.0

    self.brake_required = False
    self.brake_required_t = 0.0
    self.hard_decel_t = 0.0
    self.floor_t = 0.0
    self.at_speed_floor = False

    self.cancel = False
    self.cancel_latched = False

    self.initialized = False

  def update_lead(self, lead, plan_valid: bool, long_plan) -> None:
    valid = bool(lead is not None and lead.status and lead.modelProb >= LEAD_PROB_MIN and
                 0.0 < lead.dRel < LEAD_MAX_DIST and abs(lead.yRel) < LEAD_MAX_LATERAL)

    # Agree with the planner before acting, so we never follow something the
    # longitudinal plan has not itself accepted as a lead.
    valid = valid and (not plan_valid or long_plan.hasLead)

    if valid:
      self.lead_valid_t += DT_CTRL
      self.lead_invalid_t = 0.0
      self.lead_d_rel = float(lead.dRel)
      self.lead_v_lead = float(lead.vLead)
      self.lead_v_rel = float(lead.vRel)
    else:
      self.lead_invalid_t += DT_CTRL
      self.lead_valid_t = 0.0

    if not self.lead_active and self.lead_valid_t >= LEAD_ACQUIRE_T:
      self.lead_active = True
    elif self.lead_active and self.lead_invalid_t >= LEAD_RELEASE_T:
      self.lead_active = False

    if not self.lead_active:
      self.lead_d_rel = 0.0
      self.lead_v_lead = 0.0
      self.lead_v_rel = 0.0

  def update_target(self, CS, long_plan, plan_valid: bool, personality: int) -> float:
    v_ego = max(CS.vEgo, 0.0)

    v_target = float(self.v_target_sp)
    self.source = Source.cruise

    if plan_valid:
      v_plan = plan_speed_at(long_plan.speeds, lookahead_time(v_ego))
      if v_plan is not None and v_plan < v_target:
        v_target, self.source = v_plan, Source.plan
    elif CS.cruiseState.speedCluster > 0:
      # A missing or stale plan must never cause an automatic RES+.
      v_target = min(v_target, float(CS.cruiseState.speedCluster))

    if self.lead_active:
      t_gap = time_gap_for_personality(personality)
      v_follow = follow_target_speed(v_ego, self.lead_d_rel, self.lead_v_lead, t_gap)
      if v_follow < v_target:
        v_target, self.source = v_follow, Source.lead

    if not self.initialized:
      self.v_target_ms = v_target
      self.initialized = True

    # Rate limit upwards only: shed speed as fast as the situation demands, but
    # come back gently so a lead dropout does not snap the car back to cruise.
    return min(v_target, self.v_target_ms + A_UP_MAX * DT_CTRL)

  def update_escalation(self, CS, long_plan, plan_valid: bool, v_cruise_min_ms: float) -> None:
    v_ego = max(CS.vEgo, 0.0)

    self.a_coast = coast_decel_authority(v_ego)
    if self.lead_active:
      self.a_req = required_decel(v_ego, self.v_target_ms, max(self.lead_d_rel - D_MIN, 1.0))
    else:
      self.a_req = 0.0

    # Level 1: the target is below what the stock cruise is able to hold.
    self.at_speed_floor = self.v_target_ms < v_cruise_min_ms

    # Level 2: coasting cannot deliver the deceleration the situation needs.
    infeasible = self.lead_active and self.a_req < self.a_coast
    self.brake_required_t = self.brake_required_t + DT_CTRL if infeasible else 0.0
    self.brake_required = self.brake_required_t >= BRAKE_REQUIRED_T

    # Level 3: hand the car back. Either the plan wants a full stop, or holding
    # the cruise floor would drive us into the lead, or no amount of coasting can
    # bridge the gap.
    hard = self.lead_active and self.a_req < HARD_DECEL_A
    self.hard_decel_t = self.hard_decel_t + DT_CTRL if hard else 0.0

    under_floor = self.lead_active and self.v_target_ms < (v_cruise_min_ms - FLOOR_CANCEL_MARGIN)
    self.floor_t = self.floor_t + DT_CTRL if under_floor else 0.0

    should_stop = plan_valid and long_plan.shouldStop and self.lead_active and \
                  self.lead_d_rel < STOP_CANCEL_DIST

    if not CS.cruiseState.enabled:
      # The stock cruise is already out; rearm for the next engagement.
      self.cancel_latched = False
    elif should_stop or self.floor_t >= FLOOR_CANCEL_T or self.hard_decel_t >= HARD_DECEL_T:
      self.cancel_latched = True

    self.cancel = self.cancel_latched

  def update(self, CS, long_plan, radar_state, LP_SP, personality: int, v_cruise_min_ms: float,
             plan_valid: bool, radar_valid: bool) -> None:
    self.v_target_sp = LP_SP.vTarget
    lead = radar_state.leadOne if radar_valid else None

    self.update_lead(lead, plan_valid, long_plan)
    self.v_target_ms = self.update_target(CS, long_plan, plan_valid, personality)
    self.update_escalation(CS, long_plan, plan_valid, v_cruise_min_ms)
