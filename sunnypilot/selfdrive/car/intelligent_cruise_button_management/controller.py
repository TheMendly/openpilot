"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import car, custom
from opendbc.car import structs, apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.pseudo_acc import PseudoAcc, Source
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
VTargetSource = custom.IntelligentCruiseButtonManagement.VTargetSource

# PseudoAcc stays free of cereal so it can be unit tested on its own; map its
# plain source values onto the wire enum here.
V_TARGET_SOURCES = {
  Source.cruise: VTargetSource.cruise,
  Source.plan: VTargetSource.plan,
  Source.lead: VTargetSource.lead,
}

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4
DIRECTION_DWELL = 0.6  # s to settle before reversing RES+ <-> SET-
TARGET_SETTLE = 0.2  # s a higher target must hold before we chase it upwards


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params = None):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    self.cruise_button_timers = CRUISE_BUTTON_TIMER

    # Pseudo-ACC: lead-aware set speed management for platforms with no
    # longitudinal actuation of their own. Off unless both the platform supports
    # it and the driver opted in.
    params = params if params is not None else Params()
    self.pseudo_acc_enabled = bool(CP_SP.pseudoAccAvailable and params.get_bool("PseudoAcc"))
    self.pseudo_acc = PseudoAcc()

    self.cancel = False
    self.brake_required = False
    self.at_speed_floor = False
    self.v_target_source = VTargetSource.cruise

    self.reversal_timer = 0
    self.last_direction = State.inactive
    self.v_target_pending = 0
    self.target_settle_timer = 0

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP, sm, personality: int) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

    if self.pseudo_acc_enabled:
      plan_valid = sm.alive['longitudinalPlan'] and sm.valid['longitudinalPlan']
      radar_valid = sm.alive['radarState'] and sm.valid['radarState']
      self.pseudo_acc.update(CS, sm['longitudinalPlan'], sm['radarState'], LP_SP, personality,
                             self.v_cruise_min * ms_conv, plan_valid, radar_valid)

      v_target_ms = self.pseudo_acc.v_target_ms
      self.cancel = self.pseudo_acc.cancel
      self.brake_required = self.pseudo_acc.brake_required
      self.at_speed_floor = self.pseudo_acc.at_speed_floor
      self.v_target_source = V_TARGET_SOURCES[self.pseudo_acc.source]
    else:
      v_target_ms = LP_SP.vTarget
      self.cancel = False
      self.brake_required = False
      self.at_speed_floor = False
      self.v_target_source = VTargetSource.cruise

    self.v_target_ms_last = apply_hysteresis(v_target_ms, self.v_target_ms_last, HYST_GAP * ms_conv)
    v_target_new = round(self.v_target_ms_last * speed_conv)

    # Slowing down is accepted immediately - it is the only braking we have. Coming
    # back up has to prove itself first, so a target flickering across a rounding
    # boundary cannot turn into a burst of RES+.
    if v_target_new <= self.v_target:
      self.v_target = v_target_new
      self.target_settle_timer = 0
    elif v_target_new == self.v_target_pending:
      self.target_settle_timer += 1
      if self.target_settle_timer >= int(TARGET_SETTLE / DT_CTRL):
        self.v_target = v_target_new
        self.target_settle_timer = 0
    else:
      self.target_settle_timer = 0

    self.v_target_pending = v_target_new

  def direction_blocked(self, direction) -> bool:
    """Hold still briefly after a burst before reversing, so the cluster feedback
    has time to settle and the two directions cannot chase each other."""
    return self.reversal_timer > 0 and direction != self.last_direction

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)
    self.reversal_timer = max(0, self.reversal_timer - 1)
    previous_state = self.state

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target > self.v_cruise_cluster and not self.direction_blocked(State.increasing):
              self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min \
                 and not self.direction_blocked(State.decreasing):
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    if previous_state in SEND_BUTTONS and self.state != previous_state:
      self.last_direction = previous_state
      self.reversal_timer = int(DIRECTION_DWELL / DT_CTRL)

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = (CC.enabled and CS.cruiseState.enabled and not CS.gasPressed and
             not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume)
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, sm,
          personality: int, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric

    self.update_calculations(CS, LP_SP, sm, personality)
    self.update_readiness(CS, CC)

    if not self.is_ready:
      # Nothing to manage and nothing to cancel: start the next engagement clean.
      self.pseudo_acc.reset()
      self.cancel = False
      self.brake_required = False
      self.at_speed_floor = False

    # Always run the machine so its timers keep advancing, then let a cancel
    # override the outcome.
    self.cruise_button = self.update_state_machine()

    if self.cancel:
      # Last resort: the stock cruise cannot go low enough, or coasting cannot
      # shed the speed in time. Drop out entirely and hand the car back. CANCEL
      # is its own command now, no longer a SET- carrying a sentinel speed.
      self.state = State.inactive
      self.cruise_button = SendButtonState.none

    self.is_ready_prev = self.is_ready
