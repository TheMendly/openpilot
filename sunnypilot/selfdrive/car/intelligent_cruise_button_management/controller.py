"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import cereal.messaging as messaging

from cereal import car, custom
from opendbc.car import structs, apply_hysteresis
from opendbc.car.hyundai.values import CAR
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4

# Bayon NON-SCC pseudo-ACC settings. The stock cruise remains responsible for
# throttle control; ICBM only emulates RES+, SET- and, through a sentinel,
# CANCEL. It never sends SCC acceleration or brake commands.
BAYON_LEAD_MIN_SPEED = 35.0 * CV.KPH_TO_MS
BAYON_LEAD_LOOKAHEAD_IDX = 16  # Model trajectory point at about 2.5 seconds
BAYON_HARD_DECEL_CANCEL = -0.8  # m/s^2
BAYON_CANCEL_SENTINEL = -1


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
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

    self.is_bayon_non_scc = self.CP.carFingerprint == CAR.HYUNDAI_BAYON_1ST_GEN_NON_SCC
    self.cancel_required = False
    self.long_plan_sm = messaging.SubMaster(['longitudinalPlan']) if self.is_bayon_non_scc else None

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    v_target_ms = LP_SP.vTarget
    self.cancel_required = False

    if self.long_plan_sm is not None:
      self.long_plan_sm.update(0)
      plan_valid = self.long_plan_sm.alive['longitudinalPlan'] and self.long_plan_sm.valid['longitudinalPlan']

      if plan_valid:
        LP = self.long_plan_sm['longitudinalPlan']
        lead_active = LP.hasLead and CS.vEgo >= BAYON_LEAD_MIN_SPEED

        if lead_active and len(LP.speeds):
          lookahead_idx = min(BAYON_LEAD_LOOKAHEAD_IDX, len(LP.speeds) - 1)
          v_target_ms = min(v_target_ms, LP.speeds[lookahead_idx])

        self.cancel_required = bool(
          LP.hasLead and (LP.shouldStop or LP.aTarget <= BAYON_HARD_DECEL_CANCEL)
        )
      elif CS.cruiseState.speedCluster > 0:
        # A missing/stale plan must never cause an automatic RES+ command.
        v_target_ms = min(v_target_ms, CS.cruiseState.speedCluster)

    self.v_target_ms_last = apply_hysteresis(v_target_ms, self.v_target_ms_last, HYST_GAP * ms_conv)

    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

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

            elif self.v_target > self.v_cruise_cluster:
              self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
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

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = (CC.enabled and CS.cruiseState.enabled and
             not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume)
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    if self.is_bayon_non_scc and self.cancel_required and self.is_ready:
      # Keep the existing Cap'n Proto enum unchanged. A negative vTarget paired
      # with decrease is interpreted as CANCEL only by the Bayon Hyundai ICBM.
      self.state = State.inactive
      self.v_target = BAYON_CANCEL_SENTINEL
      self.cruise_button = SendButtonState.decrease
    else:
      self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
