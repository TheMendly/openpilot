"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import \
  DIRECTION_DWELL, INACTIVE_TIMER, IntelligentCruiseButtonManagement, SendButtonState, State
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed

V_MIN_KPH = get_minimum_set_speed(True)


class FakeParams:
  def __init__(self, pseudo_acc: bool):
    self.pseudo_acc = pseudo_acc

  def get_bool(self, key):
    return self.pseudo_acc if key == "PseudoAcc" else False


class FakeSubMaster:
  """Stands in for selfdrived's SubMaster."""

  def __init__(self, plan=None, radar=None, plan_ok=True, radar_ok=True):
    self.msgs = {
      'longitudinalPlan': plan if plan is not None else make_plan(),
      'radarState': radar if radar is not None else make_radar(),
    }
    self.alive = {'longitudinalPlan': plan_ok, 'radarState': radar_ok}
    self.valid = dict(self.alive)

  def __getitem__(self, key):
    return self.msgs[key]


def make_plan(speeds=None, has_lead=False, should_stop=False):
  return SimpleNamespace(speeds=speeds if speeds is not None else [40.0] * 17,
                         hasLead=has_lead, shouldStop=should_stop, aTarget=0.0)


def make_radar(lead=None):
  empty = SimpleNamespace(status=False, modelProb=0.0, dRel=0.0, yRel=0.0, vLead=0.0, vRel=0.0)
  return SimpleNamespace(leadOne=lead if lead is not None else empty)


def make_lead(d_rel=20.0, v_lead=0.0, v_rel=-15.0):
  return SimpleNamespace(status=True, modelProb=0.9, dRel=d_rel, yRel=0.0, vLead=v_lead, vRel=v_rel)


def make_cs(v_ego=25.0, set_speed_kph=100.0, enabled=True):
  set_speed = set_speed_kph * CV.KPH_TO_MS
  return SimpleNamespace(vEgo=v_ego, gasPressed=False,
                         cruiseState=SimpleNamespace(enabled=enabled, speed=set_speed, speedCluster=set_speed))


def make_cc(enabled=True):
  return SimpleNamespace(enabled=enabled,
                         cruiseControl=SimpleNamespace(override=False, cancel=False, resume=False))


def make_icbm(pseudo_acc=True):
  CP = structs.CarParams()
  CP.openpilotLongitudinalControl = False
  CP_SP = structs.CarParamsSP()
  CP_SP.pcmCruiseSpeed = False
  CP_SP.pseudoAccAvailable = pseudo_acc
  return IntelligentCruiseButtonManagement(CP, CP_SP, FakeParams(pseudo_acc))


def drive(icbm, seconds, cs, cc=None, sm=None, v_target_kph=100.0, personality=1):
  """Run the controller and collect the button it asked for on every frame."""
  cc = cc if cc is not None else make_cc()
  sm = sm if sm is not None else FakeSubMaster()
  LP_SP = SimpleNamespace(vTarget=v_target_kph * CV.KPH_TO_MS)

  buttons = []
  for _ in range(int(seconds / DT_CTRL)):
    icbm.run(cs, cc, LP_SP, sm, personality, True)
    buttons.append(icbm.cruise_button)
  return buttons


class TestIcbmStateMachine:
  def test_no_buttons_when_target_matches_cluster(self):
    icbm = make_icbm()
    assert set(drive(icbm, 3.0, make_cs(set_speed_kph=100.0))) == {SendButtonState.none}
    assert icbm.state == State.holding

  def test_decreases_towards_a_lower_target(self):
    icbm = make_icbm()
    sm = FakeSubMaster(plan=make_plan(speeds=[15.0] * 17))
    buttons = drive(icbm, 2.0, make_cs(set_speed_kph=100.0), sm=sm)

    assert SendButtonState.decrease in buttons
    assert SendButtonState.increase not in buttons

  def test_stops_at_the_stock_cruise_floor(self):
    icbm = make_icbm()
    sm = FakeSubMaster(plan=make_plan(speeds=[2.0] * 17))
    buttons = drive(icbm, 2.0, make_cs(set_speed_kph=V_MIN_KPH), sm=sm)

    # the stock cruise cannot hold anything lower, so there is nothing to press
    assert SendButtonState.decrease not in buttons
    assert icbm.at_speed_floor

  def test_target_wobble_across_a_rounding_boundary_sends_no_res(self):
    icbm = make_icbm()
    cs, cc, sm = make_cs(set_speed_kph=100.0), make_cc(), FakeSubMaster()

    buttons = []
    for i in range(600):
      wobble = (100.4 if i % 2 else 99.6) * CV.KPH_TO_MS
      icbm.run(cs, cc, SimpleNamespace(vTarget=wobble), sm, 1, True)
      buttons.append(icbm.cruise_button)

    assert SendButtonState.increase not in buttons

  def test_reversal_is_held_off(self):
    icbm = make_icbm()
    # cluster below the target, so the controller starts out pressing RES+
    cs, cc, sm = make_cs(set_speed_kph=80.0), make_cc(), FakeSubMaster()
    for _ in range(int(2.0 / DT_CTRL)):
      icbm.run(cs, cc, SimpleNamespace(vTarget=100.0 * CV.KPH_TO_MS), sm, 1, True)
    assert icbm.state == State.increasing

    # flipping the target below the cluster is accepted instantly, but SET- must
    # still wait out the reversal dwell instead of following on the next frame
    low = SimpleNamespace(vTarget=60.0 * CV.KPH_TO_MS)
    first_decrease = None
    for i in range(int(2.0 / DT_CTRL)):
      icbm.run(cs, cc, low, sm, 1, True)
      if icbm.cruise_button == SendButtonState.decrease and first_decrease is None:
        first_decrease = i * DT_CTRL

    assert first_decrease is not None
    # without the dwell this would land at INACTIVE_TIMER
    assert first_decrease >= DIRECTION_DWELL - DT_CTRL, (first_decrease, INACTIVE_TIMER)

  def test_disengaged_stock_cruise_sends_nothing(self):
    icbm = make_icbm()
    sm = FakeSubMaster(plan=make_plan(speeds=[15.0] * 17))
    buttons = drive(icbm, 2.0, make_cs(set_speed_kph=100.0, enabled=False), sm=sm)

    assert set(buttons) == {SendButtonState.none}
    assert not icbm.cancel

  def test_driver_on_the_gas_stops_management(self):
    icbm = make_icbm()
    cs = make_cs(set_speed_kph=100.0)
    cs.gasPressed = True
    sm = FakeSubMaster(plan=make_plan(speeds=[15.0] * 17))

    assert set(drive(icbm, 2.0, cs, sm=sm)) == {SendButtonState.none}


class TestIcbmCancelPath:
  def test_cancel_stops_buttons_and_is_published(self):
    icbm = make_icbm()
    sm = FakeSubMaster(plan=make_plan(speeds=[0.0] * 17, has_lead=True, should_stop=True),
                       radar=make_radar(make_lead()))
    buttons = drive(icbm, 2.0, make_cs(v_ego=15.0, set_speed_kph=50.0), sm=sm, v_target_kph=50.0)

    assert icbm.cancel
    assert icbm.state == State.inactive
    # CANCEL is its own command now, not a SET- carrying a sentinel speed
    assert buttons[-1] == SendButtonState.none

  def test_pseudo_acc_off_leaves_plain_icbm_untouched(self):
    icbm = make_icbm(pseudo_acc=False)
    sm = FakeSubMaster(plan=make_plan(speeds=[0.0] * 17, has_lead=True, should_stop=True),
                       radar=make_radar(make_lead()))
    buttons = drive(icbm, 2.0, make_cs(v_ego=15.0, set_speed_kph=50.0), sm=sm, v_target_kph=50.0)

    # a stopping lead must not move the set speed at all when pseudo-ACC is off
    assert not icbm.cancel
    assert set(buttons) == {SendButtonState.none}
