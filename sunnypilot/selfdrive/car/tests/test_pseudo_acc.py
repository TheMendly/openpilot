"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import \
  D_MIN, coast_decel_authority, desired_follow_distance, follow_target_speed, lookahead_time, \
  plan_speed_at, required_decel, time_gap_for_personality
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.pseudo_acc import \
  A_UP_MAX, LEAD_ACQUIRE_T, LEAD_RELEASE_T, PseudoAcc, Source

V_CRUISE_MIN_MS = 30.0 * CV.KPH_TO_MS


def make_lead(d_rel=60.0, v_lead=20.0, v_rel=-5.0, prob=0.9, y_rel=0.0, status=True):
  return SimpleNamespace(status=status, modelProb=prob, dRel=d_rel, yRel=y_rel, vLead=v_lead, vRel=v_rel)


NO_LEAD = make_lead(d_rel=0.0, v_lead=0.0, v_rel=0.0, prob=0.0, status=False)


def make_radar(lead=None):
  return SimpleNamespace(leadOne=lead if lead is not None else NO_LEAD)


def make_plan(speeds=None, has_lead=True, should_stop=False):
  return SimpleNamespace(speeds=speeds if speeds is not None else [30.0] * 17,
                         hasLead=has_lead, shouldStop=should_stop, aTarget=0.0)


def make_cs(v_ego=25.0, set_speed=100.0 * CV.KPH_TO_MS, enabled=True):
  return SimpleNamespace(vEgo=v_ego, gasPressed=False,
                         cruiseState=SimpleNamespace(enabled=enabled, speed=set_speed, speedCluster=set_speed))


def make_lp_sp(v_target=30.0):
  return SimpleNamespace(vTarget=v_target)


def run_for(pa, seconds, cs=None, plan=None, radar=None, lp_sp=None, personality=1,
            plan_valid=True, radar_valid=True):
  cs = cs if cs is not None else make_cs()
  plan = plan if plan is not None else make_plan()
  radar = radar if radar is not None else make_radar()
  lp_sp = lp_sp if lp_sp is not None else make_lp_sp()

  for _ in range(max(1, int(seconds / DT_CTRL))):
    pa.update(cs, plan, radar, lp_sp, personality, V_CRUISE_MIN_MS, plan_valid, radar_valid)


class TestPseudoAccHelpers:
  def test_time_gap_ordering_and_clamping(self):
    assert time_gap_for_personality(0) < time_gap_for_personality(1) < time_gap_for_personality(2)
    assert time_gap_for_personality(-5) == time_gap_for_personality(0)
    assert time_gap_for_personality(99) == time_gap_for_personality(2)

  def test_lookahead_grows_with_speed(self):
    slow = lookahead_time(30.0 * CV.KPH_TO_MS)
    fast = lookahead_time(130.0 * CV.KPH_TO_MS)
    assert 1.0 < slow < fast < 12.0

  def test_coast_authority_is_negative_and_grows_with_speed(self):
    slow = coast_decel_authority(30.0 * CV.KPH_TO_MS)
    fast = coast_decel_authority(130.0 * CV.KPH_TO_MS)
    assert fast < slow < 0.0

  def test_desired_follow_distance(self):
    assert desired_follow_distance(0.0, 1.45) == D_MIN
    assert desired_follow_distance(30.0, 1.45) > desired_follow_distance(30.0, 1.15)

  def test_follow_law_signs(self):
    t_gap = 1.45
    v_ego, v_lead = 30.0, 30.0
    at_gap = desired_follow_distance(v_ego, t_gap)

    # exactly at the desired gap, match the lead
    assert follow_target_speed(v_ego, at_gap, v_lead, t_gap) == v_lead
    # too close asks for less than the lead, too far asks for more
    assert follow_target_speed(v_ego, at_gap - 20.0, v_lead, t_gap) < v_lead
    assert follow_target_speed(v_ego, at_gap + 20.0, v_lead, t_gap) > v_lead
    # never negative
    assert follow_target_speed(v_ego, 1.0, 0.0, t_gap) == 0.0

  def test_plan_speed_clamps_to_horizon(self):
    speeds = [30.0 - i for i in range(17)]
    assert plan_speed_at([], 2.0) is None
    assert plan_speed_at(speeds, 0.0) == speeds[0]
    # beyond the published horizon we clamp instead of extrapolating
    assert plan_speed_at(speeds, 1e3) == speeds[-1]

  def test_required_decel(self):
    assert required_decel(30.0, 30.0, 50.0) == 0.0
    assert required_decel(30.0, 40.0, 50.0) == 0.0  # speeding up needs no braking
    near = required_decel(30.0, 20.0, 20.0)
    far = required_decel(30.0, 20.0, 200.0)
    assert near < far < 0.0


class TestPseudoAccLead:
  def test_lead_needs_to_be_stable_before_acting(self):
    pa = PseudoAcc()
    radar = make_radar(make_lead())

    run_for(pa, LEAD_ACQUIRE_T * 0.5, radar=radar)
    assert not pa.lead_active

    run_for(pa, LEAD_ACQUIRE_T, radar=radar)
    assert pa.lead_active

  def test_single_frame_dropout_does_not_release(self):
    pa = PseudoAcc()
    lead_radar, empty_radar = make_radar(make_lead()), make_radar()

    run_for(pa, 2.0, radar=lead_radar)
    assert pa.lead_active

    # one dropped frame in ten, far short of LEAD_RELEASE_T
    cs, plan, lp_sp = make_cs(), make_plan(), make_lp_sp()
    for i in range(200):
      radar = empty_radar if i % 10 == 0 else lead_radar
      pa.update(cs, plan, radar, lp_sp, 1, V_CRUISE_MIN_MS, True, True)
    assert pa.lead_active

    run_for(pa, LEAD_RELEASE_T * 1.5, radar=empty_radar, plan=make_plan(has_lead=False))
    assert not pa.lead_active

  def test_low_confidence_lead_is_ignored(self):
    pa = PseudoAcc()
    run_for(pa, 2.0, radar=make_radar(make_lead(prob=0.2)))
    assert not pa.lead_active

  def test_off_path_lead_is_ignored(self):
    pa = PseudoAcc()
    run_for(pa, 2.0, radar=make_radar(make_lead(y_rel=4.0)))
    assert not pa.lead_active

  def test_planner_disagreement_is_ignored(self):
    pa = PseudoAcc()
    run_for(pa, 2.0, radar=make_radar(make_lead()), plan=make_plan(has_lead=False))
    assert not pa.lead_active


class TestPseudoAccTarget:
  def test_lead_lowers_target_below_cruise(self):
    pa = PseudoAcc()
    # cruising at 30 m/s with a slow lead close ahead
    run_for(pa, 2.0, cs=make_cs(v_ego=30.0), lp_sp=make_lp_sp(30.0),
            radar=make_radar(make_lead(d_rel=30.0, v_lead=18.0)))
    assert pa.source == Source.lead
    assert pa.v_target_ms < 30.0

  def test_upward_ramp_is_rate_limited(self):
    pa = PseudoAcc()
    run_for(pa, 2.0, cs=make_cs(v_ego=20.0), lp_sp=make_lp_sp(30.0),
            radar=make_radar(make_lead(d_rel=25.0, v_lead=15.0)))
    slowed = pa.v_target_ms
    assert slowed < 30.0

    # lead vanishes: the target must climb back gently, not snap to cruise
    elapsed = LEAD_RELEASE_T * 1.5 + 0.1
    run_for(pa, elapsed, cs=make_cs(v_ego=20.0), lp_sp=make_lp_sp(30.0),
            radar=make_radar(), plan=make_plan(has_lead=False))
    assert pa.v_target_ms <= slowed + A_UP_MAX * elapsed + 1e-6

  def test_stale_plan_never_asks_for_more_speed(self):
    pa = PseudoAcc()
    cs = make_cs(v_ego=20.0, set_speed=60.0 * CV.KPH_TO_MS)
    run_for(pa, 1.0, cs=cs, lp_sp=make_lp_sp(40.0), plan_valid=False, radar_valid=False)
    assert pa.v_target_ms <= cs.cruiseState.speedCluster + 1e-6

  def test_plan_trajectory_can_lower_the_target(self):
    pa = PseudoAcc()
    # the plan predicts a slowdown the follow law would not see
    run_for(pa, 1.0, cs=make_cs(v_ego=25.0), lp_sp=make_lp_sp(25.0),
            plan=make_plan(speeds=[25.0 - i for i in range(17)], has_lead=False),
            radar=make_radar())
    assert pa.source == Source.plan
    assert pa.v_target_ms < 25.0


class TestPseudoAccEscalation:
  def test_brake_required_when_coasting_is_not_enough(self):
    pa = PseudoAcc()
    # 30 m/s onto a much slower lead 25 m ahead: no coast can do this
    run_for(pa, 2.0, cs=make_cs(v_ego=30.0), lp_sp=make_lp_sp(30.0),
            radar=make_radar(make_lead(d_rel=25.0, v_lead=8.0, v_rel=-22.0)))
    assert pa.brake_required
    assert pa.a_req < pa.a_coast

  def test_gentle_approach_does_not_warn(self):
    pa = PseudoAcc()
    run_for(pa, 2.0, cs=make_cs(v_ego=25.0), lp_sp=make_lp_sp(25.0),
            radar=make_radar(make_lead(d_rel=120.0, v_lead=24.0, v_rel=-1.0)))
    assert not pa.brake_required
    assert not pa.cancel

  def test_should_stop_cancels_and_latches(self):
    pa = PseudoAcc()
    cs = make_cs(v_ego=15.0)
    run_for(pa, 1.0, cs=cs, plan=make_plan(should_stop=True),
            radar=make_radar(make_lead(d_rel=25.0, v_lead=0.0, v_rel=-15.0)))
    assert pa.cancel

    # stays latched while the stock cruise is still engaged, even if the scene clears
    run_for(pa, 1.0, cs=cs, plan=make_plan(has_lead=False), radar=make_radar())
    assert pa.cancel

    # rearms only once the stock cruise has actually dropped out
    run_for(pa, 0.1, cs=make_cs(v_ego=10.0, enabled=False), plan=make_plan(has_lead=False),
            radar=make_radar())
    assert not pa.cancel

  def test_at_speed_floor_flagged_below_stock_minimum(self):
    pa = PseudoAcc()
    run_for(pa, 2.0, cs=make_cs(v_ego=12.0), lp_sp=make_lp_sp(12.0),
            radar=make_radar(make_lead(d_rel=15.0, v_lead=3.0, v_rel=-9.0)))
    assert pa.at_speed_floor

  def test_reset_clears_everything(self):
    pa = PseudoAcc()
    run_for(pa, 2.0, cs=make_cs(v_ego=30.0),
            radar=make_radar(make_lead(d_rel=25.0, v_lead=8.0, v_rel=-22.0)))
    pa.reset()
    assert not pa.lead_active and not pa.brake_required and not pa.cancel and not pa.initialized
