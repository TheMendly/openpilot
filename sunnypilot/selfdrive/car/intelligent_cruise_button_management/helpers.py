"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.constants import CV
from openpilot.selfdrive.modeld.constants import ModelConstants


def get_minimum_set_speed(is_metric: bool) -> int:
  return 30 if is_metric else 20


# --- pseudo-ACC math -------------------------------------------------------
#
# A non-SCC platform has no braking authority: the only way to slow down is to
# lower the stock cruise set speed and coast. Every constant below is therefore
# tuned to shed speed early rather than accurately.

# Following policy. Longer than a real ACC would use, on purpose.
D_MIN = 5.0  # m, buffer kept on top of the time gap
K_DIST = 0.35  # 1/s, distance error gain of the follow law

# Time gap per longitudinal personality, in log.LongitudinalPersonality order.
FOLLOW_TIME_GAP = (1.15, 1.45, 1.80)  # aggressive, standard, relaxed

# How far ahead we read the longitudinal plan, scheduled by speed. City needs a
# short horizon to stay responsive, highway needs a long one because coasting
# sheds speed very slowly.
LOOKAHEAD_BP = [30.0 * CV.KPH_TO_MS, 70.0 * CV.KPH_TO_MS, 130.0 * CV.KPH_TO_MS]
LOOKAHEAD_V = [2.0, 4.0, 7.0]  # s

# Deceleration reachable by closing the throttle (engine braking plus drag).
# Measured on ~1 h of Bayon logs (both pedals up, losing speed, going straight):
# the weakest 20% of observed coasting was -0.246 m/s^2 at 0-40 km/h, -0.248 at
# 60-80 and -0.316 at 100-130. It is essentially flat, not rising with speed, so
# the values below sit just under the weakest observed. Under-promising is the
# safe direction: it warns the driver early rather than late.
COAST_DECEL_BP = [30.0 * CV.KPH_TO_MS, 70.0 * CV.KPH_TO_MS, 130.0 * CV.KPH_TO_MS]
COAST_DECEL_V = [-0.24, -0.24, -0.30]  # m/s^2


def time_gap_for_personality(personality: int) -> float:
  return FOLLOW_TIME_GAP[int(np.clip(personality, 0, len(FOLLOW_TIME_GAP) - 1))]


def lookahead_time(v_ego: float) -> float:
  return float(np.interp(v_ego, LOOKAHEAD_BP, LOOKAHEAD_V))


def coast_decel_authority(v_ego: float) -> float:
  return float(np.interp(v_ego, COAST_DECEL_BP, COAST_DECEL_V))


def desired_follow_distance(v_ego: float, t_gap: float) -> float:
  return D_MIN + max(v_ego, 0.0) * t_gap


def plan_speed_at(speeds, t: float) -> float | None:
  """Speed the longitudinal plan predicts at time t, clamped to the plan horizon.

  longitudinalPlan.speeds is published on ModelConstants.T_IDXS[:CONTROL_N], which
  covers only about 2.5 s - far less than the 10 s of the model trajectory.
  Clamping rather than extrapolating keeps this honest: past the horizon the
  follow law takes over.
  """
  if len(speeds) == 0:
    return None

  t_idxs = ModelConstants.T_IDXS[:len(speeds)]
  return float(np.interp(min(t, t_idxs[-1]), t_idxs, speeds))


def follow_target_speed(v_ego: float, d_rel: float, v_lead: float, t_gap: float) -> float:
  """Constant time-gap follow law.

  Unlike the plan trajectory this has no horizon limit, which is what makes
  highway following work: at 130 km/h we have to start shedding speed long
  before the lead enters the plan's few-second window.
  """
  d_err = d_rel - desired_follow_distance(v_ego, t_gap)
  return max(0.0, v_lead + K_DIST * d_err)


def required_decel(v_ego: float, v_lead: float, distance: float) -> float:
  """Deceleration needed to stop closing on the lead within `distance`.

  Deliberately framed against the lead's speed, not against a wished-for target:
  the follow law drops its target to zero once the gap is too small, and asking
  what it takes to reach zero within the last few metres yields numbers with no
  physical meaning (hundreds of m/s^2) that would fire every warning we have.
  """
  closing = v_ego - v_lead
  if closing <= 0.0:
    return 0.0

  return -(closing ** 2) / (2.0 * max(distance, 1.0))
