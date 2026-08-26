#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Measure a car's real coast-down deceleration and how often pseudo-ACC would
escalate, from recorded logs.

COAST_DECEL_V in helpers.py is the one constant pseudo-ACC cannot guess: it is how
hard the car slows with both pedals up, and it decides when the driver is told that
coasting will not be enough. Promising more deceleration than the car delivers means
warning late, so the constant is derived here rather than estimated.

Run it on the device, offroad:

    cd /data/openpilot
    PYTHONPATH=/data/openpilot python sunnypilot/selfdrive/car/intelligent_cruise_button_management/tools/coast_calibration.py <route-prefix> [segments]

The route prefix is a directory name in /data/media/0/realdata without the segment
suffix, e.g. 00000044--b482fbc813. Set COAST_DECEL_V slightly weaker than the
"weakest 20%" column, which is what the shipped values do.

The escalation counts are open loop: the log was recorded with the set speed doing
whatever it did at the time, not what pseudo-ACC would have commanded. Read them as
an order of magnitude, not a prediction.
"""
import glob
import sys

import numpy as np
import zstandard

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import \
  coast_decel_authority, get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.pseudo_acc import \
  BRAKE_REQUIRED_T, FLOOR_CANCEL_MARGIN, FLOOR_CANCEL_T, HARD_DECEL_A, HARD_DECEL_T, \
  STOP_CANCEL_DIST, PseudoAcc

V_CRUISE_MIN_MS = get_minimum_set_speed(True) * CV.KPH_TO_MS
BINS = [(20, 40), (40, 60), (60, 80), (80, 100), (100, 130)]

# True coasting: both pedals up, actually losing speed, going roughly straight, and
# the stock cruise not holding a speed. Without that last condition the sample fills
# up with cruise-controlled driving, where the car is on the throttle and barely
# decelerating, and the answer comes out far too weak.
A_COAST_MAX = -0.05  # m/s^2
STEER_MAX_DEG = 30.0
V_MIN_SAMPLE = 5.0  # m/s


def read_segment(path):
  with open(path, 'rb') as f:
    return log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(f).read())


def main(prefix: str, max_segments: int) -> None:
  segs = sorted(glob.glob(f"/data/media/0/realdata/{prefix}--*/rlog.zst"))[:max_segments]
  if not segs:
    print(f"no segments matching {prefix}")
    return
  print(f"segments: {len(segs)}", flush=True)

  coast: dict = {b: [] for b in BINS}
  pa = PseudoAcc()
  CS = plan = radar = lp_sp = None
  engaged = 0

  brake_t = floor_t = hard_t = 0.0
  events = dict.fromkeys(["brakeRequired", "shouldStop", "floor", "hardDecel"], 0)
  latched = dict.fromkeys(events, False)

  for i, seg in enumerate(segs):
    try:
      messages = read_segment(seg)
    except Exception as e:
      print(f"  skip {seg}: {e}", flush=True)
      continue

    for msg in messages:
      which = msg.which()
      if which == 'carState':
        CS = msg.carState
      elif which == 'longitudinalPlan':
        plan = msg.longitudinalPlan
      elif which == 'radarState':
        radar = msg.radarState
      elif which == 'longitudinalPlanSP':
        lp_sp = msg.longitudinalPlanSP
      else:
        continue
      if which != 'carState' or CS is None:
        continue

      if (not CS.gasPressed and not CS.brakePressed and not CS.cruiseState.enabled and
          CS.vEgo > V_MIN_SAMPLE and abs(CS.steeringAngleDeg) < STEER_MAX_DEG and CS.aEgo < A_COAST_MAX):
        kph = CS.vEgo * CV.MS_TO_KPH
        for b in BINS:
          if b[0] <= kph < b[1]:
            coast[b].append(CS.aEgo)
            break

      if None in (plan, radar, lp_sp):
        continue
      pa.update(CS, plan, radar, lp_sp, 1, V_CRUISE_MIN_MS, True, True)

      # Only engaged frames count: run() clears every flag when the stock cruise is
      # not enabled, so unengaged frames would flatter the numbers.
      if not (CS.cruiseState.enabled and CS.vEgo > 1.0):
        continue
      engaged += 1

      brake_t = brake_t + DT_CTRL if (pa.lead_active and pa.a_req < pa.a_coast) else 0.0
      floor_t = floor_t + DT_CTRL if (pa.lead_active and pa.v_target_ms < V_CRUISE_MIN_MS - FLOOR_CANCEL_MARGIN) else 0.0
      hard_t = hard_t + DT_CTRL if (pa.lead_active and pa.a_req < HARD_DECEL_A) else 0.0

      now = {
        "brakeRequired": brake_t >= BRAKE_REQUIRED_T,
        "shouldStop": bool(plan.shouldStop and pa.lead_active and pa.lead_d_rel < STOP_CANCEL_DIST),
        "floor": floor_t >= FLOOR_CANCEL_T,
        "hardDecel": hard_t >= HARD_DECEL_T,
      }
      for key, on in now.items():
        if on and not latched[key]:
          events[key] += 1
        latched[key] = on

    print(f"  [{i + 1}/{len(segs)}] engaged={engaged}", flush=True)

  print("\n=== coast deceleration (both pedals up, stock cruise off, losing speed, straight) ===")
  print(f"{'band':>13} {'n':>7} {'weakest 20%':>12} {'median':>9} {'model now':>11}")
  for b in BINS:
    a = np.array(coast[b])
    if len(a) < 100:
      print(f"{f'{b[0]}-{b[1]} km/h':>13} {len(a):>7}   (too few samples)")
      continue
    mid = (b[0] + b[1]) / 2 * CV.KPH_TO_MS
    band = f"{b[0]}-{b[1]} km/h"
    print(f"{band:>13} {len(a):>7} {np.percentile(a, 80):>12.3f} {np.median(a):>9.3f} "
          + f"{coast_decel_authority(mid):>11.3f}")
  print("  set COAST_DECEL_V just under the weakest 20% column")

  hours = engaged * DT_CTRL / 3600.0
  print(f"\n=== escalation, stock cruise engaged only ({hours:.2f} h) ===")
  for key, count in events.items():
    print(f"  {key:>14}: {count:>4} events  {count / max(hours, 1e-9):>6.1f}/h")


if __name__ == "__main__":
  main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 60)
