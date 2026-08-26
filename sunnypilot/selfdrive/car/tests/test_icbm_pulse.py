"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from opendbc.car import DT_CTRL, structs
from opendbc.car.hyundai.values import CAR, Buttons
from opendbc.sunnypilot.car.hyundai.icbm import CLU11_SYNC_FIELDS, TAP_FRAMES, TAP_GAP, \
  IntelligentCruiseButtonManagementInterface

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BAYON = CAR.HYUNDAI_BAYON_1ST_GEN_NON_SCC
STOCK_CLU11_PERIOD = 2  # card frames per stock CLU11 frame (100 Hz card, ~50 Hz cluster)


class RecordingPacker:
  """Duck-typed CANPacker that keeps the signal values instead of encoding them."""

  def __init__(self):
    self.msgs = []

  def make_can_msg(self, name, bus, values):
    self.msgs.append((name, bus, dict(values)))
    return (name, b"", bus)


def make_cs(counter):
  clu11 = dict.fromkeys(CLU11_SYNC_FIELDS, 0)
  clu11["CF_Clu_AliveCnt1"] = counter
  return SimpleNamespace(clu11=clu11, is_metric=True)


def make_cc_sp(send_button=SendButtonState.none, cancel=False):
  icbm = SimpleNamespace(sendButton=send_button, cancel=cancel, vTarget=0.0)
  return SimpleNamespace(intelligentCruiseButtonManagement=icbm)


def make_interface(fingerprint=BAYON):
  CP = structs.CarParams()
  CP.carFingerprint = fingerprint
  CP_SP = structs.CarParamsSP()
  return IntelligentCruiseButtonManagementInterface(CP, CP_SP)


def run(icbm, packer, frames, cc_sp):
  """Drive the interface at 100 Hz against a ~50 Hz stock CLU11, recording activity."""
  active_frames = []
  last_button_frame = 0
  for frame in range(frames):
    CS = make_cs((frame // STOCK_CLU11_PERIOD) % 0x10)
    msgs = icbm.update(CS, cc_sp, packer, frame, last_button_frame, None)
    last_button_frame = icbm.last_button_frame
    if msgs:
      active_frames.append(frame)
  return active_frames


def bursts(frames):
  """Group frames into (start, count) runs, a gap being anything longer than one
  stock CLU11 period."""
  out = []
  for f in frames:
    if out and f - out[-1][2] <= STOCK_CLU11_PERIOD:
      start, count, _ = out[-1]
      out[-1] = (start, count + 1, f)
    else:
      out.append((f, 1, f))
  return [(start, count, last) for start, count, last in out]


class TestIcbmBayonPulse:
  def test_no_button_sends_nothing_and_resets_state(self):
    icbm, packer = make_interface(), RecordingPacker()
    icbm.tap_frames = 3
    icbm.last_clu11_counter = 7

    assert run(icbm, packer, 50, make_cc_sp()) == []
    assert icbm.tap_frames == 0
    assert icbm.last_clu11_counter is None

  def test_one_message_per_new_stock_counter(self):
    icbm, packer = make_interface(), RecordingPacker()
    sent = run(icbm, packer, TAP_FRAMES * STOCK_CLU11_PERIOD, make_cc_sp(SendButtonState.decrease))

    # the cluster counter only advances every other card frame; never duplicate it
    assert len(sent) == TAP_FRAMES
    assert all(b - a >= STOCK_CLU11_PERIOD for a, b in zip(sent, sent[1:], strict=False))

  def test_set_decel_is_pulsed_not_held(self):
    icbm, packer = make_interface(), RecordingPacker()
    groups = bursts(run(icbm, packer, 600, make_cc_sp(SendButtonState.decrease)))

    assert len(groups) >= 3, "expected several discrete taps"
    assert all(count == TAP_FRAMES for _, count, _ in groups)

    # each tap is followed by real silence, which is what lets speedCluster settle
    for (_, _, last), (next_start, _, _) in zip(groups, groups[1:], strict=False):
      assert (next_start - last) * DT_CTRL >= TAP_GAP

  def test_cancel_is_continuous(self):
    icbm, packer = make_interface(), RecordingPacker()
    sent = run(icbm, packer, 200, make_cc_sp(SendButtonState.none, cancel=True))

    # CANCEL is idempotent and must land now: one message per counter, no gaps
    assert len(bursts(sent)) == 1
    assert len(sent) == 200 // STOCK_CLU11_PERIOD

  def test_cancel_wins_over_a_pending_button(self):
    icbm, packer = make_interface(), RecordingPacker()
    run(icbm, packer, 20, make_cc_sp(SendButtonState.increase, cancel=True))
    assert all(m[2]["CF_Clu_CruiseSwState"] == Buttons.CANCEL for m in packer.msgs)

  def test_counter_is_the_next_expected_value(self):
    icbm, packer = make_interface(), RecordingPacker()
    icbm.update(make_cs(9), make_cc_sp(SendButtonState.increase), packer, 0, 0, None)

    _, bus, values = packer.msgs[0]
    assert bus == 0
    assert values["CF_Clu_AliveCnt1"] == 10
    assert values["CF_Clu_CruiseSwState"] == Buttons.RES_ACCEL

  def test_counter_wraps_at_four_bits(self):
    icbm, packer = make_interface(), RecordingPacker()
    icbm.update(make_cs(15), make_cc_sp(SendButtonState.decrease), packer, 0, 0, None)
    assert packer.msgs[0][2]["CF_Clu_AliveCnt1"] == 0

  def test_other_platforms_keep_the_generic_path(self):
    icbm = make_interface(CAR.HYUNDAI_ELANTRA_2021)
    assert not icbm.counter_sync


class TestIcbmDbc:
  """The synced frame is built by hand, so make sure every signal it names really
  exists in the Bayon's DBC."""

  def test_signals_exist_in_dbc(self):
    CANPacker = pytest.importorskip("opendbc.can").CANPacker
    from opendbc.car import Bus
    from opendbc.car.hyundai.values import DBC

    icbm = make_interface()
    packer = CANPacker(DBC[BAYON][Bus.pt])
    msgs = icbm.update(make_cs(3), make_cc_sp(SendButtonState.decrease), packer, 0, 0, None)
    assert len(msgs) == 1
    assert msgs[0][0] == 0x4F1  # CLU11
