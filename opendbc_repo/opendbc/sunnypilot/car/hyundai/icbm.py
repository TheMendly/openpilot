"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai import hyundaican, hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CANFD_CAR, CAR
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTON_COPIES = 2
BUTTON_COPIES_TIME = 7
BUTTON_COPIES_TIME_IMPERIAL = [BUTTON_COPIES_TIME + 3, 70]
BUTTON_COPIES_TIME_METRIC = [BUTTON_COPIES_TIME, 40]

# Platforms whose CLU11 receiver rejects the generic frame % 16 counter sequence.
# For those we rebuild the frame from the latest stock CLU11 and send the next
# expected AliveCnt1 value instead.
CLU11_COUNTER_SYNC_CAR = (CAR.HYUNDAI_BAYON_1ST_GEN_NON_SCC,)

CLU11_SYNC_FIELDS = (
  "CF_Clu_CruiseSwState",
  "CF_Clu_CruiseSwMain",
  "CF_Clu_SldMainSW",
  "CF_Clu_ParityBit1",
  "CF_Clu_VanzDecimal",
  "CF_Clu_Vanz",
  "CF_Clu_SPEED_UNIT",
  "CF_Clu_DetentOut",
  "CF_Clu_RheostatLevel",
  "CF_Clu_CluInfo",
  "CF_Clu_AmpInfo",
  "CF_Clu_AliveCnt1",
)

# RES+ / SET- are sent as discrete taps rather than a held button: one tap is one
# set-speed step, and the silence in between is what lets cruiseState.speedCluster
# settle so the controller closes the loop on a real value instead of guessing
# mid-press. CANCEL is not pulsed - it is idempotent and we want it to land now.
TAP_FRAMES = 4  # stock CLU11 frames per tap, ~80 ms at 50 Hz
TAP_GAP = 0.18  # s of silence between taps

BUTTONS = {
  SendButtonState.increase: Buttons.RES_ACCEL,
  SendButtonState.decrease: Buttons.SET_DECEL,
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.counter_sync = CP.carFingerprint in CLU11_COUNTER_SYNC_CAR
    self.last_clu11_counter = None
    self.tap_frames = 0
    self.tap_end_frame = 0

  def reset_button_state(self) -> None:
    self.last_clu11_counter = None
    self.tap_frames = 0
    self.tap_end_frame = 0

  def create_synced_clu11_message(self, packer, CS, send_button, pulsed: bool) -> list[CanData]:
    stock_counter = int(CS.clu11["CF_Clu_AliveCnt1"])

    # card runs at 100 Hz while the stock CLU11 is approximately 50 Hz. Send only
    # once per newly observed stock counter instead of flooding the bus with
    # duplicate or out-of-order counters.
    if stock_counter == self.last_clu11_counter:
      return []

    self.last_clu11_counter = stock_counter

    if pulsed:
      if self.tap_frames >= TAP_FRAMES:
        if (self.frame - self.tap_end_frame) * DT_CTRL < TAP_GAP:
          return []
        self.tap_frames = 0

      self.tap_frames += 1
      if self.tap_frames >= TAP_FRAMES:
        self.tap_end_frame = self.frame

    values = {signal: CS.clu11[signal] for signal in CLU11_SYNC_FIELDS}
    values["CF_Clu_CruiseSwState"] = send_button
    values["CF_Clu_AliveCnt1"] = (stock_counter + 1) % 0x10

    self.last_button_frame = self.frame
    return [packer.make_can_msg("CLU11", 0, values)]

  def create_can_mock_button_messages(self, packer, CS, send_button, pulsed: bool = True) -> list[CanData]:
    if self.counter_sync:
      return self.create_synced_clu11_message(packer, CS, send_button, pulsed)

    can_sends = []
    copies_xp = BUTTON_COPIES_TIME_METRIC if CS.is_metric else BUTTON_COPIES_TIME_IMPERIAL
    copies = int(np.interp(BUTTON_COPIES_TIME, copies_xp, [1, BUTTON_COPIES]))

    # send resume at a max freq of 10Hz
    if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
      # Send repeated messages to increase the likelihood of the stock cruise accepting the button press.
      can_sends.extend([hyundaican.create_clu11(packer, self.frame, CS.clu11, send_button, self.CP)] * copies)
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
        self.last_button_frame = self.frame

    return can_sends

  def create_canfd_mock_button_messages(self, packer, CS, CAN, send_button) -> list[CanData]:
    can_sends = []
    if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
      # TODO: resume for alt button cars
      pass
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          for _ in range(20):
            can_sends.append(hyundaicanfd.create_buttons(packer, self.CP, CAN, (CS.buttons_counter + button_counter_offset) % 0xF, send_button))
          self.last_button_frame = self.frame

    return can_sends

  def update(self, CS, CC_SP, packer, frame, last_button_frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    cancel = bool(self.ICBM.cancel)

    if not cancel and self.ICBM.sendButton == SendButtonState.none:
      self.reset_button_state()
      return can_sends

    send_button = Buttons.CANCEL if cancel else BUTTONS[self.ICBM.sendButton]

    if self.CP.carFingerprint in CANFD_CAR:
      can_sends.extend(self.create_canfd_mock_button_messages(packer, CS, CAN, send_button))
    else:
      can_sends.extend(self.create_can_mock_button_messages(packer, CS, send_button, pulsed=not cancel))

    return can_sends
