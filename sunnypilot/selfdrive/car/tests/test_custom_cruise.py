import pytest

from cereal import car, custom
from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized_class
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS, IMPERIAL_INCREMENT, V_CRUISE_INITIAL, VCruiseHelper
from openpilot.selfdrive.car.tests.test_cruise_speed import TestVCruiseHelper
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, \
  LONG_PRESS_MULTIPLIER, LONG_PRESS_MULTIPLIER_ICBM

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


# TODO: test pcmCruise and pcmCruiseSpeed
@parameterized_class(('pcm_cruise', 'pcm_cruise_speed'), [(False, True)])
class TestCustomAccIncrements(TestVCruiseHelper):
  def setup_method(self):
    TestVCruiseHelper.setup_method(self)
    self.params = Params()
    self.reset_custom_params()

  def reset_custom_params(self) -> None:
    """Reset to default custom ACC parameters"""
    self.params.put_bool("CustomAccIncrementsEnabled", False, block=True)
    self.params.put("CustomAccShortPressIncrement", 1, block=True)
    self.params.put("CustomAccLongPressIncrement", 5, block=True)
    self.v_cruise_helper.read_custom_set_speed_params()

  def press_button_short(self, button_type: car.CarState.ButtonEvent.Type) -> None:
    """Simulate a short button press (press + release)"""
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

  def press_button_long(self, button_type: car.CarState.ButtonEvent.Type) -> None:
    """Simulate a long button press (50+ frames)"""
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    # Hold for 50 frames to trigger long press
    CS.buttonEvents = []
    for _ in range(50):
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

  def set_custom_increments(self, enabled: bool, short_inc: int, long_inc: int) -> None:
    """Set custom ACC increment parameters"""
    self.params.put_bool("CustomAccIncrementsEnabled", enabled, block=True)
    self.params.put("CustomAccShortPressIncrement", short_inc, block=True)
    self.params.put("CustomAccLongPressIncrement", long_inc, block=True)
    self.v_cruise_helper.read_custom_set_speed_params()

  def test_default_behavior_when_disabled(self):
    """Test that default increments are used when custom ACC is disabled"""
    self.set_custom_increments(enabled=False, short_inc=5, long_inc=10)
    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph

    # Short press should increment by 1 (default)
    self.press_button_short(ButtonType.accelCruise)
    assert self.v_cruise_helper.v_cruise_kph == initial_speed + 1

  @pytest.mark.parametrize("increment", (1, 2, 3, 4, 5, 6, 7, 8, 9, 10))
  def test_custom_short_press_increments(self, increment):
    """Test custom short press increments (1-10)"""
    self.set_custom_increments(enabled=True, short_inc=increment, long_inc=5)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    self.press_button_short(ButtonType.accelCruise)

    if increment in (5, 10):
      # Should round to nearest increment
      expected_speed = ((initial_speed // increment) + 1) * increment
    else:
      expected_speed = initial_speed + increment

    assert self.v_cruise_helper.v_cruise_kph == expected_speed

  @pytest.mark.parametrize("increment", (1, 5, 10))
  def test_custom_long_press_increments(self, increment):
    """Test custom long press increments (1, 5, 10)"""
    self.set_custom_increments(enabled=True, short_inc=1, long_inc=increment)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    self.press_button_long(ButtonType.accelCruise)

    if increment in (5, 10):
      # Should round to nearest increment
      expected_speed = ((initial_speed // increment) + 1) * increment
    else:
      expected_speed = initial_speed + increment

    assert self.v_cruise_helper.v_cruise_kph == expected_speed

  @pytest.mark.parametrize("button_type", [ButtonType.accelCruise, ButtonType.decelCruise])
  def test_accel_decel_symmetry(self, button_type):
    """Test that acceleration and deceleration work symmetrically"""
    self.set_custom_increments(enabled=True, short_inc=3, long_inc=5)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    self.press_button_short(button_type)

    expected_change = 3 if button_type == ButtonType.accelCruise else -3
    assert self.v_cruise_helper.v_cruise_kph == initial_speed + expected_change

  def test_rounding_behavior(self):
    """Test rounding behavior for 5 and 10 increments"""
    test_cases = [
      (47, 5, 50),   # 47 -> 50 (round up to next 5)
      (45, 5, 50),   # 45 -> 50 (already at 5, increment by 5)
      (43, 10, 50),  # 43 -> 50 (round up to next 10)
      (40, 10, 50),  # 40 -> 50 (already at 10, increment by 10)
    ]

    for initial, increment, expected in test_cases:
      self.set_custom_increments(enabled=True, short_inc=increment, long_inc=increment)
      self.reset_cruise_speed_state()
      self.enable(initial * CV.KPH_TO_MS, False, False)

      self.press_button_short(ButtonType.accelCruise)
      assert self.v_cruise_helper.v_cruise_kph == expected

  def test_invalid_values_fallback(self):
    """Test that invalid values fallback to safe defaults"""
    # Test invalid short increment
    self.set_custom_increments(enabled=True, short_inc=-1, long_inc=5)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    self.press_button_short(ButtonType.accelCruise)
    assert self.v_cruise_helper.v_cruise_kph == initial_speed + 1  # Should fallback to 1

    # Test invalid long increment
    self.reset_cruise_speed_state()
    self.set_custom_increments(enabled=True, short_inc=1, long_inc=99)
    self.enable(50 * CV.KPH_TO_MS, False, False)

    initial_speed = self.v_cruise_helper.v_cruise_kph
    self.press_button_long(ButtonType.accelCruise)
    assert self.v_cruise_helper.v_cruise_kph == initial_speed + 10  # Should fallback to 10


class TestIcbmLongPressIncrement:
  """ICBM platforms have no set speed of their own: the driver's button goes to the
  car's cluster, which jumps straight to the next multiple of 10. Ours has to agree,
  or ICBM spends every long press tapping SET- to drag the cluster back down."""

  def setup_method(self):
    # CRUISE_BUTTON_TIMER is module level state shared with the ICBM controller
    for k in CRUISE_BUTTON_TIMER:
      CRUISE_BUTTON_TIMER[k] = 0

    self.CP = car.CarParams(pcmCruise=True)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)

    self.params = Params()
    self.set_custom_increments(False, 1, 5)

  def set_custom_increments(self, enabled: bool, short_inc: int, long_inc: int) -> None:
    self.params.put_bool("CustomAccIncrementsEnabled", enabled, block=True)
    self.params.put("CustomAccShortPressIncrement", short_inc, block=True)
    self.params.put("CustomAccLongPressIncrement", long_inc, block=True)
    self.v_cruise_helper.read_custom_set_speed_params()

  def make_cs(self, v_cruise_kph: float) -> car.CarState:
    speed = v_cruise_kph * CV.KPH_TO_MS
    return car.CarState(cruiseState={"available": True, "enabled": True,
                                     "speed": speed, "speedCluster": speed})

  def engage(self, v_cruise_kph: float) -> None:
    # update_enabled_state holds the first frame back, and until it settles
    # v_cruise_kph is seeded straight from cruiseState.speed
    CS = self.make_cs(v_cruise_kph)
    for _ in range(3):
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(v_cruise_kph)

  def press(self, button_type, long_press: bool, v_cruise_kph: float) -> None:
    CS = self.make_cs(v_cruise_kph)
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    CS.buttonEvents = []
    for _ in range(CRUISE_LONG_PRESS if long_press else 1):
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)

  @pytest.mark.parametrize("initial,expected", [(122, 130), (120, 130), (129, 130)])
  def test_long_press_accel_walks_the_decade_grid(self, initial, expected):
    self.engage(initial)
    self.press(ButtonType.accelCruise, True, initial)
    assert self.v_cruise_helper.v_cruise_kph == expected

  @pytest.mark.parametrize("initial,expected", [(122, 120), (130, 120), (121, 120)])
  def test_long_press_decel_walks_the_decade_grid(self, initial, expected):
    self.engage(initial)
    self.press(ButtonType.decelCruise, True, initial)
    assert self.v_cruise_helper.v_cruise_kph == expected

  def test_short_press_is_untouched(self):
    self.engage(122)
    self.press(ButtonType.accelCruise, False, 122)
    assert self.v_cruise_helper.v_cruise_kph == 123

  def test_the_users_own_increment_still_wins(self):
    self.set_custom_increments(True, 1, 5)
    self.engage(122)
    self.press(ButtonType.accelCruise, True, 122)
    assert self.v_cruise_helper.v_cruise_kph == 125

  def test_imperial_keeps_the_five_mph_step(self):
    helper = VCruiseHelper(self.CP, self.CP_SP)
    helper.get_minimum_set_speed(False)
    assert helper.update_v_cruise_delta(True, IMPERIAL_INCREMENT)[1] == \
           pytest.approx(LONG_PRESS_MULTIPLIER * IMPERIAL_INCREMENT)

  def test_openpilot_long_platforms_keep_the_five_step(self):
    helper = VCruiseHelper(car.CarParams(pcmCruise=False), custom.CarParamsSP(pcmCruiseSpeed=True))
    helper.get_minimum_set_speed(True)
    assert helper.update_v_cruise_delta(True, 1.)[1] == LONG_PRESS_MULTIPLIER

  def test_icbm_metric_long_press_uses_the_decade_step(self):
    self.v_cruise_helper.get_minimum_set_speed(True)
    assert self.v_cruise_helper.update_v_cruise_delta(True, 1.)[1] == LONG_PRESS_MULTIPLIER_ICBM
    assert self.v_cruise_helper.update_v_cruise_delta(False, 1.)[1] == 1
