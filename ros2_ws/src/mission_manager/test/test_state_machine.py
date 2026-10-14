import pytest

from mission_manager.safety_validator import SafetyError, SafetyLimits, SafetyValidator
from mission_manager.state_machine import MissionStateMachine, State


def test_nominal_sequence():
    sm = MissionStateMachine()
    for action, after in (('TAKEOFF', State.HOVER), ('GOTO', State.GOAL_REACHED), ('HOVER', State.HOVER),
                          ('RETURN_HOME', State.GOAL_REACHED), ('LAND', State.IDLE)):
        assert sm.can_start(action)
        sm.start(action)
        sm.finish(True)
        assert sm.state == after


def test_llm_cannot_bypass():
    sm = MissionStateMachine()
    assert not sm.can_start('GOTO')          # must take off first
    with pytest.raises(ValueError):
        sm.start('LAND')
    sm.start('TAKEOFF'); sm.finish(False, 'TRACKING_LOST')
    assert sm.state == State.ERROR and sm.error_reason == 'TRACKING_LOST'
    assert sm.can_start('LAND') and not sm.can_start('GOTO')


def test_safety_validator():
    v = SafetyValidator(SafetyLimits(max_distance_from_home_m=5.0))
    v.check_goto((2.0, 2.0, 1.2), (0.0, 0.0, 0.0))
    with pytest.raises(SafetyError):
        v.check_goto((10.0, 0.0, 1.2), (0.0, 0.0, 0.0))
    with pytest.raises(SafetyError):
        v.check_goto((1.0, 0.0, 4.0), (0.0, 0.0, 0.0))
    goal = v.object_goal((3.0, 0.0, 0.4), (0.0, 0.0, 1.2), (0.0, 0.0, 0.0), 0.9)
    assert goal == pytest.approx((2.0, 0.0, 1.2))
    with pytest.raises(SafetyError):
        v.object_goal((3.0, 0.0, 0.4), (0.0, 0.0, 1.2), (0.0, 0.0, 0.0), 0.2)


def test_landed_during_land_action_does_not_break_finish():
    sm = MissionStateMachine()
    sm.start('TAKEOFF'); sm.finish(True)
    sm.start('LAND')
    sm.landed()                      # vehicle_state callback fires before the action result arrives
    assert sm.state == State.LANDING and sm.current_action == 'LAND'
    sm.finish(True)
    assert sm.state == State.IDLE and sm.current_action == ''
    sm.landed()                      # idle + landed again: harmless
    assert sm.state == State.IDLE


def test_cancel_restores_previous_state():
    sm = MissionStateMachine()
    sm.start('TAKEOFF'); sm.finish(True)
    sm.start('GOTO_OBJECT')
    assert sm.state == State.NAVIGATING
    sm.cancel()                       # object not in map -> nothing was flown
    assert sm.state == State.HOVER and sm.current_action == ''
    assert sm.can_start('RETURN_HOME')
