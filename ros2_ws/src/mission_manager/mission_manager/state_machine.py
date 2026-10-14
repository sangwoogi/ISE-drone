"""Deterministic mission state machine (plan §28). Pure Python, unit-testable without ROS.

States:  IDLE -> ARMING -> TAKEOFF -> HOVER -> NAVIGATING -> GOAL_REACHED -> HOVER -> LANDING -> LANDED(IDLE)
Errors:  TRACKING_LOST, COMMUNICATION_LOST, INVALID_GOAL, MAP_UNAVAILABLE, LOW_CONFIDENCE, FLIGHT_ERROR
The LLM (or any client) can only request actions; transitions are decided here.
"""
from __future__ import annotations

from enum import Enum
from typing import Dict, Set


class State(str, Enum):
    IDLE = 'IDLE'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    HOVER = 'HOVER'
    NAVIGATING = 'NAVIGATING'
    GOAL_REACHED = 'GOAL_REACHED'
    LANDING = 'LANDING'
    EXPLORING = 'EXPLORING'
    ERROR = 'ERROR'


ERROR_REASONS = {'TRACKING_LOST', 'COMMUNICATION_LOST', 'INVALID_GOAL', 'MAP_UNAVAILABLE', 'LOW_CONFIDENCE', 'FLIGHT_ERROR'}

# action -> states in which it may start
ALLOWED: Dict[str, Set[State]] = {
    'TAKEOFF': {State.IDLE},
    'LAND': {State.HOVER, State.GOAL_REACHED, State.NAVIGATING, State.EXPLORING, State.ERROR},
    'HOVER': {State.HOVER, State.GOAL_REACHED},
    'GOTO': {State.HOVER, State.GOAL_REACHED},
    'RETURN_HOME': {State.HOVER, State.GOAL_REACHED, State.NAVIGATING, State.EXPLORING},
    'GOTO_OBJECT': {State.HOVER, State.GOAL_REACHED},
    'LOOK_AT_OBJECT': {State.HOVER, State.GOAL_REACHED},
    'EXPLORE': {State.HOVER, State.GOAL_REACHED},
    'SEARCH_OBJECT': {State.HOVER, State.GOAL_REACHED},
}

# action -> state while running
RUNNING: Dict[str, State] = {
    'TAKEOFF': State.TAKEOFF, 'LAND': State.LANDING, 'HOVER': State.HOVER, 'GOTO': State.NAVIGATING,
    'RETURN_HOME': State.NAVIGATING, 'GOTO_OBJECT': State.NAVIGATING, 'LOOK_AT_OBJECT': State.NAVIGATING,
    'EXPLORE': State.EXPLORING, 'SEARCH_OBJECT': State.EXPLORING,
}

# action -> state after success
DONE: Dict[str, State] = {
    'TAKEOFF': State.HOVER, 'LAND': State.IDLE, 'HOVER': State.HOVER, 'GOTO': State.GOAL_REACHED,
    'RETURN_HOME': State.GOAL_REACHED, 'GOTO_OBJECT': State.GOAL_REACHED, 'LOOK_AT_OBJECT': State.GOAL_REACHED,
    'EXPLORE': State.HOVER, 'SEARCH_OBJECT': State.GOAL_REACHED,
}


class MissionStateMachine:
    def __init__(self):
        self.state = State.IDLE
        self.error_reason = ''
        self.current_action = ''
        self.history = []
        self._previous = State.IDLE

    def can_start(self, action: str) -> bool:
        return action in ALLOWED and self.state in ALLOWED[action]

    def start(self, action: str) -> None:
        if not self.can_start(action):
            raise ValueError(f'action {action} not allowed in state {self.state.value}')
        self._previous = self.state
        self._set(RUNNING[action])
        self.current_action = action
        if action == 'TAKEOFF':
            self.history.append(State.ARMING)   # arming happens inside px4_bridge TAKEOFF; recorded for traceability

    def finish(self, success: bool, reason: str = '') -> None:
        action, self.current_action = self.current_action, ''
        if not action:
            # landed() already resolved the action (e.g. LAND confirmed by vehicle state) - nothing to do
            return
        if success:
            self._set(DONE[action])
            self.error_reason = ''
        else:
            self.error_reason = reason if reason in ERROR_REASONS else 'FLIGHT_ERROR'
            # after a failed LAND we cannot know; stay in ERROR until operator/auto-land resolves
            self._set(State.ERROR)

    def cancel(self) -> None:
        """The action was rejected before any flight command was issued (safety / unknown object):
        the vehicle is exactly where it was, so return to the previous state instead of ERROR."""
        if not self.current_action:
            return
        self.current_action = ''
        self._set(getattr(self, '_previous', State.IDLE))

    def landed(self) -> None:
        """External confirmation (vehicle_state.landed and disarmed) clears ERROR/LANDING.
        Ignored while an action is still executing; that action's finish() will set the state."""
        if self.current_action:
            return
        self._set(State.IDLE)
        self.error_reason = ''

    def _set(self, s: State) -> None:
        self.state = s
        self.history.append(s)
