"""
Finite State Machine for robot supervisor.

Manages high-level behavior: WAIT_START → BOUNDARY_DISCOVERY → COVERAGE → DONE
"""

import time
from enum import Enum, auto
from typing import Tuple, Optional


class RobotState(Enum):
    """Robot operational states."""

    WAIT_START = auto()
    BOUNDARY_DISCOVERY = auto()
    COVERAGE = auto()
    DONE = auto()
    ERROR = auto()


class SupervisorFSM:
    """High-level state machine for robot behavior."""

    def __init__(self):
        """Initialize FSM."""
        self.state = RobotState.WAIT_START
        self.start_time = None
        self.boundary_start_time = None
        self.coverage_start_time = None

        # Timeouts
        self.boundary_timeout = 120.0  # seconds
        self.coverage_timeout = 180.0  # seconds

        # State tracking
        self.rectangle_confident = False
        self.coverage_complete = False

    def update(self, context: dict) -> RobotState:
        """
        Update FSM based on current context.

        Args:
            context: Dictionary with:
                - 'start_signal': bool, user start signal
                - 'rectangle_confident': bool, rectangle fit is confident
                - 'coverage_ratio': float, coverage completion ratio
                - 'coverage_planner_complete': bool, all coverage lanes completed
                - 'error': bool, error condition

        Returns:
            Current state after update
        """
        current_time = time.time()

        # Check for errors
        if context.get("error", False):
            self.state = RobotState.ERROR
            return self.state

        # State transitions
        if self.state == RobotState.WAIT_START:
            if context.get("start_signal", False):
                self.state = RobotState.BOUNDARY_DISCOVERY
                self.boundary_start_time = current_time
                self.start_time = current_time
                print("[FSM] Starting boundary discovery")

        elif self.state == RobotState.BOUNDARY_DISCOVERY:
            # Check timeout
            if current_time - self.boundary_start_time > self.boundary_timeout:
                print("[FSM] Boundary discovery timeout")
                self.state = RobotState.DONE
                return self.state

            # Check if rectangle is confident
            if context.get("rectangle_confident", False):
                self.rectangle_confident = True
                self.state = RobotState.COVERAGE
                self.coverage_start_time = current_time
                print("[FSM] Rectangle confident, starting coverage")

        elif self.state == RobotState.COVERAGE:
            # Check timeout
            if current_time - self.coverage_start_time > self.coverage_timeout:
                print("[FSM] Coverage timeout")
                self.state = RobotState.DONE
                return self.state

            # Check if coverage is complete
            coverage_ratio = context.get("coverage_ratio", 0.0)
            coverage_planner_complete = context.get("coverage_planner_complete", False)
            
            # Complete if coverage ratio is high OR all lanes are traversed
            if coverage_ratio >= 0.97 or coverage_planner_complete:
                self.coverage_complete = True
                self.state = RobotState.DONE
                if coverage_planner_complete:
                    print(f"[FSM] Coverage complete - all lanes traversed ({coverage_ratio:.1%} coverage ratio)")
                else:
                    print(f"[FSM] Coverage complete ({coverage_ratio:.1%})")

        elif self.state == RobotState.DONE:
            # Stay in DONE state
            pass

        elif self.state == RobotState.ERROR:
            # Stay in ERROR state
            pass

        return self.state

    def get_state(self) -> RobotState:
        """Get current state."""
        return self.state

    def is_active(self) -> bool:
        """Check if robot should be actively moving."""
        return self.state in [RobotState.BOUNDARY_DISCOVERY, RobotState.COVERAGE]

    def should_run_boundary(self) -> bool:
        """Check if robot should run boundary discovery."""
        return self.state == RobotState.BOUNDARY_DISCOVERY

    def should_run_coverage(self) -> bool:
        """Check if robot should run coverage."""
        return self.state == RobotState.COVERAGE

    def is_done(self) -> bool:
        """Check if robot is done."""
        return self.state == RobotState.DONE

    def is_error(self) -> bool:
        """Check if robot is in error state."""
        return self.state == RobotState.ERROR

    def reset(self):
        """Reset FSM to initial state."""
        self.state = RobotState.WAIT_START
        self.start_time = None
        self.boundary_start_time = None
        self.coverage_start_time = None
        self.rectangle_confident = False
        self.coverage_complete = False

    def get_elapsed_time(self) -> float:
        """Get elapsed time since start."""
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time
