# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
from __future__ import annotations

from typing import Union

from .subsystem import Subsystem
from wpimath.trajectory import TrapezoidProfile


class TrapezoidProfileSubsystem(Subsystem):
    """
    A subsystem that generates and runs trapezoidal motion profiles automatically. The user specifies
    how to use the current state of the motion profile by overriding the `useState` method.
    """

    def __init__(
        self,
        constraints: TrapezoidProfile.Constraints,
        initial_position: float = 0.0,
        period: float = 0.02,
    ):
        """
        Creates a new TrapezoidProfileSubsystem.

        :param constraints: The constraints (maximum velocity and acceleration) for the profiles.
        :param initial_position: The initial position of the controlled mechanism when the subsystem is constructed.
        :param period: The period of the main robot loop, in seconds.
        """
        self._profile = TrapezoidProfile(constraints)
        self._state = TrapezoidProfile.State(initial_position, 0)
        self.setGoal(initial_position)
        self._period = period
        self._enabled = True

    def periodic(self):
        """
        Executes the TrapezoidProfileSubsystem logic during each periodic update.

        This method is called synchronously from the subsystem's periodic() method.
        """
        self._state = self._profile.calculate(self._period, self._goal, self._state)
        if self._enabled:
            self.useState(self._state)

    def setGoal(self, goal: Union[TrapezoidProfile.State, float]):
        """
        Sets the goal state for the subsystem. Goal velocity assumed to be zero.

        :param goal: The goal position for the subsystem's motion profile. The goal
        can either be a `TrapezoidProfile.State` or `float`. If float is provided,
        the assumed velocity for the goal will be 0.
        """
        # If we got a float, instantiate the state
        if isinstance(goal, (float, int)):
            goal = TrapezoidProfile.State(goal, 0)

        self._goal = goal

    def enable(self):
        """Enable the TrapezoidProfileSubsystem's output."""
        self._enabled = True

    def disable(self):
        """Disable the TrapezoidProfileSubsystem's output."""
        self._enabled = False

    def useState(self, state: TrapezoidProfile.State):
        """
        Users should override this to consume the current state of the motion profile.

        :param state: The current state of the motion profile.
        """
        raise NotImplementedError("Subclasses must implement this method")
