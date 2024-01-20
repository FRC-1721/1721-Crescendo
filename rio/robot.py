#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# wpilib
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller

# drivetrain
from components.drivetrain import Drivetrain


class UnnamedToaster(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.Joystick(0)
        self.drivetrain = Drivetrain()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

    def disabledPeriodic(self) -> None:
        """
        Runs when the robot is in DISABLED mode.
        """

        # Run periodic tasks
        self.drivetrain.periodic()

    def autonomousPeriodic(self) -> None:
        """
        Runs when the robot is in AUTONOMOUS mode.
        """

        self.driveWithJoystick(False)

        # Run periodic tasks
        self.drivetrain.periodic()

    def teleopPeriodic(self) -> None:
        """
        Runs when the robot is in TELEOP mode.
        """

        self.driveWithJoystick(True)

        # Run periodic tasks
        self.drivetrain.periodic()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = -self.xspeedLimiter.calculate(
            wpimath.applyDeadband(
                self.controller.getRawAxis(0), 0.02
            )  # TODO: What axis?! Make a controls.yaml!
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = -self.yspeedLimiter.calculate(
            wpimath.applyDeadband(
                self.controller.getRawAxis(1), 0.02
            )  # TODO: What axis?! Make a controls.yaml!
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = -self.rotLimiter.calculate(
            wpimath.applyDeadband(
                self.controller.getRawAxis(2), 0.02
            )  # TODO: What axis?! Make a controls.yaml!
        )

        self.drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
