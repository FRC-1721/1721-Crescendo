#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpilib
import logging

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()
        self.autonomousCommand = None

        if not wpilib.RobotBase.isReal():
            # Do some things if the robot is NOT real

            # Ovverride default logging
            logging.basicConfig(level=logging.DEBUG)

    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def disabledInit(self) -> None:
        # Joe added this
        commands2.CommandScheduler.getInstance().cancelAll()
        logging.info("All commands canceled when entering disabled")

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
