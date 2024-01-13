# commands
import commands2

from commands2 import cmd

# wpilib
import wpimath
import wpilib

from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

# subsystems
from subsystems.drivesubsystem import DriveSubsystem

# constants
from constants.mathConstant import AutoConstants, DriveConstants
from constants.constants import getConstants

# yes its the enitire ottoman empire
from ottomanEmpire.bursa.swervecontrollercommand import SwerveControllerCommand

# misc
import math


class RobotContainer:
    def __init__(self) -> None:
        # constants
        # drivetrain
        hardwareConstants = getConstants("robot_hardware")
        self.driveConst = hardwareConstants["driveTrain"]
        self.autoConst = self.driveConst["autonomous"]

        # controller
        controllerConst = getConstants("robot_controls")
        self.driverConst = controllerConst["driver"]

        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # The driver's controller
        self.driverController = wpilib.XboxController(
            self.driverConst["ControllerPort"]
        )

        # Config buttons
        self.configureButtonBindings()

        # default command
        self.robotDrive.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        self.driverController.getLeftY(), self.driverConst["DeadZone"]
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getLeftX(), self.driverConst["DeadZone"]
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), self.driverConst["DeadZone"]
                    ),
                    True,
                    True,
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self) -> None:
        pass

    def disablePIDSubsystems(self) -> None:
        """
        'Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup.'
        - rev
        """

    def getAutonomousCommand(self) -> commands2.Command:
        # rev has stuff here but I don't want to debug it
        pass
