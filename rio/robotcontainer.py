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
        # Create config for trajectory
        config = TrajectoryConfig(
            self.autoConst["maxSpeedMetersPerSecond"],
            self.autoConst["maxAccelerationMetersPerSecondSquared"],
        )
        # ensure max speed is obeyed
        config.setKinematics(DriveConstants.driveKinematics)

        # An example trajectory to follow. All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(1, 1), Translation2d(2, -1)],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(3, 0, Rotation2d(0)),
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            self.autoConst["pThetaController"],
            0,
            0,
            AutoConstants.thetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        swerveControllerCommand = SwerveControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose(),
            DriveConstants.driveKinematics,
            PIDController(self.autoConst["pXController"], 0, 0),
            PIDController(self.autoConst["pYController"], 0, 0),
            thetaController,
            self.robotDrive.setModuleStates(),
            (self.robotDrive,),
        )

        # Reset odometry to starting pose
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )
