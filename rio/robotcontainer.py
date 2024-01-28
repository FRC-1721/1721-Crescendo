import math

# vendor libs
import wpilib

import wpimath

from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

import commands2

from commands2 import cmd
from commands2.button import CommandJoystick, CommandXboxController

from constants import AutoConstants, DriveConstants, OIConstants

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intake import IntakeSubsystem

from commands.intakeSuck import IntakeSuck
from commands.intakeRotationPID import IntakeRotationPID


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        self.intake = IntakeSubsystem()

        # The driver's controller
        self.driverController = CommandJoystick(OIConstants.kDriverControllerPort)

        # the operators controller
        self.opController = CommandXboxController(OIConstants.kOpControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.cmd.run(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        self.driverController.getRawAxis(1),
                        OIConstants.kDriveDeadband,  # TODO: Use constants to set these controls
                    )
                    * 0.3,
                    -wpimath.applyDeadband(
                        self.driverController.getRawAxis(0),
                        OIConstants.kDriveDeadband,  # TODO: Use constants to set these controls
                    )
                    * 0.3,
                    -wpimath.applyDeadband(
                        self.driverController.getRawAxis(2),
                        OIConstants.kDriveDeadband,  # TODO: Use constants to set these controls
                    )
                    * 0.3,
                    False,
                    False,
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        self.opController.x().onTrue(
            commands2.cmd.run(lambda: IntakeSuck(0.4), self.intake)
        )

        self.opController.a().onTrue(
            commands2.cmd.run(lambda: IntakeRotationPID(25), self.intake)
        )

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

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
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        holoController = HolonomicDriveController(
            PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
        )

        swerveControllerCommand = commands2.SwerveControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controller
            holoController,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )
