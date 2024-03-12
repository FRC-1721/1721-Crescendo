# Other libs
import math
import wpilib
import logging
import os
import importlib

# wpimath
import wpimath
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

# CommandsV2
import commands2
from commands2 import cmd
from commands2.button import CommandXboxController, CommandGenericHID

from constants import AutoConstants, DriveConstants, OIConstants, SuperStrucConstants

# Subsystems
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.climbersubsystem import ClimberSubsystem
from subsystems.shooter import Shooter
from subsystems.intake import IntakeSubsystem
from subsystems.climber import Climber
from subsystems.limelight import limeLightCommands

# Commands
from commands.setIntakeSpeed import SetIntakeSpeed
from commands.rotateIntake import RotateIntake
from commands.FlyWheelSpeed import FlyWheelSpeed
from commands.intakeRotationMAN import IntakeRotationMAN
from commands.SendToPos import sendToFieldPos
from commands.NavToObj import sendToObject
from commands.shooterROT import ShooterROT
from commands.manualRot import manualROT
from commands.intakeUntilNote import intakeUntilNote
from commands.setIntakeSpeed import SetIntakeSpeed
from commands.loadMagazine import LoadMagazine
from commands.climb import Climb
from commands.resetYaw import ResetYaw
from commands.spool import Spool
from commands.lock import Lock

# NetworkTables
from ntcore import NetworkTableInstance

# Misc
from utils.deploydata import getDeployData


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
        self.shooter = Shooter()
        self.intake = IntakeSubsystem()
        self.climber = Climber()
        self.limelight = limeLightCommands()

        # The driver's controller
        self.driverController = CommandXboxController(0)

        # the operators controller
        self.opController = CommandXboxController(OIConstants.kOpControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure networktables
        self.configureNetworktables()

        # Configure autonomous
        self.configureAutonomous()

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
                    * 0.5,
                    -wpimath.applyDeadband(
                        self.driverController.getRawAxis(0),
                        OIConstants.kDriveDeadband,  # TODO: Use constants to set these controls
                    )
                    * 0.5,
                    -wpimath.applyDeadband(
                        self.driverController.getRawAxis(4),
                        OIConstants.kDriveDeadband,  # TODO: Use constants to set these controls
                    )
                    * 0.5,
                    False,
                    True,
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

        # ==============================
        #        Driver Commands
        # ==============================

        # Drop intake (controller x)
        self.driverController.x().onTrue(
            commands2.SequentialCommandGroup(
                ShooterROT(SuperStrucConstants.LoadPos, self.shooter),
                FlyWheelSpeed(0, self.shooter),  # Stop shooter (if its running)
                RotateIntake(120, self.intake),  # Put intake down
                intakeUntilNote(0.5, self.intake),  # Intake till note
                RotateIntake(0, self.intake),  # Put intake back up
            )
        )

        # Shoot to speaker (button y)
        self.driverController.y().onTrue(
            commands2.SequentialCommandGroup(
                RotateIntake(
                    0, self.intake
                ),  # Put intake fully inside (if it wasn't already)
                FlyWheelSpeed(1.0, self.shooter, False),  # Power up the flywheels (?)
                SetIntakeSpeed(
                    -0.6, self.intake
                ),  # Load magazine? (but without ending)
                commands2.WaitCommand(3),
                FlyWheelSpeed(0.0, self.shooter),  # Stop flywheel
                SetIntakeSpeed(0, self.intake),  # Stop intake
            )
        )

        # Deliver to amp (button a), part a
        self.driverController.a().onTrue(
            commands2.SequentialCommandGroup(
                RotateIntake(0, self.intake),  # Rotate to fully closed
                # SetIntakeSpeed(-0.6, self.intake),  # Eject slowly
                LoadMagazine(self.shooter, self.intake),  # Load the magazine
                # SetIntakeSpeed(0, self.intake),  # Stop ejecting
                ShooterROT(
                    SuperStrucConstants.ShootPos, self.shooter
                ),  # Rotate the shooter
                # power flywheels
            )
        )

        # Deliver to amp (button b), part b
        self.driverController.b().onTrue(
            commands2.SequentialCommandGroup(
                FlyWheelSpeed(0.25, self.shooter, False),  # rotates the Flywheel
                commands2.WaitCommand(2),
                ShooterROT(
                    SuperStrucConstants.LoadPos, self.shooter
                ),  # Rotate the shooter
                FlyWheelSpeed(0, self.shooter),  # stops the Flywheel
            )
        )
        self.driverController.start().onTrue(ResetYaw(self.robotDrive))

        self.driverController.back().onTrue(
            commands2.InstantCommand(self.intake.zeroIntake)
        )

        # Climbing
        self.driverController.rightBumper().whileTrue(
            Climb(
                lambda: self.opController.getRightTriggerAxis()
                - self.opController.getLeftTriggerAxis(),
                self.climber,
                self.shooter,
            )
        )

        self.driverController.leftBumper().whileTrue(
            sendToObject(self.robotDrive, self.limelight)
        )

        # ==============================
        #        Operator Commands
        # ==============================
        # intake keybinds
        # intake movement
        self.opController.button(2).whileTrue(IntakeRotationMAN(1, self.intake))  # out
        self.opController.button(1).whileTrue(IntakeRotationMAN(-1, self.intake))  # in

        # intake spin
        self.opController.button(6).whileTrue(SetIntakeSpeed(0.6, self.intake))
        self.opController.button(9).whileTrue(SetIntakeSpeed(-0.6, self.intake))

        # shooter keybinds
        # shooter movement
        self.opController.button(3).whileTrue(manualROT(0.5, self.shooter))
        self.opController.button(4).whileTrue(manualROT(-0.5, self.shooter))

        # climber
        self.opController.button(5).whileTrue(Climb(0.2, self.climber, self.shooter))
        self.opController.button(7).whileTrue(Spool(-0.1, self.climber))

        # Cancel all
        self.opController.button(8).onTrue(commands2.InstantCommand(self.cancelAll))

    def cancelAll(self) -> None:
        """
        Joe wrote this so the operator could stop everything!
        """

        logging.warning("CANCELING ALL COMMANDS!")
        commands2.CommandScheduler.getInstance().cancelAll()

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def configureNetworktables(self):
        """
        Configure any networktable defaults
        """

        # Configure networktables
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # Subtables
        self.build_table = self.sd.getSubTable("BuildData")

        # Build data (May need to be moved to a dedicated function to be updated more than once)
        data = getDeployData()
        for key in data:
            key_entry = self.build_table.getEntry(str(key))
            key_entry.setString(str(data[key]))

        # Use sendable choosers for some settings
        self.fieldCentricChooser = wpilib.SendableChooser()
        self.fieldCentricChooser.setDefaultOption("Field Centric", True)
        self.fieldCentricChooser.addOption("Robot Centric", False)
        wpilib.SmartDashboard.putData("FieldCentric", self.fieldCentricChooser)

    def configureAutonomous(self) -> None:
        """
        Configure Autonomous Modes
        """

        self.autoChooser = wpilib.SendableChooser()

        from autonomous.noAuto import NoAuto

        self.autoChooser.setDefaultOption("No Auto", NoAuto(self.robotDrive))

        list(
            map(
                lambda x: self.autoChooser.addOption(x.NAME, x.load(self)),
                map(
                    lambda mod: importlib.import_module(f"autonomous.{mod[:-3]}"),
                    os.listdir("./autonomous"),
                ),
            )
        )

        wpilib.SmartDashboard.putData("Autonomous", self.autoChooser)

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns:
        command to run in autonomous
        """
        # # Create a sendable chooser
        # self.autoChooser = wpilib.SendableChooser()

        # # Add options
        # self.autoChooser.setDefaultOption("No Auto", NoAuto())

        # # Put the chooser on the dashboard
        # wpilib.SmartDashboard.putData("Autonomous", self.autoChooser)

        # ===========================
        # DEFAULT STUFF
        # ===========================

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
                lambda: self.robotDrive.drive(0, 0, 0, True, False),
                self.robotDrive,
            )
        )
