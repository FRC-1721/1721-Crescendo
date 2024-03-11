# Other libs
import math
import wpilib
import logging

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
from commands2.button import CommandJoystick, CommandXboxController

from constants import AutoConstants, DriveConstants, OIConstants, SuperStrucConstants

# Subsystems
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.shooter import Shooter
from subsystems.intake import IntakeSubsystem

# Commands
from commands.setIntakeSpeed import SetIntakeSpeed
from commands.rotateIntake import RotateIntake
from commands.FlyWheelSpeed import FlyWheelSpeed
from commands.intakeRotationMAN import IntakeRotationMAN
from commands.shooterROT import ShooterROT
from commands.manualRot import manualROT
from commands.intakeUntilNote import intakeUntilNote
from commands.setIntakeSpeed import SetIntakeSpeed
from commands.loadMagazine import LoadMagazine

# autonomous
from autonomous.forwardDrive import ForwardDrive
from autonomous.ampDrive import AmpDrive

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

        # The driver's controller
        self.driverController = CommandXboxController(0)

        # the operators controller
        self.opController = CommandXboxController(OIConstants.kOpControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure networktables
        self.configureNetworktables()

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
                    * 0.6,
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

        # ==============================
        #        Operator Commands
        # ==============================

        # moving intake
        self.opController.pov(90).whileTrue(IntakeRotationMAN(1, self.intake))  # out
        self.opController.pov(270).whileTrue(IntakeRotationMAN(-1, self.intake))  # in

        # _____POST_INTAKE_KEYBINDS_____

        # moving shooter
        self.opController.pov(0).whileTrue(manualROT(0.5, self.shooter))
        self.opController.pov(180).whileTrue(manualROT(-0.5, self.shooter))

        self.driverController.pov(0).whileTrue(ShooterROT(60, self.shooter))

        # PID shooter rotation (NOT CURRENTLY WORKING)

        # self.opController.pov(0).whileTrue(ShooterROT(0,self.shooter))  # out
        # self.opController.pov(180).whileTrue(ShooterROT(40,self.shooter))  # out

        self.opController.b().whileTrue(RotateIntake(60, self.intake))

        # Cancel all when x is pressed
        self.opController.x().onTrue(commands2.InstantCommand(self.cancelAll))

        self.opController.back().onTrue(
            commands2.InstantCommand(self.intake.zeroIntake)
        )

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

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns:
        command to run in autonomous
        """

        # TODO replace with chooser
        return AmpDrive(self.robotDrive, self.shooter, self.intake)
