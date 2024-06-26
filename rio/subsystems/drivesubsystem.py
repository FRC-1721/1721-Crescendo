import math
import typing

import wpilib
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    ReplanningConfig,
    PIDConstants,
)

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

from phoenix5.sensors import Pigeon2

from ntcore import NetworkTableInstance

from constants import DriveConstants, GyroConstants, OIConstants

import utils.swerveutils as swerveutils

from utils.dummygyro import DummyGyro

from .mikeswervemodule import MikeSwerveModule


class DriveSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Create MAXSwerveModules
        self.frontLeft = MikeSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset,
            DriveConstants.kFrontLeftDriveInversion,
        )

        self.frontRight = MikeSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset,
            DriveConstants.kFrontRightDriveInversion,
        )

        self.rearLeft = MikeSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset,
            DriveConstants.kRearLeftDriveInversion,
        )

        self.rearRight = MikeSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset,
            DriveConstants.kRearRightDriveInversion,
        )

        # Configure networktables
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # The gyro sensor
        if wpilib.RobotBase.isReal():
            self.gyro = Pigeon2(GyroConstants.id)
        else:
            # Bug with navx init! For sim/unit testing just use the ADIS
            self.gyro = DummyGyro()

        # the mounting pose for the gyro
        self.gyro.configMountPose(
            GyroConstants.yawPose,
            GyroConstants.pitchPose,
            GyroConstants.rollPose,
        )
        # Slew rate filter variables for controlling lateral acceleration
        self.currentRotation = 0.0
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(self.gyro.getYaw()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
        )
        self.gyroAngle = self.gyro.getYaw()

        AutoBuilder.configureHolonomic(
            self.odometry.getPose,
            self.resetOdometry,
            self.getChassisSpeedsFromSwerveOdometry,
            self.speedsDrive,
            DriveConstants.HolonomicConfig,
            self.shouldFlipPath,
            self,
        )

    def periodic(self) -> None:
        # Update the odometry in the periodic block
        # print(self.getHeading())
        # print(self.gyro.getBiasedAccelerometer())
        self.odometry.update(
            Rotation2d.fromDegrees(self.gyroAngle),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
        )

        # Desired Positions
        self.sd.putNumber(
            "Swerve/FL/desired",
            self.frontLeft.desiredState.angle.degrees(),
        )
        self.sd.putNumber(
            "Swerve/FR/desired",
            self.frontRight.desiredState.angle.degrees(),
        )
        self.sd.putNumber(
            "Swerve/RL/desired",
            self.rearLeft.desiredState.angle.degrees(),
        )
        self.sd.putNumber(
            "Swerve/RR/desired",
            self.rearRight.desiredState.angle.degrees(),
        )

        # Actual Positions
        self.sd.putNumber(
            "Swerve/FL/actual",
            self.frontLeft.getState().angle.degrees(),
        )
        self.sd.putNumber(
            "Swerve/FR/actual",
            self.frontRight.getState().angle.degrees(),
        )
        self.sd.putNumber(
            "Swerve/RL/actual",
            self.rearLeft.getState().angle.degrees(),
        )
        self.sd.putNumber(
            "Swerve/RR/actual",
            self.rearRight.getState().angle.degrees(),
        )

        # Thermals
        self.sd.putNumber("Thermals/Swerve/FL/drive", self.frontLeft.getDriveTemp())
        self.sd.putNumber("Thermals/Swerve/FR/drive", self.frontRight.getDriveTemp())
        self.sd.putNumber("Thermals/Swerve/RL/drive", self.rearLeft.getDriveTemp())
        self.sd.putNumber("Thermals/Swerve/RR/drive", self.rearRight.getDriveTemp())
        self.sd.putNumber("Thermals/Swerve/FL/turn", self.frontLeft.getTurnTemp())
        self.sd.putNumber("Thermals/Swerve/FR/turn", self.frontRight.getTurnTemp())
        self.sd.putNumber("Thermals/Swerve/RL/turn", self.rearLeft.getTurnTemp())
        self.sd.putNumber("Thermals/Swerve/RR/turn", self.rearRight.getTurnTemp())

    def getChassisSpeedsFromSwerveOdometry(self) -> ChassisSpeeds:
        return DriveConstants.kDriveKinematics.toChassisSpeeds(self.getModuleStates())

    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def isPoseZero(self):
        Pose = self.odometry.getPose()
        for item in Pose:
            if item != 0:
                return False
        return True

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        self.odometry.resetPosition(
            Rotation2d.fromDegrees(self.gyro.getYaw()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
            pose,
        )

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        dampened: bool,
        fieldRelative: typing.Callable[
            [],
            bool,
        ],
        rateLimit: bool,
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the
                              field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        """
        xSpeedCommanded = xSpeed * (OIConstants.kDampeningAmount if dampened else 1)
        ySpeedCommanded = ySpeed * (OIConstants.kDampeningAmount if dampened else 1)
        self.sd.putNumber("pos/rot", rot)
        self.currentRotation = rot * (OIConstants.kDampeningAmount if dampened else 1)

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = self.currentRotation * DriveConstants.kMaxAngularSpeed

        self.speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(-self.gyro.getYaw()),
            )
            if fieldRelative
            else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        )

        self.speedsDrive(self.speeds)

    def speedsDrive(self, speeds) -> None:

        self.speeds = speeds

        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            self.speeds
        )
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.rearLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def getModuleStates(
        self,
    ) -> (SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState):

        # Messy... use a loop next time!
        return [
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.rearLeft.getState(),
            self.rearRight.getState(),
        ]

    def setModuleStates(
        self,
        desiredStates: typing.Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.rearLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.rearRight.resetEncoders()

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        # Pigeon2 doesn't have a reset command
        # self.gyro.reset()
        pass

    def getHeading(self) -> float:
        """Returns the heading of the robot.

        :returns: the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.gyro.getYaw()).degrees()

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if DriveConstants.kGyroReversed else 1.0)

    def resetYaw(self, angle) -> None:
        self.gyro.setYaw(angle, 100)

    def isZero(self, angle):
        yaw = self.gyro.getYaw()
        if yaw == angle:
            return True
        return False

    def getAcc(self):
        xAcc = self.gyro.getBiasedAccelerometer()[1][0]
        # print(xAcc)
        if xAcc < -2500:
            return True
        return False

    def shouldFlipPath(self):
        return self.nt.getTable("FMSinfo").getEntry("IsRedAlliance")
