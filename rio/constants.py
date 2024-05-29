# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""

import math

from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from rev import CANSparkMax

from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    ReplanningConfig,
    PIDConstants,
)


class NeoMotorConstants:
    kFreeSpeedRpm = 5676


class DriveConstants:

    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = math.tau  # radians per second

    kDirectionSlewRate = 1.2  # radians per second
    kMagnitudeSlewRate = 1.8  # percent per second (1 = 100%)
    kRotationalSlewRate = 2.0  # percent per second (1 = 100%)

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(20.3937)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(20.5)
    kDriveBaseRadius = math.sqrt(kTrackWidth**2 + kWheelBase**2)

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # Angular offsets of the modules relative to the chassis in radians
    kFrontLeftChassisAngularOffset = 0
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = 0
    kBackRightChassisAngularOffset = 0

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 5
    kRearLeftDrivingCanId = 7
    kFrontRightDrivingCanId = 1
    kRearRightDrivingCanId = 3

    kFrontLeftDriveInversion = True
    kRearLeftDriveInversion = True
    kFrontRightDriveInversion = False
    kRearRightDriveInversion = False

    kGyroReversed = True

    HolonomicConfig = HolonomicPathFollowerConfig(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
        PIDConstants(5.0, 0.0, 0.0),  # Translation PID constants
        PIDConstants(5.0, 0.0, 0.0),  # Rotation PID constants
        kMaxSpeedMetersPerSecond,  # Max module speed, in m/s
        kDriveBaseRadius,  # Drive base radius in meters. Distance from robot center to furthest module.
        ReplanningConfig(),  # Default path replanning config. See the API for the options here
    )


class ModuleConstants:
    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    kDrivingMotorPinionTeeth = 17

    # Invert the turning encoder, since the output shaft rotates in the opposite direction of
    # the steering motor in the MAXSwerve Module.
    kTurningEncoderInverted = True

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 0.09525
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (60 * 34) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = (
        (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    ) / 60.0  # meters per second

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1
    kTurningI = 0
    kTurningD = 0.1
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = CANSparkMax.IdleMode.kBrake
    kTurningMotorIdleMode = CANSparkMax.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50  # amp
    kTurningMotorCurrentLimit = 20  # amp


class OIConstants:
    # driver controller
    kDriverControllerPort = 0
    kDriveDeadband = 0.075
    kDampeningAmount = 0.15

    # operator controller
    kOpControllerPort = 1
    kIntakeButton = 1


class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )


class SuperStrucConstants:
    # angles for shooter
    ShootPos = 321
    LoadPos = 204
    ClimbPos = 285
    HarmonyPos = 316

    # CANSpark IDS
    rotateID = 11
    flyID = 12
    guideID = 13

    kRotConversion = 360  # Configured feb 12 by joe

    krotateInversion = True

    # PID values
    krotateP = 0.01
    krotateI = 0.0
    krotateD = 0.0
    krotateFF = 0

    kflyP = 1
    kflyI = 0.00001
    kflyD = 0.0002
    kflyFF = 0


class IntakeConstants:
    # CANSparkMax ports
    kliftCanId = 9
    kIntakeCanId = 10

    # inversion
    kLiftInversion = False
    kIntakeInversion = True

    # conversion factor
    kLiftConversion = 1  # Configured feb 12 by joe
    SuckPos = 0.645
    BlowPos = 0.1
    # lift pid
    kLiftP = 3.3
    kLiftI = 0.0000001
    kLiftD = 0.003
    kLiftFF = 0


class GyroConstants:
    id = 0

    # pose
    yawPose = 0
    pitchPose = 0
    rollPose = 0


class ClimberConstants:
    # ids
    kmotorID = 14
    kServoID = 0

    # inversions
    kInversion = True

    # servo angles
    kServoLock = -0.1
    kservoOpen = 0.165

    # Relationship between climber speed and shooter angle (rough but ugh..)
    kClimberShooterForward = 0.4  # Down
    kClimberShooterBackward = 0.75  # Up
