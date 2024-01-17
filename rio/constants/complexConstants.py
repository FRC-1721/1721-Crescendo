# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""


import math

from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from rev import CANSparkMax

from constants.getConstants import getConstants

constants = getConstants("simple_hardware")
neoConsts = constants["neo"]
driveConsts = constants["drive"]
moduleConsts = constants["module"]


class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    kMaxAngularSpeed = math.tau  # radians per second

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(driveConsts["kWheelBase"] / 2, driveConsts["kTrackWidth"] / 2),
        Translation2d(driveConsts["kWheelBase"] / 2, -driveConsts["kTrackWidth"] / 2),
        Translation2d(-driveConsts["kWheelBase"] / 2, driveConsts["kTrackWidth"] / 2),
        Translation2d(-driveConsts["kWheelBase"] / 2, -driveConsts["kTrackWidth"] / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # Angular offsets of the modules relative to the chassis in radians
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = math.pi
    kBackRightChassisAngularOffset = math.pi / 2


class ModuleConstants:
    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = neoConsts["kFreeSpeedRpm"] / 60

    kWheelCircumferenceMeters = moduleConsts["kWheelDiameterMeters"] * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (60 * 34) / (moduleConsts["kDrivingMotorPinionTeeth"] * 15)
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        moduleConsts["kWheelDiameterMeters"] * math.pi
    ) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = (
        (moduleConsts["kWheelDiameterMeters"] * math.pi) / kDrivingMotorReduction
    ) / 60.0  # meters per second

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    kDrivingFF = 1 / kDriveWheelFreeSpeedRps

    kDrivingMotorIdleMode = CANSparkMax.IdleMode.kBrake
    kTurningMotorIdleMode = CANSparkMax.IdleMode.kBrake


class AutoConstants:
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )
