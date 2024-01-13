"""
this is hear to make the files cleaner
and to do alot of needed math in its own file
"""

# wpilib
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

# vendor libs
from rev import CANSparkMax

# constants
from constants.constants import getConstants

# misc
import math


class DriveConstants:
    # constants
    constants = getConstants("robot_hardware")
    drivetrain = constants["driveTrain"]

    # allowed max speed
    maxAngularSpeed = math.tau

    # Chassis config
    trackWidth = drivetrain["TrackWidth"]
    # Distance between centers of right and left wheels
    wheelBase = drivetrain["WheelBase"]

    # Distance of front and back wheels
    modulePositions = [
        Translation2d(wheelBase / 2, trackWidth / 2),
        Translation2d(wheelBase / 2, -trackWidth / 2),
        Translation2d(-wheelBase / 2, trackWidth / 2),
        Translation2d(-wheelBase / 2, -trackWidth / 2),
    ]
    driveKinematics = SwerveDrive4Kinematics(*modulePositions)

    # Angular offsets of modules relative to chassis in radians
    fPChassisAngularOffset = -math.pi / 2
    fSChassisAngularOffset = 0
    aPChassisAngularOffset = math.pi
    aSChassisAngularOffset = math.pi / 2


class ModuleConstants:
    # constants
    # hardware
    hardwareConstants = getConstants("robot_hardware")
    drivetrain = hardwareConstants["driveTrain"]
    NeoMotor = hardwareConstants["NeoMotor"]

    # pids
    pid = getConstants("robot_pid")["PID"]

    driveMotorPinionTeeth = drivetrain["driveMotorPinionTeeth"]

    # Calculations for drive motor conversion factors and feed forward
    driveMotorFreeSpeedRps = NeoMotor["FreeSpeedRpm"] / 60
    WheelDiameterMeters = drivetrain["WheelDiameterMeters"]
    WheelCircumferenceMeters = WheelDiameterMeters * math.pi

    driveMotorReduction = (
        drivetrain["BevelGearTeeth"] * drivetrain["firstStageSpurTeeth"]
    ) / (driveMotorPinionTeeth * drivetrain["bevelPinion"])

    driveEncoderPositionFactor = (WheelDiameterMeters * math.pi) / driveMotorReduction
    driveEncoderVelocityFactor = (
        (WheelDiameterMeters * math.pi) / driveMotorReduction
    ) / 60.0

    turnEncoderPositionFactor = math.tau
    turnEncoderVelocityFactor = math.tau / 60.0

    turnEncoderPositionPIDMinInput = pid["turnEncoderPositionPIDMinInput"]  # radian
    turnEncoderPositionPIDMaxInput = turnEncoderPositionFactor

    driveMotorIdleMode = CANSparkMax.IdleMode.kBrake
    turnMotorIdleMode = CANSparkMax.IdleMode.kBrake


class AutoConstants:
    maxAngularSpeedRadiansPerSecond = math.pi
    maxAngularSpeedRadiansPerSecondSquared = math.pi

    # Constraint for  motion robot angle controller
    thetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared
    )
