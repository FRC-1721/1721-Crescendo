# rename to drivesubsystem.py when completed

from commands2 import Subsystem
from constants.constants import getConstants
from .swerveModule import SwerveModule


class drivesubsystem(Subsystem):
    def __init__(self) -> None:
        # constants
        constants = getConstants("robot_hardware")
        self.driveConst = constants["drivetrain"]

        # modules
        self.fPMotors = self.driveConst["FPMotors"]
        self.fSMotors = self.driveConst["FSMotors"]
        self.aPMotors = self.driveConst["APMotors"]
        self.aSMotors = self.driveConst["ASMotors"]

        # create swerve modules
        self.fpModule = SwerveModule(
            self.fPMotors["drivePort"],
            self.fPMotors["turnPort"],
            self.fPMotors["EncoderPort"][0],
            self.fPMotors["EncoderPort"][1],
        )
        self.fsModule = SwerveModule(
            self.fSMotors["drivePort"],
            self.fSMotors["turnPort"],
            self.fSMotors["EncoderPort"][0],
            self.fSMotors["EncoderPort"][1],
        )
        self.apModule = SwerveModule(
            self.aPMotors["drivePort"],
            self.aPMotors["turnPort"],
            self.aPMotors["EncoderPort"][0],
            self.aPMotors["EncoderPort"][1],
        )
        self.asModule = SwerveModule(
            self.aSMotors["drivePort"],
            self.aSMotors["turnPort"],
            self.aSMotors["EncoderPort"][0],
            self.aSMotors["EncoderPort"][1],
        )

        self.fsZero.setBoolean(self.fsModule.isZeroed)
        self.asZero.setBoolean(self.asModule.isZeroed)
        self.fpZero.setBoolean(self.fpModule.isZeroed)
        self.apZero.setBoolean(self.apModule.isZeroed)
