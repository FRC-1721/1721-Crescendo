import wpilib

from ntcore import NetworkTableInstance
from commands2 import Subsystem
from rev import (
    CANSparkLowLevel,
    CANSparkMax,
    SparkMaxAbsoluteEncoder,
    SparkMaxLimitSwitch,
)

from constants import SuperStrucConstants


class Superstructure(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # dashboard
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # motors
        self.rotateMotor = CANSparkMax(
            SuperStrucConstants.rotateID, CANSparkLowLevel.MotorType.kBrushless
        )
        self.flyMotor = CANSparkMax(
            SuperStrucConstants.flyID, CANSparkLowLevel.MotorType.kBrushless
        )
        self.guideMotor = CANSparkMax(
            SuperStrucConstants.guideID, CANSparkLowLevel.MotorType.kBrushless
        )

        # encoders
        self.flyEncoder = self.flyMotor.getEncoder()
        self.guideEncoder = self.guideMotor.getEncoder()
        self.rotateEncoder = self.rotateMotor.getAbsoluteEncoder(
            SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )

        # PID values
        self.rotatePIDController = self.rotateMotor.getPIDController()

        self.rotatePIDController.getP(SuperStrucConstants.kP)
        self.rotatePIDController.getI(SuperStrucConstants.kI)
        self.rotatePIDController.getD(SuperStrucConstants.kD)
        self.rotatePIDController.getFF(SuperStrucConstants.kFF)

        # limit switches
        self.limit = self.guideMotor.getForwardLimitSwitch(
            SparkMaxLimitSwitch.Type.kNormallyOpen
        )

        self.limit.enableLimitSwitch(True)

        # current limits
        # TODO change these
        self.rotateMotor.setSmartCurrentLimit(SuperStrucConstants.rotateCurrentLimit)
        self.flyMotor.setSmartCurrentLimit(SuperStrucConstants.flyCurrentLimit)
        self.guideMotor.setSmartCurrentLimit(SuperStrucConstants.guideCurrentLimit)

    def periodic(self) -> None:
        self.sd.putNumber("Thermals/rotate", self.rotateMotor.getMotorTemperature())
        self.sd.putNumber("Thermals/fly", self.flyMotor.getMotorTemperature())
        self.sd.putNumber("Thermals/guide", self.guideMotor.getMotorTemperature())

    def fire(self, speed):
        self.flyMotor.set(speed)

    def gotoPOS(self, angle: float):
        self.rotatePIDController.setReference(angle, CANSparkMax.ControlType.kPosition)

    def guiding(self):
        if self.limit.get():
            self.guideMotor.set(0)
