import wpilib

from ntcore import NetworkTableInstance
from commands2 import Subsystem
from rev import (
    CANSparkLowLevel,
    CANSparkMax,
    SparkAbsoluteEncoder,
    SparkMaxLimitSwitch,
)

from constants import SuperStrucConstants


class Shooter(Subsystem):
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

        # encoders
        self.flyEncoder = self.flyMotor.getEncoder()
        self.rotateEncoder = self.rotateMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

        # PID values
        self.rotatePIDController = self.rotateMotor.getPIDController()

        self.rotatePIDController.getP(SuperStrucConstants.kP)
        self.rotatePIDController.getI(SuperStrucConstants.kI)
        self.rotatePIDController.getD(SuperStrucConstants.kD)
        self.rotatePIDController.getFF(SuperStrucConstants.kFF)


    def periodic(self) -> None:
        self.sd.putNumber("Thermals/rotate", self.rotateMotor.getMotorTemperature())
        self.sd.putNumber("Thermals/fly", self.flyMotor.getMotorTemperature())
        # print(self.flyEncoder.getVelocity())

    def setFlyWheelSpeed(self, speed):
        self.flyMotor.set(speed)

    def rotateManual(self,speed):
        self.rotateMotor.set(speed)
        
    def setRotateAngle(self, angle: float):
        self.rotatePIDController.setReference(angle, CANSparkMax.ControlType.kPosition)

    def isReady(self):
        return self.flyEncoder.getVelocity() > 5500 # Magic