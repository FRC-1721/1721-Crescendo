import commands2

from ntcore import NetworkTableInstance
from rev import CANSparkMax, CANSparkLowLevel
from wpilib import Servo
from constants import ClimberConstants


class Climber(commands2.Subsystem):
    def __init__(self) -> None:
        """
        creates the climber subsystem
        """
        super().__init__()

        # dashboard
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # climbing motor
        self.motor = CANSparkMax(
            ClimberConstants.kmotorID, CANSparkLowLevel.MotorType.kBrushless
        )
        self.servo = Servo(ClimberConstants.kServoID)

        # encoder
        self.encoder = self.motor.getEncoder()

        # inversion
        self.motor.setInverted(ClimberConstants.kInversion)

    def periodic(self) -> None:
        self.sd.putNumber("Thermals/Climber/Winch", self.motor.getMotorTemperature())

    def setClimberMotorSpeed(self, speed):
        self.motor.set(speed)

    def setServoAngle(self, angle):
        self.servo.set(angle)
