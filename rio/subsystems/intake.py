import commands2

from ntcore import NetworkTableInstance
from rev import CANSparkMax, CANSparkLowLevel
from constants import IntakeConstants


class IntakeSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        """
        creates a intake subsystem
        """
        super().__init__()

        # dashboard
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # lifting motor
        self.liftMotor = CANSparkMax(
            IntakeConstants.kliftCanId,
            CANSparkLowLevel.MotorType.kBrushless,
        )

        # intaking motor
        self.intakeMotor = CANSparkMax(
            IntakeConstants.kIntakeCanId,
            CANSparkLowLevel.MotorType.kBrushless,
        )

        # setting iverted
        self.liftMotor.setInverted(IntakeConstants.kLiftInversion)
        self.intakeMotor.setInverted(IntakeConstants.kIntakeInversion)

    def periodic(self) -> None:
        self.sd.putNumber("Thermals/Intake", self.intakeMotor.getMotorTemperature())
        self.sd.putNumber("Thermals/Lift", self.liftMotor.getMotorTemperature())

    def intake(self, speed):
        self.intakeMotor.set(speed)

    def lift(self, speed):
        self.liftMotor.set(speed)

    def liftCurrentLimit(self, current):
        self.liftMotor.setSmartCurrentLimit(current)

    def intakeCurrentLimit(self, current):
        self.intakeMotor.setSmartCurrentLimit(current)
