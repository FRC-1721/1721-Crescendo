import commands2
import rev

from constants import ClimberConstants

from subsystems.climber import Climber
from subsystems.shooter import Shooter


class Climb(commands2.Command):
    def __init__(
        self,
        _speed,
        _climber: Climber,
        _shooter: Shooter,
    ):
        """
        rotates the climber
        """
        super().__init__()

        # local instance of subsystem
        self.climber = _climber
        self.shooter = _shooter

        # local var of speed
        self.speed = _speed

        # Command requires
        self.addRequirements((self.climber, self.shooter))

        self.shooter.setIdleMode(rev._rev.CANSparkBase.IdleMode.kCoast)
        self.climber.setServoAngle(ClimberConstants.kservoOpen)

    def execute(self):
        self.climber.setClimberMotorSpeed(self.speed)
        self.shooter.setIdleMode(rev._rev.CANSparkBase.IdleMode.kBrake)

    def end(self, interrupted: bool):
        self.climber.setServoAngle(ClimberConstants.kServoLock)
        self.climber.setClimberMotorSpeed(0)
