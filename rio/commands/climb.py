import commands2
import rev

from constants import ClimberConstants, SuperStrucConstants

from subsystems.climber import Climber
from subsystems.shooter import Shooter
import logging


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
        self.time = 0

        # Command requires
        self.addRequirements((self.climber, self.shooter))

        self.climber.setServoAngle(ClimberConstants.kservoOpen)

        self.climber.setIdleMode(rev._rev.CANSparkBase.IdleMode.kBrake)
        self.shooter.setIdleMode(rev._rev.CANSparkBase.IdleMode.kCoast)

    def execute(self):
        angle = self.shooter.getAngle()
        softLimit = SuperStrucConstants.LoadPos + 35
        self.shooter.rotateManual(0)

        if angle >= softLimit:
            logging.info(f"Climbing... {int(angle)}/{softLimit}")
            self.climber.setClimberMotorSpeed(self.speed)
        else:
            logging.warn(f"You've gone too far! {int(angle):}/{softLimit}")
            self.climber.setClimberMotorSpeed(0)

    def end(self, interrupted: bool):
        self.shooter.setIdleMode(rev._rev.CANSparkBase.IdleMode.kCoast)
        self.climber.setIdleMode(rev._rev.CANSparkBase.IdleMode.kCoast)

        self.climber.setServoAngle(ClimberConstants.kServoLock)
        self.climber.setClimberMotorSpeed(0)
