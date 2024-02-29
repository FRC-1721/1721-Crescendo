import commands2

from constants import ClimberConstants

from subsystems.climber import Climber
from subsystems.shooter import Shooter


class Climb(commands2.Command):
    def __init__(self, speed, _climber: Climber, _shooter: Shooter):
        """
        rotates the climber
        """
        super().__init__()

        # local instance of subsystem
        self.climber = _climber
        self.shooter = _shooter

        # local var of speed
        self.speed = speed

        # Command requires
        self.addRequirements((self.climber, self.shooter))

    def execute(self):
        if self.speed >= 0:
            # Positive numbers imply we're pulling DOWN
            self.climber.setServoAngle(ClimberConstants.kServoLock)
        else:
            # Negative numbers imply we're relaxing
            self.climber.setServoAngle(ClimberConstants.kservoOpen)

        self.climber.setClimberMotorSpeed(self.speed)

    def end(self, interrupted: bool):
        self.climber.setClimberMotorSpeed(0)
