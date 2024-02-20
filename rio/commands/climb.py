import commands2

from constants import ClimberConstants


class Climb(commands2.Command):
    def __init__(self, speed, subsystem):
        """
        rotates the climber
        """
        super().__init__()

        # local instance of subsystem
        self.climberSubsystem = subsystem

        # local var of speed
        self.speed = speed

    def execute(self):
        self.climberSubsystem.setClimberMotorSpeed(self.speed)

    def end(self, interrupted: bool):
        self.climberSubsystem.setClimberMotorSpeed(0)
