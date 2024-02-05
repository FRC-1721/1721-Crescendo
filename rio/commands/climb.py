import commands2


class Climb(commands2.Command):
    def __init__(self, subsystem, speed):
        """
        rotates the climber
        """
        super().__init__()

        # local instance of subsystem
        self.climberSubsystem = subsystem

        # speed
        self.speed = speed

    def initialize(self):
        # this is to stop it from seizing when going down
        self.climberSubsystem.setClimberMotorSpeed(0.1)

        # the rest of the command
        commands2.waitcommand(0.5)
        self.climberSubsystem.setClimberMotorSpeed(self.speed)

    def end(self, interrupted: bool):
        self.climberSubsystem.setClimberMotorSpeed(0)
