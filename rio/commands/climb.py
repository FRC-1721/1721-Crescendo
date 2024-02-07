import commands2


class Climb(commands2.Command):
    def __init__(self, subsystem):
        """
        rotates the climber
        """
        super().__init__()

        # local instance of subsystem
        self.climberSubsystem = subsystem

    def initialize(self):
        pass

    def end(self, interrupted: bool):
        pass
