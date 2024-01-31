import commands2

from subsystems.superstructure import Superstructure


class Fire(commands2.Command):
    def __init__(self, subsystem):
        super().__init__()

        # local subsystem instance
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.setGuidingMotorSpeed()

    def end(self, interrupted: bool):
        self.subsystem.guiding(0)
        return True
