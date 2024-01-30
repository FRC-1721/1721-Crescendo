import commands2

from subsystems.superstructure import Superstructure


class Fire(commands2.Command):
    def __init__(self, speed, subsystem):
        super().__init__()

        # local subsystem instance
        self.subsystem = subsystem

        # requested speed
        self.speed = speed

        # TODO change current limit later in amps
        self.subsystem.intakeCurrentLimit(1)

    def initialize(self):
        self.subsystem.fire(self.speed)

    def end(self, interrupted: bool):
        self.subsystem.fire(0)
        return True
