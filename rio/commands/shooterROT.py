import commands2

from subsystems.superstructure import Superstructure


class shooterROT(commands2.Command):
    def __init__(self, angle, subsystem):
        super().__init__()

        # local subsystem instance
        self.subsystem = subsystem

        # requested speed
        self.angle = angle

        # TODO change current limit later in amps
        self.subsystem.rotateCurrentLimit(1)

    def initialize(self):
        self.subsystem.gotoPOS(self.angle)

    def end(self, interrupted: bool):
        return True
