import commands2
import logging

from subsystems.shooter import Shooter


class ShooterROT(commands2.Command):
    def __init__(self, angle: float, _shooter=Shooter):
        super().__init__()

        # local subsystem instance
        self.subsystem = _shooter

        # requested speed
        self.angle = angle

        self.addRequirements(self.subsystem)

    def initialize(self):
        logging.info(f"Moving shooter to {self.angle}")

    def execute(self):
        self.subsystem.setRotateAngle(self.angle)

        logging.debug(f"Still moving... {self.subsystem.getAngle()} to {self.angle}")

    def end(self, interrupted: bool):
        if not interrupted:
            logging.info(f"Done shooter moved to {self.subsystem.getAngle()}")
        else:
            logging.warn("ShooterROT was interrupted!")
        return True
