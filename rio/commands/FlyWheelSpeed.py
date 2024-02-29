import logging
import commands2

from subsystems.shooter import Shooter


class FlyWheelSpeed(commands2.Command):
    def __init__(self, speed, _shooter: Shooter, stopWhenComplete=True):
        super().__init__()

        # local subsystem instance
        self.shooter = _shooter

        # requested speed
        self.speed = speed

        # Whether to stop when done or not
        self.swc = stopWhenComplete

        self.addRequirements(self.shooter)

    def initialize(self):
        logging.info("Spinning up...")

    def execute(self):
        self.shooter.setFlyWheelSpeed(self.speed)
        logging.debug(f"Flywheel at speed {self.shooter.currentSpeed()}")

    def isFinished(self):
        return self.shooter.isReady() or self.speed < 0.9

    def end(self, interrupted: bool):
        if not interrupted:
            logging.info(f"Done, up to speed is {self.isFinished()}")
        else:
            logging.warn("FlyWheelSpeed was interrupted!")

        if self.swc:
            self.shooter.setFlyWheelSpeed(0)
        return True
