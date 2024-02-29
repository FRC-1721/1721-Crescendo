import logging
import commands2


class FlyWheelSpeed(commands2.Command):
    def __init__(self, speed, subsystem):
        super().__init__()

        # local subsystem instance
        self.subsystem = subsystem

        # requested speed
        self.speed = speed

    def initialize(self):
        logging.info("Spinning up...")

    def execute(self):
        self.subsystem.setFlyWheelSpeed(self.speed)

    def isFinished(self):
        return self.subsystem.isReady() or self.speed <= 0.1

    def end(self, interrupted: bool):
        if not interrupted:
            logging.info(f"Done, up to speed is {self.isFinished()}")
        else:
            logging.warn("FlyWheelSpeed was interrupted!")
        # self.subsystem.setFlyWheelSpeed(0)
        return True
