import logging
import commands2


class FlyUntilTrigger(commands2.Command):
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
        print(self.subsystem.switchPress())

    def isFinished(self):
        return self.subsystem.switchPress()

    def end(self, interrupted: bool):
        self.subsystem.setFlyWheelSpeed(0)
        logging.info(f"Done, up to speed is {self.isFinished()}")
        # self.subsystem.setFlyWheelSpeed(0)
        return True
