import logging
import commands2

from subsystems.shooter import Shooter


class DefaultFlywheel(commands2.Command):
    def __init__(self, speed, _shooter: Shooter):
        super().__init__()

        # local subsystem instance
        self.shooter = _shooter

        # requested speed
        self.speed = speed

        # Command requirements
        self.addRequirements(self.shooter)

    def execute(self):
        """
        offset = 0.1
        if self.speed() > offset:
            self.shooter.setFlyWheelSpeed(
                self.speed() - (offset * 1.1)
            )  # To increase the deadzone
            logging.debug(f"Flywheel at speed {self.shooter.currentSpeed()}")"""
        minimum = 0  # increase this one for more deadzone
        maximum = 0.78
        if self.speed() > minimum:
            self.shooter.setFlyWheelSpeed(
                (self.speed() - minimum) * (1 / (maximum - minimum))
            )
            logging.debug(f"Flywheel at speed {self.shooter.currentSpeed()}")
        else:
            self.shooter.setFlyWheelSpeed(0)
            logging.debug(f"Flywheel to 0")

    def end(self, interrupted: bool):
        if not interrupted:
            logging.info(f"Default flywheel is done with speed {self.isFinished()}")
        else:
            logging.warning("Default Flywheel was interrupted!")

        return True
