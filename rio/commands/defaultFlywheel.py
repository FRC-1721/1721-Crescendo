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
        offset = 0.1
        if self.speed() > offset:
            self.shooter.setFlyWheelSpeed(
                self.speed() - (offset * 1.1)
            )  # To increase the deadzone
            logging.debug(f"Flywheel at speed {self.shooter.currentSpeed()}")

    def end(self, interrupted: bool):
        if not interrupted:
            logging.info(f"Default is done with speed {self.isFinished()}")
        else:
            logging.warn("Default Flywheel was interrupted!")

        return True
