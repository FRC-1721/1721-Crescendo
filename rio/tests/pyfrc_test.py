"""
This module provides pyfrc tests useful for almost any robot.
use `python -m robotpy test` to run
"""

from pyfrc.tests import *

# Testable modules
from constants.constants import getConstants
from subsystems.drivesubsystem import DriveSubsystem


def test_yaml():
    assert len(getConstants("robot_controls")) > 0


def test_nothing():
    assert True  # Tests if true is true!
