# Tidal Force Robotics
# 2022

import os
from sre_constants import CATEGORY_WORD
from wpilib import RobotBase
import yaml
import logging

"""
Use this file only for storing non-changing constants.
"""


def getConstants(identifier):
    constants = {}

    # Clunky but it works
    if RobotBase.isReal():
        path = "/home/lvuser/py/constants/"
    else:
        path = "constants/"

    retries = 3

    try:
        while retries > 0:
            try:
                # Try opening requested .yaml
                with open(f"{path}{identifier}.yaml", "r") as yamlFile:
                    # Use yaml.safe_load to load the yaml into a dict
                    constants = yaml.safe_load(yamlFile)
            except FileNotFoundError as e:
                logging.info(
                    f"File {identifier} not found, currently in {os.getcwd()}, trying alternative locations... {retries} left. Trying to load path {path}{identifier}.yaml"
                )
                path = os.getcwd().replace("tests", "constants/")
                retries -= 1
                continue  # Retry loop
            break
    except FileNotFoundError as e:
        # If the file is not found, report it!
        logging.error(f"{identifier} config not found!")
        raise e

    # When all is done, return the important bits!
    return constants
