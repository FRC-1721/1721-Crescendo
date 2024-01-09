# Helper function for getting deploy data
import json

from wpilib import RobotBase


def getDeployData():
    """
    Returns appropriate deploy data.
    """

    fakedata = {
        "git-desc": "week0-69-g42096-dirty",
        "git-branch": "sim/sim",
        "deploy-host": "SimulatedLaptop",
        "deploy-user": "SimUser",
        "code-path": "/sim/simulatedrobot",
        "deploy-date": "never",
    }

    if RobotBase.isReal():
        try:
            with open("/home/lvuser/py/deploy.json") as fp:
                return json.load(fp)
        except FileNotFoundError:
            return fakedata
    else:
        return fakedata
