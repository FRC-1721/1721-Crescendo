[![Robot Workflow](https://github.com/FRC-1721/1721-Crescendo/actions/workflows/robot-workflow.yml/badge.svg)](https://github.com/FRC-1721/1721-Crescendo/actions/workflows/robot-workflow.yml)
[![cov](https://FRC-1721.github.io/1721-Crescendo/badges/coverage.svg)](https://github.com/FRC-1721/1721-Crescendo/actions)

# 1721-Crescendo


## Developing for the Robo Rio

The robo rio code is based on RobotPy, find the docs here https://robotpy.readthedocs.io/en/stable/

To begin, setup a pipenv development environment and confirm you can run the simulator.

```shell
cd rio                # Rio Directory
pipenv install --dev  # Install requirements
pipenv shell          # Spawn a shell with pipenv python
make sim              # Run the robot simulator
make test             # Run automated tests
```
