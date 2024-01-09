# FRC 1721 Tidal Force
# 2022

# Because robotpy is so easy to use, this is moreso a
# collection of shortcuts, handy for doing simple scripts
# and macros.

.PHONY: help

help:	## Prints this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

sim:	## Run the simulator
	python -m robotpy sim


deploy:	## Deploy to the robot (with console)
	python -m robotpy deploy --nc 


push:	## Deploy to the robot (push only, no console)
	python -m robotpy deploy


download:	## Download robot requirements locally
	robotpy-installer download -r robot_requirements.txt


install:	## Install requirements, run download first!
	robotpy-installer install -r robot_requirements.txt


info:	## Shortcut to get information about the code already on the bot
	python -m robotpy deploy-info

download-python:	## Download python (for robot)
	robotpy-installer download-python

install-python:	## Install python (for robot)
	robotpy-installer install-python

clean:	## Clean the repo
	git clean -fdX
