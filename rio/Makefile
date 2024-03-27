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

nc:
	robotpy deploy --nc

rush:	## Deploy now! (With netconsole)
	python -m robotpy deploy --nc --no-install --skip-test

push:	## Deploy to the robot (push only, no netconsole)
	robotpy deploy

sync:		## Sync robotpy requirements
	robotpy sync

info:	## Shortcut to get information about the code already on the bot
	python -m robotpy deploy-info

test:	## Run automated tests
	python -m robotpy test

download-python:	## Download python (for robot)
	python -m robotpy installer download-python

install-python:	## Install python (for robot)
	python -m robotpy installer install-python

clean:	## Clean the repo
	git clean -fdX
