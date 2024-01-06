# FRC 1721 Tidal Force
# 2022

# Because robotpy is so easy to use, this is moreso a
# collection of shortcuts, handy for doing simple scripts
# and macros.

all: help

help:
	@echo
	@echo "Targets:"
	@echo "   sim               Shortcut for roboy.py sim."
	@echo "   clean             Shortucut for git clean -fdX"
	@echo "   deploy            Deploy code with netconsole"
	@echo "   push              Deploy code by the seat of your pants"
	@echo "   download          Download extra requirements"
	@echo "   install           Install extra requirements"
	@echo "   info              Get the robot current build info"
	@echo "   download-python   Download python (new robots only)"
	@echo "   install-python   Install python (new robots only)"
	@echo

.PHONY: deploy

# Shortcut for the simulator
sim:
	python robot.py sim

# Shortcut for deploy
deploy:
	python robot.py deploy --nc 

# Ommit the --nc, this is a custom keyword~
push:
	python robot.py deploy

# Shortcut for the download command
download:
	robotpy-installer download -r robot_requirements.txt

# Shortcut for the install command, run download first!
install:
	robotpy-installer install -r robot_requirements.txt

# Shortcut to get information about the code already on the bot
info:
	python robot.py deploy-info

# Shortcuts for inital robot setup
download-python:
	robotpy-installer download-python

install-python:
	robotpy-installer install-python

clean:
	git clean -fdX
