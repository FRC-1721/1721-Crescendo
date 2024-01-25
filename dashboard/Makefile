#
# FRC 1721 Tidal Force
# 2023-2024
#

ROOT := $(PWD)
STAGEDIR := $(ROOT)/dist/
DASH_HASH := $(shell git describe --tags --always)

all: help
help: # List all commands and their descriptions
	@echo
	@cat Makefile | grep ': #' --color=never | sed '/^\t/d' | sed 's/: / /' | column -t -s '#'
	@echo

ready: # Run the necessary commands for development
	pipenv install
	npm ci
build: # Run `parcel build' and inject NetworkTables
	rm -rf dist/
	npm run build
	./build.sh
run: # Run a live server of the dashboard
	cd dist && pipenv run python -m pynetworktables2js
dev: # Automatically run `make build' whenever a file changes in src/ (requires program `entr')
	find src/ | entr sh -c 'make build'
clean: # Run `git clean -fdX
	git clean -fdX
stage: # Prepares dist/ for deployment
	make build
	cp entrypoint.sh $(STAGEDIR)
	cp run.bat $(STAGEDIR)
