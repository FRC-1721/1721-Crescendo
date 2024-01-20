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

build: # Run `parcel build' and inject NetworkTables
	rm -rf dist/
	npm run build
	./build.sh

run: # Run a live server of the dashboard
	npm run run
dev: # Automatically run `make build' whenever a file changes in src/ (requires program `entr')
	find src/ | entr sh -c 'make build'
clean: # Run `git clean -fdX
	git clean -fdX
stage: # Prepares dist/ for deployment
	make build
	cp entrypoint.sh $(STAGEDIR)
	cp run.bat $(STAGEDIR)
deploy: # Use scp to deploy the new files to the nuc
	make stage
	@echo "═══════════════════════════════════════════════════════════════════════════════════════"
	@echo "When prompted for a password, type 'dash' and press enter."
	@echo "═══════════════════════════════════════════════════════════════════════════════════════"

	# Transfer
	scp -r $(STAGEDIR)/* dash@10.17.21.11:/opt/dashboard

	# Restart
	ssh dash@10.17.21.11 "sudo -S systemctl daemon-reload"
	ssh dash@10.17.21.11 "sudo -S systemctl restart dashboard"