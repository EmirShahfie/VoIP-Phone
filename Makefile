SHELL := /bin/bash
PHONY_TARGETS := venv build flash monitor clean
.PHONY: $(PHONY_TARGETS)

.ONESHELL:
ACTIVATE_VENV := . ../zephyrproject/.venv/bin/activate

venv:
	$(ACTIVATE_VENV)

build :
	west build -b esp32s3_devkitc/esp32s3/procpu

flash :
	west flash

monitor :
	sudo tio /dev/ttyACM0

clean :
	rm -rf build/