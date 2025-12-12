SHELL := /bin/bash
PHONY_TARGETS := venv build flash monitor clean
.PHONY: $(PHONY_TARGETS)

ACTIVATE_VENV := ../zephyrproject/.venv/bin/activate

build :
	source $(ACTIVATE_VENV) && \
	west build -b esp32s3_devkitc/esp32s3/procpu

flash :
	source $(ACTIVATE_VENV) && \
	west flash

monitor :
	sudo tio /dev/ttyACM0

clean :
	rm -rf build/