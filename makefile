
PROJECT_NAME := led-controller
BUILD_DIR := ${CURDIR}/build

PICO_TARGET := rp2350
EXTRA_LIB_DEPENDENCIES := 

include Pico-Zig/rules.mk


# == Repos ==
pico-examples:
	git clone https://github.com/raspberrypi/pico-examples.git

