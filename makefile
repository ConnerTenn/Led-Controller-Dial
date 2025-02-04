
PROJECT_NAME := led-controller
BUILD_DIR := ${CURDIR}/build

PICO_TARGET := rp2350
EXTRA_LIB_DEPENDENCIES := ${BUILD_DIR}/ws2812.pio.h

include Pico-Zig/rules.mk


${BUILD_DIR}/ws2812.pio.h: pico-sdk Pico-Zig/library/ws2812.pio | ${BUILD_DIR}
	@cd $(BUILD_DIR) && cmake .. -DPICO_SDK_PATH=${RUN_DIR}/pico-sdk && make -j 20 ${PROJECT_NAME}_ws2812_pio_h


# == Repos ==
pico-examples:
	git clone https://github.com/raspberrypi/pico-examples.git

