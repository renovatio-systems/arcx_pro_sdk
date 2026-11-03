# Builds arcx_firetv_ctrl against Nordic pc-ble-driver (static).
# SD_API selects the SoftDevice API of the dongle's connectivity firmware (5 or 6; the attached PCA10059 dongle runs API v5).

SD_API       ?= 5
DRIVER_DIR   := third_party/pc-ble-driver
ASIO_DIR     := third_party/asio/asio/include
DRIVER_BUILD := build/pc-ble-driver
DRIVER_LIB   := $(DRIVER_BUILD)/libnrf-ble-driver-sd_api_v$(SD_API).a
DRIVER_PATCH := patches/pc-ble-driver-gcc-thread-include.patch

CC      ?= gcc
CFLAGS  += -O2 -Wall -Wextra -std=gnu11 -DNRF_SD_BLE_API=$(SD_API) \
           -I$(DRIVER_DIR)/include/sd_api_v$(SD_API) \
           -I$(DRIVER_DIR)/include/common \
           -I$(DRIVER_DIR)/include/common/config \
           -I$(DRIVER_DIR)/include/common/sdk_compat
LDLIBS  += -lstdc++ -lpthread -ludev

TARGET := arcx_firetv_ctrl

all: $(TARGET)

$(TARGET): arcx_firetv_ctrl.c kasa.c kasa.h firetv.c firetv.h $(DRIVER_LIB)
	$(CC) $(CFLAGS) -o $@ arcx_firetv_ctrl.c kasa.c firetv.c $(DRIVER_LIB) $(LDLIBS)

$(DRIVER_LIB):
	@test -f $(DRIVER_DIR)/CMakeLists.txt || git submodule update --init
	@# Apply the local fix unless it is already applied.
	@git -C $(DRIVER_DIR) apply --reverse --check $(abspath $(DRIVER_PATCH)) 2>/dev/null || \
	    git -C $(DRIVER_DIR) apply $(abspath $(DRIVER_PATCH))
	cmake -S $(DRIVER_DIR) -B $(DRIVER_BUILD) -DCMAKE_BUILD_TYPE=Release \
	    -DDISABLE_TESTS=ON -DDISABLE_EXAMPLES=ON \
	    -DASIO_INCLUDE_DIR=$(abspath $(ASIO_DIR)) \
	    -DNRF_BLE_DRIVER_VERSION=4.1.4 -DCONNECTIVITY_VERSION=4.1.4 -DSD_API_VER_NUMS="5;6"
	$(MAKE) -C $(DRIVER_BUILD) nrf_ble_driver_sd_api_v$(SD_API)_static

clean:
	rm -f $(TARGET)

distclean: clean
	rm -rf build

.PHONY: all clean distclean
