cmake_minimum_required(VERSION 3.13)

# planck project is BLE ONLY (for now)
set(PICO_BOARD pico_w)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(planck_picow)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(planck
    planck.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(planck 
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_adc
)
target_include_directories(planck PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
)
pico_btstack_make_gatt_header(planck PRIVATE "${CMAKE_CURRENT_LIST_DIR}/planck.gatt")

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(planck)
