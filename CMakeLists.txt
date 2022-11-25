cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(stickerCutter)

pico_sdk_init()

add_executable(stickerCutter
    sticker_cutter.c
    lcd/LCDops.c
    lcd/generalOps.c
    lcd/presetChars.c
    lcd/presetMessages.c
    pid/PID.c
    pwm/servo_pwm.c
)

pico_generate_pio_header(stickerCutter ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
pico_generate_pio_header(stickerCutter ${CMAKE_CURRENT_LIST_DIR}/pio_rotary_encoder.pio)

# pico_enable_stdio_usb(picoLCD 1)
# pico_enable_stdio_uart(picoLCD 1)

pico_add_extra_outputs(stickerCutter)

target_link_libraries(stickerCutter pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm)
        