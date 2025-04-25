cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(stickerCutter)

pico_sdk_init()

add_executable(stickerCutter
    sticker_cutter.c
    lcd/ant_lcd.c
    pid/PID.c
    servo_motor/servo_pwm.c
    servo_motor/servo_motor.c
    servo_motor/button.c
    machine/machine_controller.c
    machine/machine_manual_mode.c
    machine/machine_automatic_mode.c
    machine/mark_detector.c
)

pico_generate_pio_header(stickerCutter ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(stickerCutter 1)
pico_enable_stdio_uart(stickerCutter 0)

pico_add_extra_outputs(stickerCutter)

target_link_libraries(stickerCutter pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm
        hardware_adc
        )
        