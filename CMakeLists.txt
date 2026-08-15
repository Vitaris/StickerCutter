# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(stickerCutter)

execute_process(
    COMMAND git describe --tags --always
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

pico_sdk_init()

add_executable(stickerCutter
    sticker_cutter.c
    core1_main.c
    lcd/ant_lcd.c
    pid/PID.c
    servo_motor/servo_pwm.c
    servo_motor/servo_motor.c
    servo_motor/button.c
    machine/machine_controller.c
    machine/machine_manual_mode.c
    machine/machine_automatic_mode.c
    hmi/hmi.c
    rotary_encoder/rotary_encoder.c
)

target_compile_definitions(stickerCutter PRIVATE GIT_VERSION=\"${GIT_VERSION}\")
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
        hardware_watchdog
        )
        
# Add a custom option in CMake (ON by default for testing)
option(ENABLE_SIMULATION_MODE "Build with simulated hardware" ON)

if(ENABLE_SIMULATION_MODE)
    # Defines SIMULATION_MODE globally for all source files in the project
    add_compile_definitions(SIMULATION_MODE)
endif()