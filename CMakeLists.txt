cmake_minimum_required(VERSION 3.13)
set(ENV{PICO_SDK_PATH} "~/pico/pico-sdk/")
include(pico_sdk_import.cmake)
project(motorPID C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(motorPID
  motorPID.cpp
)
pico_add_extra_outputs(motorPID)

pico_enable_stdio_usb(motorPID 1)
pico_enable_stdio_uart(motorPID 0)
target_link_libraries(motorPID pico_stdlib hardware_pwm hardware_gpio hardware_timer hardware_i2c hardware_uart)
