cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME kLineCan)
project(${PROJECT_NAME})

set(${CMAKE_PROJECT_NAME}_SKETCH kLineToCan.ino)
generate_arduino_firmware(${CMAKE_PROJECT_NAME})
include_directories( "../libraries/"  "C:\\Program Files (x86)\\Arduino\\")
add_compile_definitions(__AVR_ATmega328P__)