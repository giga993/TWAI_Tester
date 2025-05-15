# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
 
set(COMPONENTS cxx app bootloader esptool_py main esp_gdbstub)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(TWAI_Tester)
 
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)