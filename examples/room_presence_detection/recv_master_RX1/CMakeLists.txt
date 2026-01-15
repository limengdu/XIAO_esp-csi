# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
add_compile_options(-fdiagnostics-color=always)
# Fix strict-aliasing errors in esp-radar managed component (ESP-IDF v6.0 compatibility)
add_compile_options(-Wno-error=strict-aliasing)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(recv_master)
