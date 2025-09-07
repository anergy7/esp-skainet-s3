cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS
    ../../components
    )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(speech_commands_recognition)

# Ignore unknown warning options that might be causing issues
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-warning-option")
