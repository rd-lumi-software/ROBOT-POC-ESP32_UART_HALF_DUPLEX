cmake_minimum_required(VERSION 3.16)

# Fallback to PlatformIO's framework-espidf if system IDF_PATH is missing or invalid
if(NOT DEFINED ENV{IDF_PATH} OR NOT EXISTS "$ENV{IDF_PATH}/tools/cmake/project.cmake")
    if(DEFINED ENV{USERPROFILE} AND EXISTS "$ENV{USERPROFILE}/.platformio/packages/framework-espidf/tools/cmake/project.cmake")
        set(ENV{IDF_PATH} "$ENV{USERPROFILE}/.platformio/packages/framework-espidf")
    elseif(DEFINED ENV{HOME} AND EXISTS "$ENV{HOME}/.platformio/packages/framework-espidf/tools/cmake/project.cmake")
        set(ENV{IDF_PATH} "$ENV{HOME}/.platformio/packages/framework-espidf")
    endif()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ec4012_motor_ping)
