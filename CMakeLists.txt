#

cmake_minimum_required(VERSION 3.5)

include_directories(components)
add_definitions(-D__MANUVR_ESP32=1 -DMANUVR_PLATFORM=ESP32 -DCONFIG_MANUVR_I2C -DCONFIG_MANUVR_SUPPORT_SPI -DCONFIG_MANUVR_CBOR -DCONFIG_MANUVR_M2M_SUPPORT)
set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(calor-sentinam)

#message(STATUS "****************  " ${EXTRA_COMPONENTS_DIRS})
