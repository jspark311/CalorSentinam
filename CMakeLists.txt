#

cmake_minimum_required(VERSION 3.5)

include_directories(components)
add_definitions(-DCONFIG_MANUVR_I2C -DCONFIG_MANUVR_SUPPORT_SPI -DCONFIG_MANUVR_CBOR -DCONFIG_MANUVR_STORAGE -DCONFIG_MANUVR_M2M_SUPPORT -DCONFIG_MANUVR_IMG_SUPPORT)
set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(calor-sentinam)

#message(STATUS "****************  " ${EXTRA_COMPONENTS_DIRS})
