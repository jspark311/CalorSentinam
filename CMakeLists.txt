#

cmake_minimum_required(VERSION 3.5)

include_directories(components)
add_definitions(-DCONFIG_C3P_I2C -DCONFIG_C3P_SUPPORT_SPI -DCONFIG_C3P_CBOR -DCONFIG_C3P_STORAGE -DCONFIG_C3P_M2M_SUPPORT -DCONFIG_C3P_IMG_SUPPORT)
set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(calor-sentinam)

#message(STATUS "****************  " ${EXTRA_COMPONENTS_DIRS})
