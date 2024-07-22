file(GLOB_RECURSE srcs
  *.c
  *.cpp
)

idf_component_register(SRCS "${srcs}"
  INCLUDE_DIRS "srcs"
  PRIV_INCLUDE_DIRS "srcs"
  REQUIRES nvs_flash CppPotpourri ManuvrDrivers ManuvrPlatform
  PRIV_REQUIRES esp_http_client mqtt esp_wifi
)

target_compile_options(${COMPONENT_LIB} PUBLIC -Wno-ignored-qualifiers -Wno-implicit-fallthrough)
