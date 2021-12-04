idf_component_register(
  SRC_DIRS "."
  PRIV_INCLUDE_DIRS "."
  REQUIRES CppPotpourri ManuvrPlatform ManuvrDrivers nvs_flash esp_http_client esp_adc_cal mqtt
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-ignored-qualifiers -Wno-implicit-fallthrough)
