#include <SensorFilter.h>
#include <ManuvrDrivers.h>

/* Sensor representations */
extern TMP102 tmp102;
extern GridEYE grideye;

/* SensorFilters. These are the memory hogs. */
extern SensorFilter<float> temperature_filter_0;
extern SensorFilter<float> temperature_filter_1;
extern SensorFilter<float> temperature_filter_2;
extern SensorFilter<float> temperature_filter_3;
extern SensorFilter<float> temperature_filter_4;
extern SensorFilter<float> temperature_filter_5;
extern SensorFilter<float> temperature_filter_6;
extern SensorFilter<float> temperature_filter_7;

extern SensorFilter<float> current_filter_0;
extern SensorFilter<float> current_filter_1;
extern SensorFilter<float> current_filter_2;

extern SensorFilter<uint16_t> fan_speed_0;
extern SensorFilter<uint16_t> fan_speed_1;
extern SensorFilter<uint16_t> fan_speed_2;
extern SensorFilter<uint16_t> pump_speed_0;
extern SensorFilter<uint16_t> pump_speed_1;


int8_t init_sensor_memory();
