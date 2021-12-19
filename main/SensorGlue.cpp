#include <SensorFilter.h>

/*******************************************************************************
* Globals
* These are extern'd into place by translation units that need them.
* Sloppy. But neatness isn't a value at the top level.
*******************************************************************************/

/* Data buffers for sensors. */
SensorFilter<float> temperature_filter_m(96, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_0(96, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_1(96, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_2(96, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_3(96, FilteringStrategy::RAW);

SensorFilter<float> pressure_filter(96, FilteringStrategy::RAW);
SensorFilter<float> humidity_filter(96, FilteringStrategy::RAW);
SensorFilter<float> air_temp_filter(96, FilteringStrategy::RAW);

SensorFilter<float> fan_speed_0(96, FilteringStrategy::RAW);
SensorFilter<float> fan_speed_1(96, FilteringStrategy::RAW);
SensorFilter<float> fan_speed_2(96, FilteringStrategy::RAW);
SensorFilter<float> pump_speed_0(96, FilteringStrategy::RAW);
SensorFilter<float> pump_speed_1(96, FilteringStrategy::RAW);


/*
* Perform the massive heap allocations for all the sensor buffers.
*/
int8_t init_sensor_memory() {
  int8_t ret = -1;
  if (0 != temperature_filter_m.init()) {    return ret;   }
  ret--;
  if (0 != temperature_filter_0.init()) {    return ret;   }
  ret--;
  if (0 != temperature_filter_1.init()) {    return ret;   }
  ret--;
  if (0 != temperature_filter_2.init()) {    return ret;   }
  ret--;
  if (0 != temperature_filter_3.init()) {    return ret;   }
  ret--;

  if (0 != pressure_filter.init()) {         return ret;   }
  ret--;
  if (0 != humidity_filter.init()) {         return ret;   }
  ret--;
  if (0 != air_temp_filter.init()) {         return ret;   }

  ret--;
  if (0 != fan_speed_0.init()) {             return ret;   }
  ret--;
  if (0 != fan_speed_1.init()) {             return ret;   }
  ret--;
  if (0 != fan_speed_2.init()) {             return ret;   }
  ret--;
  if (0 != pump_speed_0.init()) {            return ret;   }
  ret--;
  if (0 != pump_speed_1.init()) {            return ret;   }

  return 0;
}
