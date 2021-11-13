#include "HeatPump.h"

/*******************************************************************************
* Globals
* These are extern'd into place by translation units that need them.
* Sloppy. But neatness isn't a value at the top level.
*******************************************************************************/

/* Data buffers for sensors. */
SensorFilter<float> temperature_filter_0(256, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_1(256, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_2(256, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_3(256, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_4(256, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_5(256, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_6(256, FilteringStrategy::RAW);
SensorFilter<float> temperature_filter_7(256, FilteringStrategy::RAW);

SensorFilter<float> current_filter_0(256, FilteringStrategy::RAW);
SensorFilter<float> current_filter_1(256, FilteringStrategy::RAW);
SensorFilter<float> current_filter_2(256, FilteringStrategy::RAW);

SensorFilter<float> fan_speed_0(256, FilteringStrategy::RAW);
SensorFilter<float> fan_speed_1(256, FilteringStrategy::RAW);
SensorFilter<float> fan_speed_2(256, FilteringStrategy::RAW);



/*
* Perform the massive heap allocations for all the sensor buffers.
*/
int8_t init_sensor_memory() {
  int8_t ret = -1;
  if (0 != temperature_filter_0.init()) {       return ret;   }
  ret--;
  if (0 != temperature_filter_1.init()) {       return ret;   }
  ret--;
  if (0 != temperature_filter_2.init()) {       return ret;   }
  ret--;
  if (0 != temperature_filter_3.init()) {       return ret;   }
  ret--;
  if (0 != temperature_filter_4.init()) {       return ret;   }
  ret--;
  if (0 != temperature_filter_5.init()) {       return ret;   }
  ret--;
  if (0 != temperature_filter_6.init()) {       return ret;   }
  ret--;
  if (0 != temperature_filter_7.init()) {       return ret;   }

  ret--;
  if (0 != current_filter_0.init()) {       return ret;   }
  ret--;
  if (0 != current_filter_1.init()) {       return ret;   }
  ret--;
  if (0 != current_filter_2.init()) {       return ret;   }

  ret--;
  if (0 != fan_speed_0.init()) {       return ret;   }
  ret--;
  if (0 != fan_speed_1.init()) {       return ret;   }
  ret--;
  if (0 != fan_speed_2.init()) {       return ret;   }

  return 0;
}
