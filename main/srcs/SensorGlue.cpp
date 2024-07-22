#include "HeatPump.h"

/*******************************************************************************
* Globals
* These are extern'd into place by translation units that need them.
* Sloppy. But neatness isn't a value at the top level.
*******************************************************************************/

/* Data buffers for sensors. */
TimeSeries<float> temperature_filter_m(96);
TimeSeries<float> temperature_filter_0(96);
TimeSeries<float> temperature_filter_1(96);
TimeSeries<float> temperature_filter_2(96);
TimeSeries<float> temperature_filter_3(96);

TimeSeries<float> pressure_filter(96);
TimeSeries<float> humidity_filter(96);
TimeSeries<float> air_temp_filter(96);

TimeSeries<uint16_t> fan_speed_0(96);
TimeSeries<uint16_t> fan_speed_1(96);
TimeSeries<uint16_t> fan_speed_2(96);
TimeSeries<uint16_t> pump_speed_0(96);
TimeSeries<uint16_t> pump_speed_1(96);


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


TimeSeries<float>* getTemperatureFilter(uint8_t idx) {
  switch (idx) {
    case TMP_SENSE_IDX_H_BRIDGE:  return &temperature_filter_m;
    case TMP_SENSE_IDX_EXT_AFF:   return &temperature_filter_0;
    case TMP_SENSE_IDX_EXT_EFF:   return &temperature_filter_1;
    case TMP_SENSE_IDX_INT_AFF:   return &temperature_filter_2;
    case TMP_SENSE_IDX_INT_EFF:   return &temperature_filter_3;
    case TMP_SENSE_IDX_AIR:       return &air_temp_filter;
  }
  return nullptr;
}
