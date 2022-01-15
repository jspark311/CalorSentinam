#include "HeatPump.h"


void HomeostasisParams::printDebug(StringBuilder* output) {
  output->concat("Current settings:\n");
  output->concat("\tTemperatures:\n");
  output->concatf("\t\tExternal loop (min/max):  (%.1fC / %.1fC)\n", temperature_min_internal_loop, temperature_max_internal_loop);
  output->concatf("\t\tInternal loop (min/max):  (%.1fC / %.1fC)\n", temperature_min_internal_loop, temperature_max_internal_loop);
  output->concatf("\t\tAir max:                  %.1fC\n", temperature_max_air);
  output->concatf("\t\tH-Bridge max:             %.1fC\n", temperature_max_h_bridge);
  output->concatf("\t\tExternal target:          %.1fC\n", temperature_target_external_loop);
  output->concatf("\t\tInternal delta target:    %.1fC\n", temperature_delta_internal_loop);

  output->concat("\n\tModel parameters:\n");
  output->concatf("\t\tTach check period:        %u ms\n", period_tach_check);
  output->concatf("\t\tFan adjust hysteresis:    %.1fC\n", hysteresis_fan_temperature);

  output->concat("\n\tAlerting parameters:\n");
  output->concatf("\t\tFan failure hysteresis:   %u checks\n", hysteresis_fan_alert);
  output->concatf("\t\tPump failure hysteresis:  %u checks\n", hysteresis_pump_alert);

  output->concat("\n\tHeat circuit conf switches:\n");
  output->concatf("\t\tAllow sub-zero temps:     %c\n", conf_sw1_enable_subzero   ? 'y' : 'n');
  output->concatf("\t\tTEC banks staged:         %c\n", conf_sw2_staged_tec_banks ? 'y' : 'n');
};
