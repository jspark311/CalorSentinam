#include "HeatPump.h"
#include <inttypes.h>
#include <stdint.h>
#include <math.h>
#include <CppPotpourri.h>
#include <Image/Image.h>


static uint32_t off_time_led_r    = 0;      // millis() when LED_R should be disabled.
static uint32_t off_time_led_g    = 0;      // millis() when LED_G should be disabled.
static uint32_t off_time_led_b    = 0;      // millis() when LED_B should be disabled.


/*******************************************************************************
* LED and vibrator control
* Only have enable functions since disable is done by timer in the main loop.
*******************************************************************************/
void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity) {
  uint32_t* millis_ptr = nullptr;
  switch (idx) {
  //  case LED_R_PIN:
  //    analogWrite(LED_R_PIN, intensity);
  //    millis_ptr = &off_time_led_r;
  //    break;
  //  case LED_G_PIN:
  //    analogWrite(LED_G_PIN, intensity);
  //    millis_ptr = &off_time_led_g;
  //    break;
  //  case LED_B_PIN:
  //    analogWrite(LED_B_PIN, intensity);
  //    millis_ptr = &off_time_led_b;
  //    break;
    default:
      return;
  }
  *millis_ptr = millis() + duration;
}



void timeoutCheckVibLED() {
  uint32_t millis_now = millis();
  //if (millis_now >= off_time_led_r) {   pinMode(LED_R_PIN, GPIOMode::INPUT);     }
  //if (millis_now >= off_time_led_g) {   pinMode(LED_G_PIN, GPIOMode::INPUT);     }
  //if (millis_now >= off_time_led_b) {   pinMode(LED_B_PIN, GPIOMode::INPUT);     }
}



/*******************************************************************************
* Enum support functions
*******************************************************************************/

const char* const getSensorIDString(const SensorID e) {
  switch (e) {
    case SensorID::TEMP_RADIATOR_IN:     return "TEMP_RADIATOR_IN";
    case SensorID::TEMP_RADIATOR_OUT:    return "TEMP_RADIATOR_OUT";
    case SensorID::TEMP_XCHANGER0_IN:    return "TEMP_XCHANGER0_IN";
    case SensorID::TEMP_XCHANGER0_OUT:   return "TEMP_XCHANGER0_OUT";
    case SensorID::TEMP_XCHANGER1_IN:    return "TEMP_XCHANGER1_IN";
    case SensorID::TEMP_XCHANGER1_OUT:   return "TEMP_XCHANGER1_OUT";
    case SensorID::CURRENT_TEC_BANK_0:   return "CURRENT_TEC_BANK_0";
    case SensorID::CURRENT_TEC_BANK_1:   return "CURRENT_TEC_BANK_1";
    default:                             return "UNKNOWN";
  }
}


void listAllSensors(StringBuilder* output) {
  for (uint8_t i = 0; i < 11; i++) {
    output->concatf("%2u: %s\n", i, getSensorIDString((SensorID) i));
  }
}



BrushlessFan::BrushlessFan(const uint8_t accl_pin, const uint8_t tach_pin, const uint8_t target_speed) :
_ACCEL_PIN(accl_pin), _TACH_PIN(tach_pin), _rps_target(target_speed), _rps_last(0) {}


BrushlessFan::~BrushlessFan() {}
