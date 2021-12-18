#include <inttypes.h>
#include <stdint.h>
#include <AbstractPlatform.h>
#include <CppPotpourri.h>
#include <StringBuilder.h>
#include <SensorFilter.h>
#include <ManuvrDrivers.h>

#ifndef __HEAT_PUMP_H__
#define __HEAT_PUMP_H__

#define TEST_PROG_VERSION           "1.0"
#define TOUCH_DWELL_LONG_PRESS       1000  // Milliseconds for "long-press".


/*******************************************************************************
* Pin definitions and hardware constants.
*******************************************************************************/
/* Platform pins */
#define DISPLAY_RST_PIN      4
#define DISPLAY_DC_PIN      12
#define DISPLAY_CS_PIN      14
#define FAN_PWM_PIN         15
#define SCL1_PIN            16   // Sensor service bus.
#define SDA1_PIN            17   // Sensor service bus.
#define SDA0_PIN            18   // Touch controller and PMU bus.
#define SCL0_PIN            19   // Touch controller and PMU bus.

#define SX8634_RESET_PIN    21
#define SX8634_IRQ_PIN      22
#define SX1503_IRQ_PIN      23
#define SPICLK_PIN          25
#define SPIMISO_PIN         26
#define SPIMOSI_PIN         27
#define FAN0_TACH_PIN       32
#define FAN1_TACH_PIN       33
#define FAN2_TACH_PIN       34
#define PUMP0_TACH_PIN      35
#define PUMP1_TACH_PIN      39


/* SX1503 GPIO pins */
// All outputs are on a 5V power domain and default to a condition "disabled".
#define PUMP0_ENABLE_PIN     0  // Enables a pump.
#define PUMP1_ENABLE_PIN     1  // Enables a pump.
#define AUX0_ENABLE_PIN      2  // Reserved for relays, fans, etc.
#define AUX1_ENABLE_PIN      3  // Reserved for relays, fans, etc.
#define TEC_BANK0_P_PIN      4  // H-bridge control
#define TEC_BANK0_N_PIN      5  // H-bridge control
#define TEC_BANK1_P_PIN      6  // H-bridge control
#define TEC_BANK1_N_PIN      7  // H-bridge control

#define DEBUG_LED_0_PIN      8  // Status output
#define DEBUG_LED_1_PIN      9  // Status output
#define DEBUG_LED_2_PIN     10  // Status output
#define DEBUG_LED_3_PIN     11  // Status output
#define RESERVED_IN_0_PIN   12  //
#define RESERVED_IN_1_PIN   13  //
#define CIRCUIT_CONF0_PIN   14  // Used to tell firmware about the heat circuit.
#define CIRCUIT_CONF1_PIN   15  // Used to tell firmware about the heat circuit.


/* SX8634 GPIO pins */
#define LED_TOUCH_PIN        0
#define LED_ERROR_PIN        1
#define LED_FAN_PIN          2
#define LED_PUMP_PIN         3
#define LED_TEC_PIN          4
#define VIBRATOR_PIN         7


/*******************************************************************************
* Data handling flags
*******************************************************************************/


/*******************************************************************************
* Types
*******************************************************************************/
enum class SensorID : uint8_t {
  TEMP_RADIATOR_IN     = 0,  //
  TEMP_RADIATOR_OUT    = 1,  //
  TEMP_XCHANGER0_IN    = 2,  //
  TEMP_XCHANGER0_OUT   = 3,  //
  TEMP_XCHANGER1_IN    = 4,  //
  TEMP_XCHANGER1_OUT   = 5,  //
  CURRENT_TEC_BANK_0   = 6,  //
  CURRENT_TEC_BANK_1   = 7,  //
};


class BrushlessFan {
  public:
    BrushlessFan(const uint8_t accl_pin, const uint8_t tach_pin, const uint8_t target_speed);
    ~BrushlessFan();

    int8_t init();
    void   setTargetSpeed();
    inline uint8_t speed() {    return _rps_last;   };


  private:
    const uint8_t _ACCEL_PIN;
    const uint8_t _TACH_PIN;
    uint8_t       _rps_target;
    uint8_t       _rps_last;
};






/*******************************************************************************
* Function prototypes
*******************************************************************************/
void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity = 3500);
void timeoutCheckVibLED();
const char* const getSensorIDString(const SensorID);
void listAllSensors(StringBuilder*);

#endif    // __HEAT_PUMP_H__
