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
#define FAN0_TACH_PIN       12
#define FAN1_TACH_PIN       14
#define FAN2_TACH_PIN       15
#define SCL1_PIN            16   // Sensor service bus.
#define SDA1_PIN            17   // Sensor service bus.
#define SDA0_PIN            18   // Touch controller and PMU bus.
#define SCL0_PIN            19   // Touch controller and PMU bus.

#define FAN_PWM_PIN         21
#define PUMP0_ENABLE_PIN    22
#define PUMP1_ENABLE_PIN    23

#define SPICLK_PIN          25
#define TEC_BANK0_PIN       26
#define TEC_BANK1_PIN       27

#define SPIMOSI_PIN         32
#define PUMP0_TACH_PIN      33
#define PUMP1_TACH_PIN      34
#define SPIMISO_PIN         35

//#define DISPLAY_DC_PIN      26
//#define DISPLAY_RST_PIN     32


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
