#include <inttypes.h>
#include <stdint.h>
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
#define COMM_TX_PIN          0   // MCU RX
#define COMM_RX_PIN          1   // MCU TX
#define FAN0_TACH_PIN        2
#define FAN1_TACH_PIN        3
#define FAN2_TACH_PIN        4
#define PUMP0_TACH_PIN       5
#define PUMP1_TACH_PIN       6
#define PUMP0_ENABLE_PIN     7
#define PUMP1_ENABLE_PIN     8
#define FAN_PWM_PIN          9
#define TEC_BANK0_PIN       20
#define TEC_BANK1_PIN       21


//#define DISPLAY_CS_PIN      10
#define SPIMOSI_PIN         11
#define SPIMISO_PIN         12
#define SPISCK_PIN          13
#define LED_R_PIN           14
#define LED_G_PIN           15
#define SCL1_PIN            16   // Sensor service bus.
#define SDA1_PIN            17   // Sensor service bus.
#define SDA0_PIN            18   // Touch controller and PMU bus.
#define SCL0_PIN            19   // Touch controller and PMU bus.
#define DISPLAY_DC_PIN      26
#define AMG8866_IRQ_PIN    255 // 30
#define DISPLAY_RST_PIN     32
#define LED_B_PIN           33




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
