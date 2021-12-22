/* Compiler */
#include <inttypes.h>
#include <stdint.h>
#include <math.h>

/* CppPotpourri */
#include <CppPotpourri.h>
#include <AbstractPlatform.h>
#include <StringBuilder.h>
#include <ParsingConsole.h>
#include <StopWatch.h>
#include <Image/Image.h>
#include <uuid.h>
#include <SensorFilter.h>
#include <ManuvrLink/ManuvrLink.h>
#include <I2CAdapter.h>
#include <SPIAdapter.h>

#include <cbor-cpp/cbor.h>

/* ManuvrDrivers */
#include <ManuvrDrivers.h>

#ifndef __HEAT_PUMP_H__
#define __HEAT_PUMP_H__

#define TEST_PROG_VERSION           "1.0"
#define TOUCH_DWELL_LONG_PRESS       1000  // Milliseconds for "long-press".


/*******************************************************************************
* Pin definitions and hardware constants.
*******************************************************************************/
/* Platform pins */
#define SX1503_IRQ_PIN       4   // INPUT_PULLUP
#define FAN_PWM_PIN          5   // ANALOG_OUT  (Outputs PWM at boot)
#define SX8634_RESET_PIN    12   // OUTPUT  (Boot will fail if pulled high)
#define DISPLAY_RST_PIN     13   // OUTPUT
#define DISPLAY_DC_PIN      14   // OUTPUT  (Outputs PWM at boot)
#define FAN1_TACH_PIN       16   // INPUT_PULLUP
#define FAN0_TACH_PIN       17   // INPUT_PULLUP
#define FAN2_TACH_PIN       18   // INPUT_PULLUP
#define TEMP_ALERT_1_PIN    19   // INPUT_PULLUP
#define TEMP_ALERT_0_PIN    21   // INPUT_PULLUP
#define SDA1_PIN            22   // Sensor service bus.
#define SCL1_PIN            23   // Sensor service bus.
#define SDA0_PIN            25   // Touch and power board service.
#define SCL0_PIN            26   // Touch and power board service.
#define DISPLAY_CS_PIN      27   // OUTPUT
#define SPIMOSI_PIN         32   // OUTPUT
#define SPICLK_PIN          33   // OUTPUT
#define PUMP0_TACH_PIN      34   // INPUT (Needs pullup)
#define PUMP1_TACH_PIN      35   // INPUT (Needs pullup)
#define SX8634_IRQ_PIN      36   // INPUT (Needs pullup)
#define TEMP_M_ALERT_PIN    39   // INPUT (Needs pullup)



/* SX1503 GPIO pins */
// 5V power domain. All outputs default to a condition "disabled".
#define PUMP0_ENABLE_PIN     0  // Enables a pump.
#define PUMP1_ENABLE_PIN     1  // Enables a pump.
#define AUX0_ENABLE_PIN      2  // Reserved for relays, fans, etc.
#define AUX1_ENABLE_PIN      3  // Reserved for relays, fans, etc.
#define TEC_BANK0_P_PIN      4  // H-bridge control
#define TEC_BANK0_N_PIN      5  // H-bridge control
#define TEC_BANK1_P_PIN      6  // H-bridge control
#define TEC_BANK1_N_PIN      7  // H-bridge control

// 3V3 power domain.
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
* Application color scheme (16-bit)
*******************************************************************************/
#define	BLACK           0x0000
#define	BLUE            0x1F00
#define	RED             0x00F8
#define	GREEN           0xE007
#define CYAN            0xFF07
#define MAGENTA         0x1FF8
#define YELLOW          0xE0FF
#define WHITE           0xFFFF


/*******************************************************************************
* Display related flags
*******************************************************************************/
#define GRAPH_FLAG_LOCK_RANGE_V             0x00800000   // Lock the V range.
#define GRAPH_FLAG_TEXT_RANGE_V             0x01000000   // Text overlay for axis values.
#define GRAPH_FLAG_TEXT_VALUE               0x02000000   // Text overlay for current value.
#define GRAPH_FLAG_PARTIAL_REDRAW           0x04000000   // Partial redraw
#define GRAPH_FLAG_FULL_REDRAW              0x08000000   // Full redraw
#define GRAPH_FLAG_DRAW_RULE_H              0x10000000   //
#define GRAPH_FLAG_DRAW_RULE_V              0x20000000   //
#define GRAPH_FLAG_DRAW_TICKS_H             0x40000000   //
#define GRAPH_FLAG_DRAW_TICKS_V             0x80000000   //



/*******************************************************************************
* Types
*******************************************************************************/
enum class SensorID : uint8_t {
  BARO                 =  0,  //
  TEMP_MOSFET_BANK     =  1,  //
  TEMP_XCHANGER0_IN    =  2,  //
  TEMP_XCHANGER0_OUT   =  3,  //
  TEMP_XCHANGER1_IN    =  4,  //
  TEMP_XCHANGER1_OUT   =  5,  //
  FAN_SPEED_0          =  6,  //
  FAN_SPEED_1          =  7,  //
  FAN_SPEED_2          =  8,  //
  PUMP_SPEED_0         =  9,  //
  PUMP_SPEED_1         = 10,  //
};

enum class DataVis : uint8_t {
  NONE,       // A time-series graph.
  GRAPH,      // A time-series graph.
  SCHEMATIC,  // A schematic render.
  FIELD,      // A 2d array.
  TEXT        // Prefer alphanumeric readout.
};


/*******************************************************************************
* Externs to hardware resources
*******************************************************************************/

extern SPIAdapter spi_bus;
extern I2CAdapter i2c0;
extern I2CAdapter i2c1;
extern SSD13xx display;

extern SX1503 sx1503;
extern SX8634* touch;

/* Sensor representations */
extern BME280I2C baro;
extern TMP102 temp_sensor_m;
extern TMP102 temp_sensor_0;
extern TMP102 temp_sensor_1;
extern TMP102 temp_sensor_2;
extern TMP102 temp_sensor_3;


/*******************************************************************************
* Externs to software singletons
*******************************************************************************/

extern ManuvrLink* m_link;

/* SensorFilters. These are the memory hogs. */
extern SensorFilter<float> temperature_filter_m;
extern SensorFilter<float> temperature_filter_0;
extern SensorFilter<float> temperature_filter_1;
extern SensorFilter<float> temperature_filter_2;
extern SensorFilter<float> temperature_filter_3;

extern SensorFilter<float> pressure_filter;
extern SensorFilter<float> humidity_filter;
extern SensorFilter<float> air_temp_filter;

extern SensorFilter<uint16_t> fan_speed_0;
extern SensorFilter<uint16_t> fan_speed_1;
extern SensorFilter<uint16_t> fan_speed_2;
extern SensorFilter<uint16_t> pump_speed_0;
extern SensorFilter<uint16_t> pump_speed_1;


/*******************************************************************************
* Function prototypes
*******************************************************************************/
int8_t init_sensor_memory();
void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity = 3500);
void timeoutCheckVibLED();
const char* const getSensorIDString(const SensorID);
void listAllSensors(StringBuilder*);

uint16_t* bitmapPointer(unsigned int idx);

/* Display helper routines */
void render_button_icon(uint8_t sym, int x, int y, uint16_t color);

void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color0, uint16_t color1, uint16_t color2,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<float>* filt0, SensorFilter<float>* filt1, SensorFilter<float>* filt2
);

void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color0, uint16_t color1,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<float>* filt0, SensorFilter<float>* filt1
);

void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<float>* filt
);

void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<uint32_t>* filt
);

void draw_progress_bar_horizontal(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_val, float percent
);

void draw_progress_bar_vertical(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_val, float percent
);

void draw_data_square_field(
  int x, int y, int w, int h,
  uint32_t flags,
  float* range_min, float* range_max,
  SensorFilter<float>* filt
);

void draw_data_view_selector(
  int x, int y, int w, int h,
  DataVis opt0, DataVis opt1, DataVis opt2, DataVis opt3, DataVis opt4, DataVis opt5,
  DataVis selected
);

void draw_3sphere(
  int x, int y, int w, int h,
  bool opaque,
  int meridians, int parallels,
  float euler_about_x, float euler_about_y   // TODO: A quat would be cleaner.
);


// TODO: Enum this?
#define ICON_CANCEL    0  // Flash-resident bitmap
#define ICON_ACCEPT    1  // Flash-resident bitmap
#define ICON_THERMO    2  // Flash-resident bitmap
#define ICON_RH        3  // Flash-resident bitmap
#define BUTTON_LEFT  252  // Drawn with code
#define BUTTON_RIGHT 253  // Drawn with code
#define BUTTON_UP    254  // Drawn with code
#define BUTTON_DOWN  255  // Drawn with code

#endif    // __HEAT_PUMP_H__