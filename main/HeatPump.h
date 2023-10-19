/* Compiler */
#include <inttypes.h>
#include <stdint.h>
#include <math.h>

/* ManuvrPlatform */
#include <ESP32.h>

/* CppPotpourri */
#include <CppPotpourri.h>
#include <AbstractPlatform.h>
#include <StringBuilder.h>
#include <ParsingConsole.h>
#include <TimerTools.h>
#include <uuid.h>
#include <Identity/Identity.h>
#include <Identity/IdentityUUID.h>
#include <SensorFilter.h>
#include <BusQueue/I2CAdapter.h>
#include <BusQueue/SPIAdapter.h>
#include <BusQueue/UARTAdapter.h>
#include <M2MLink/M2MLink.h>
#include <Image/Image.h>
#include <Image/ImageUtils.h>
#include <cbor-cpp/cbor.h>

/* ManuvrDrivers */
#include <ManuvrDrivers.h>

#ifndef __HEAT_PUMP_H__
#define __HEAT_PUMP_H__

// TODO: I _HaTe* that I have replicated this awful pattern of hard-coded
//   program versions (which are never updated) into so many projects. Finally
//   decide on a means of doing this that more-closely resembles the awesome
//   arrangement that I have at LTi for automatically binding the firmware
//   version to source-control.
#define TEST_PROG_VERSION           "1.0"


/*******************************************************************************
* Pin definitions and hardware constants.
*******************************************************************************/
/* Platform pins */
#define UART1_RX_PIN         2   // INPUT
#define SX1503_IRQ_PIN       4   // INPUT_PULLUP
#define FAN_PWM_PIN          5   // ANALOG_OUT  (Outputs PWM at boot)
#define SX8634_RESET_PIN    12   // OUTPUT  (Boot will fail if pulled high)
#define DISPLAY_RST_PIN     13   // OUTPUT
#define DISPLAY_DC_PIN      14   // OUTPUT  (Outputs PWM at boot)
#define UART1_TX_PIN        15   // OUTPUT  (Outputs PWM at boot)
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
#define AUX0_ENABLE_PIN      0  // Reserved for relays, fans, etc.
#define TEC_HV_ENABLE_PIN    1  // Must be driven high to supply the TECs.
#define PUMP1_ENABLE_PIN     2  // Enables a pump.
#define PUMP0_ENABLE_PIN     3  // Enables a pump.
#define TEC_BANK0_P_PIN      4  // H-bridge control
#define TEC_BANK0_N_PIN      5  // H-bridge control
#define TEC_BANK1_P_PIN      6  // H-bridge control
#define TEC_BANK1_N_PIN      7  // H-bridge control

// 3V3 power domain.
#define RESERVED_IN_0_PIN    8  //
#define RESERVED_IN_1_PIN    9  //
#define RESERVED_IN_2_PIN   10  //
#define RESERVED_IN_3_PIN   11  //
#define RESERVED_IN_4_PIN   12  //
#define RESERVED_IN_5_PIN   13  //
#define CIRCUIT_CONF1_PIN   14  // Used to tell firmware about the heat circuit.
#define CIRCUIT_CONF2_PIN   15  // Used to tell firmware about the heat circuit.

/* SX8634 GPIO pins */
#define LED_TOUCH_PIN        0
#define LED_ERROR_PIN        1
#define LED_FAN_PIN          2
#define LED_PUMP_PIN         3
#define LED_TEC_PIN          4
#define VIBRATOR_PIN         7

/* H-bridge operational parameters */
#define H_BRIDGE_DEADBAND_MS    500  // How many millis between "break" and "make"?

/* Our thermal flow model needs some parameters of the machine. */
#define MASS_OF_EXCHANGER         250.0f     // Grams
#define MASS_OF_INTERNAL_COOLANT  100.0f     // Grams


/*******************************************************************************
* Invariant software parameters
*******************************************************************************/
// Indicies for tachometer arrays.
#define TACH_IDX_FAN0     0
#define TACH_IDX_FAN1     1
#define TACH_IDX_FAN2     2
#define TACH_IDX_PUMP0    3
#define TACH_IDX_PUMP1    4

// Indicies for arrays that hold temperature histories.
#define TMP_SENSE_IDX_H_BRIDGE  0  // The temperature of the H-bridge.
#define TMP_SENSE_IDX_EXT_AFF   1  // External loop afferent flow temperature.
#define TMP_SENSE_IDX_EXT_EFF   2  // External loop efferent flow temperature.
#define TMP_SENSE_IDX_INT_AFF   3  // Internal loop afferent flow temperature.
#define TMP_SENSE_IDX_INT_EFF   4  // Internal loop efferent flow temperature.
#define TMP_SENSE_IDX_AIR       5  // Temperature data from the barometer.

#define RELAY_IDX_TEC_SUPPLY         0   // The first relay is the TEC supply.
#define RELAY_IDX_AUX_OUTLET         1   // The second relay is the AUX outlet.

// What are the fans rated for?
#define RPM_RADIATOR_FAN0_RATING  1700   // What are the fans rated for?
#define RPM_RADIATOR_FAN1_RATING  1700   // What are the fans rated for?
#define RPM_RADIATOR_FAN2_RATING  1700   // What are the fans rated for?
#define RPM_PUMP0_RATING          1000   // What are the pumps rated for?
#define RPM_PUMP1_RATING          1000   // What are the pumps rated for?

// Parameters for the touch board.
#define TOUCH_DWELL_LONG_PRESS       1000  // Milliseconds for "long-press".


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


/*******************************************************************************
* Externs to hardware resources
*******************************************************************************/

/**
* Homeostasis state machine positions
* ------------------------------------------------------------------------------
* \dot
*   digraph statemachine {
*     node [shape=record, fontname=Helvetica, fontsize=10];
*     BOOT         [ label="System Setup"     style="rounded,filled" ];
*     IDLE         [ label="Idle"             fillcolor="green"  style="rounded,filled" ];
*     PROG_RUNNING [ label="Program Running"  fillcolor="green"  style="rounded,filled" ];
*     FAULT        [ label="Fault"            fillcolor="red"    style="rounded,filled" ];
*
*     BOOT         -> IDLE          [ label ="Self-diagnostics pass", arrowhead="open", style="solid" ];
*     BOOT         -> FAULT         [ label ="Self-diagnostics fail", arrowhead="open", style="solid" ];
*     IDLE         -> PROG_RUNNING  [ label ="Homeostatic program initiated", arrowhead="open", style="solid" ];
*     IDLE         -> FAULT         [ label ="Fault detected", arrowhead="open", style="dashed" ];
*     PROG_RUNNING -> IDLE          [ label ="Homeostatic program complete", arrowhead="open", style="dashed" ];
*     PROG_RUNNING -> FAULT         [ label ="Fault detected", arrowhead="open", style="dashed" ];
*     FAULT        -> BOOT          [ label ="System reboot", arrowhead="open", style="dashed" ];
*   }
* \enddot
*/
enum class HomeostasisFSM : uint8_t {
  BOOT = 0,
  IDLE,
  PROG_RUNNING,
  FAULT
};

/*
* Homeostatic parameters. All temperatures are in Celcius.
*/
class HomeostasisParams {
  public:
    // Limiting factor for maximums is the working range of the semiconductors under
    //   instrumentation
    // Exceeding these limits will cause the firmware to shut down the power and
    //   raise an alarm.
    float  temperature_max_external_loop = 80.0;   // Limiting factor: TEC
    float  temperature_max_internal_loop = 80.0;   // Limiting factor: TEC
    float  temperature_max_h_bridge      = 90.0;   // Limiting factor: MOSFETs forming the H-bridges.
    float  temperature_max_air           = 50.0;   // Limiting factor: Temperature of the air in the case.

    // Limiting factor for maximums is the freezing point of the working fluid in
    //   the given loop. Unless the CONF switches reflect otherwise, that fluid is
    //   presumed to be distilled water. If porpylene glycol, or some other
    //   low-temperature fluid is used, the switch must be set to allow the minimums
    //   to be reduced below the default.
    // Exceeding these limits will cause the firmware to reverse-bias the TECs to
    //   warm the working fluid. If this fails, the firmware will raise an alarm.
    float  temperature_min_external_loop = 5.0;
    float  temperature_min_internal_loop = 5.0;

    // These are the user-defined paramters that set the target temperatures and
    //   thresholds for the loops.
    float  temperature_target_external_loop = 10.0;   // Try to maintain external loop at 10C.
    float  temperature_delta_internal_loop  = 20.0;   // Try to maintain a 20C delta.

    uint32_t period_tach_check          = 1000;     // In milliseconds.
    float    hysteresis_fan_temperature = 2.0;      // How far deviant from the internal delta target before fans are adjusted?
    uint8_t  hysteresis_fan_alert       = 4;        // How many successive 0-RPM samples before alert?
    uint8_t  hysteresis_pump_alert      = 4;        // How many successive 0-RPM samples before alert?

    bool     conf_sw1_enable_subzero    = false;    // The external loop has a fluid that freezes below 0C.
    bool     conf_sw2_staged_tec_banks  = false;    // The TEC banks are configured for maximum delta.

    void printDebug(StringBuilder*);
    int8_t console_handler(StringBuilder* text_return, StringBuilder* args);

    static const char* fsmToStr(const HomeostasisFSM);
};


/*******************************************************************************
* Externs to hardware resources
*******************************************************************************/

extern SPIAdapter spi_bus;
extern I2CAdapter i2c0;
extern I2CAdapter i2c1;
extern SSD1331 display;

extern SX1503 sx1503;
extern SX8634* touch;

extern float fan_pwm_ratio;

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
// TODO: If you replicate this any further, you deserve the extra work you make
//   for yourself. This is a terrible pattern for so many reasons...
extern M2MLink* mlink_local;

extern HomeostasisParams homeostasis;

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
SensorFilter<float>* getTemperatureFilter(uint8_t idx);

void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity = 3500);
void timeoutCheckVibLED();
const char* const getSensorIDString(const SensorID);
void listAllSensors(StringBuilder*);

/* Top-level TEC control */
int8_t tec_safety(bool en);
int8_t tec_powered(const uint8_t bank_id, bool en);
int8_t tec_reversed(const uint8_t bank_id, bool en);
bool   tec_safety();
bool   tec_powered(const uint8_t bank_id);
bool   tec_reversed(const uint8_t bank_id);

/* Top-level pump control */
int8_t pump_powered(const uint8_t pump_id, bool en);
bool   pump_powered(const uint8_t pump_id);

int8_t report_fault_condition(int8_t);


/* Display helper routines */
uint16_t* bitmapPointer(unsigned int idx);
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
