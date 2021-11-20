#include "HeatPump.h"

#include <math.h>
#include <StringBuilder.h>
#include <SensorFilter.h>
#include <ParsingConsole.h>
#include <StopWatch.h>
#include <ManuvrLink/ManuvrLink.h>
#include <cbor-cpp/cbor.h>
#include <uuid.h>

#include <SPIAdapter.h>
#include <I2CAdapter.h>
#include <UARTAdapter.h>

#include <EEPROM.h>
#include <TimeLib.h>

#include <ManuvrDrivers.h>
#include <ManuvrArduino.h>

#include "SensorGlue.h"


/*******************************************************************************
* Globals
*******************************************************************************/

const I2CAdapterOptions i2c0_opts(
  0,   // Device number
  SDA0_PIN,
  SCL0_PIN,
  0,   // No pullups.
  400000
);

const I2CAdapterOptions i2c1_opts(
  1,   // Device number
  SDA1_PIN,
  SCL1_PIN,
  0,   // No pullups.
  400000
);


ManuvrLinkOpts link_opts(
  100,   // ACK timeout is 100ms.
  2000,  // Send a KA every 2s.
  2048,  // MTU for this link is 2 kibi.
  TCode::CBOR,   // Payloads should be CBOR encoded.
  // This side of the link will send a KA while IDLE, and
  //   allows remote log write.
  (MANUVRLINK_FLAG_SEND_KA | MANUVRLINK_FLAG_ALLOW_LOG_WRITE)
);

UARTOpts comm_unit_uart_opts {
  .bitrate       = 115200,
  .start_bits    = 0,
  .bit_per_word  = 8,
  .stop_bits     = UARTStopBit::STOP_1,
  .parity        = UARTParityBit::NONE,
  .flow_control  = UARTFlowControl::NONE,
  .xoff_char     = 0,
  .xon_char      = 0,
  .padding       = 0
};

UARTOpts usb_comm_opts {
  .bitrate       = 115200,
  .start_bits    = 0,
  .bit_per_word  = 8,
  .stop_bits     = UARTStopBit::STOP_1,
  .parity        = UARTParityBit::NONE,
  .flow_control  = UARTFlowControl::NONE,
  .xoff_char     = 0,
  .xon_char      = 0,
  .padding       = 0
};


I2CAdapter i2c0(&i2c0_opts);
I2CAdapter i2c1(&i2c1_opts);
SPIAdapter spi0(0, SPISCK_PIN, SPIMOSI_PIN, SPIMISO_PIN, 8);
UARTAdapter console_uart(0, 255, 255, 255, 255, 48, 256);
UARTAdapter comm_unit_uart(1, COMM_RX_PIN, COMM_TX_PIN, 255, 255, 2048, 2048);


/* This object will contain our relationship with the Comm unit. */
ManuvrLink* m_link = nullptr;

/* Sensor representations... */
//GridEYE grideye(0x69, AMG8866_IRQ_PIN);

TMP102 temp_sensor_0(0x48, 255);
TMP102 temp_sensor_1(0x49, 255);
TMP102 temp_sensor_2(0x4A, 255);
TMP102 temp_sensor_3(0x4B, 255);
TMP102 temp_sensor_4(0x48, 255);
TMP102 temp_sensor_5(0x49, 255);
TMP102 temp_sensor_6(0x4A, 255);
TMP102 temp_sensor_7(0x4B, 255);


/* Profiling data */
StopWatch stopwatch_main_loop_time;
StopWatch stopwatch_sensor_grideye;

/* Cheeseball async support stuff. */
uint32_t boot_time         = 0;      // millis() at boot.
uint32_t config_time       = 0;      // millis() at end of setup().
uint32_t last_interaction  = 0;      // millis() when the user last interacted.

/* Console junk... */
ParsingConsole console(128);
const char* console_prompt_str = "HeatPump # ";

volatile static uint32_t fan0_tach_counter  = 0;
volatile static uint32_t fan1_tach_counter  = 0;
volatile static uint32_t fan2_tach_counter  = 0;
volatile static uint32_t pump0_tach_counter = 0;
volatile static uint32_t pump1_tach_counter = 0;
volatile static uint32_t last_tach_check    = 0;

static uint16_t fan0_target_rpm   = 200;
static uint16_t fan1_target_rpm   = 200;
static uint16_t fan2_target_rpm   = 200;

static uint8_t fan0_pwm_percent   = 100;
static uint8_t fan1_pwm_percent   = 100;
static uint8_t fan2_pwm_percent   = 100;

static float    fan_pwm_ratio     = 0.0;


/*******************************************************************************
* ISRs
*******************************************************************************/
void isr_fan0_tach_fxn() {         fan0_tach_counter++;       }
void isr_fan1_tach_fxn() {         fan1_tach_counter++;       }
void isr_fan2_tach_fxn() {         fan2_tach_counter++;       }
void isr_pump0_tach_fxn() {        pump0_tach_counter++;      }
void isr_pump1_tach_fxn() {        pump1_tach_counter++;      }


void update_tach_values() {
  const uint32_t FAN_COUNT            = 3;
  const uint32_t FAN_RPM_HYSTERESIS   = 60;
  const uint32_t FAN_PERCENTAGE_DELTA = 2;
  const uint32_t now = millis();
  uint32_t ms_delta_tach = wrap_accounted_delta(last_tach_check, now);
  if (ms_delta_tach >= 1000) {
    // Every second or so, update the tach values.
    //for (uint i = 0; i < FAN_COUNT; i++) {
    //}
    uint32_t tmp_fan0_count  = fan0_tach_counter;
    uint32_t tmp_fan1_count  = fan1_tach_counter;
    uint32_t tmp_fan2_count  = fan2_tach_counter;
    uint32_t tmp_pump0_count = pump0_tach_counter;
    uint32_t tmp_pump1_count = pump1_tach_counter;
    fan0_tach_counter  -= tmp_fan0_count;
    fan1_tach_counter  -= tmp_fan1_count;
    fan2_tach_counter  -= tmp_fan2_count;
    pump0_tach_counter -= tmp_pump0_count;
    pump1_tach_counter -= tmp_pump1_count;

    // We want RPMs
    fan_speed_0.feedFilter((uint16_t) ((tmp_fan0_count * 30000) / ms_delta_tach));
    fan_speed_1.feedFilter((uint16_t) ((tmp_fan1_count * 30000) / ms_delta_tach));
    fan_speed_2.feedFilter((uint16_t) ((tmp_fan2_count * 30000) / ms_delta_tach));
    pump_speed_0.feedFilter((uint16_t) ((tmp_pump0_count * 30000) / ms_delta_tach));
    pump_speed_1.feedFilter((uint16_t) ((tmp_pump1_count * 30000) / ms_delta_tach));

    if (fan_speed_0.dirty()) {
      const uint16_t FAN_THRESHOLD_SPEED_UP   = fan0_target_rpm + FAN_RPM_HYSTERESIS;
      const uint16_t FAN_THRESHOLD_SPEED_DOWN = fan0_target_rpm - FAN_RPM_HYSTERESIS;   // TODO: magnitude check.
      const uint16_t LAST_RPM_VALUE           = fan_speed_0.value();
      if ((0 == LAST_RPM_VALUE) && (fan0_pwm_percent >= 100)) {   // Test for fan failure.
      }
      else if (LAST_RPM_VALUE < FAN_THRESHOLD_SPEED_UP) {    fan0_pwm_percent += FAN_PERCENTAGE_DELTA;   }
      else if (LAST_RPM_VALUE > FAN_THRESHOLD_SPEED_DOWN) {  fan0_pwm_percent -= FAN_PERCENTAGE_DELTA;   }
    }
    if (fan_speed_1.dirty()) {
      const uint16_t FAN_THRESHOLD_SPEED_UP   = fan1_target_rpm + FAN_RPM_HYSTERESIS;
      const uint16_t FAN_THRESHOLD_SPEED_DOWN = fan1_target_rpm - FAN_RPM_HYSTERESIS;   // TODO: magnitude check.
      const uint16_t LAST_RPM_VALUE           = fan_speed_1.value();
      if ((0 == LAST_RPM_VALUE) && (fan1_pwm_percent >= 100)) {   // Test for fan failure.
      }
      else if (LAST_RPM_VALUE < FAN_THRESHOLD_SPEED_UP) {    fan1_pwm_percent += FAN_PERCENTAGE_DELTA;   }
      else if (LAST_RPM_VALUE > FAN_THRESHOLD_SPEED_DOWN) {  fan1_pwm_percent -= FAN_PERCENTAGE_DELTA;   }
    }
    if (fan_speed_2.dirty()) {
      const uint16_t FAN_THRESHOLD_SPEED_UP   = fan2_target_rpm + FAN_RPM_HYSTERESIS;
      const uint16_t FAN_THRESHOLD_SPEED_DOWN = fan2_target_rpm - FAN_RPM_HYSTERESIS;   // TODO: magnitude check.
      const uint16_t LAST_RPM_VALUE           = fan_speed_2.value();
      if ((0 == LAST_RPM_VALUE) && (fan2_pwm_percent >= 100)) {   // Test for fan failure.
      }
      else if (LAST_RPM_VALUE < FAN_THRESHOLD_SPEED_UP) {    fan2_pwm_percent += FAN_PERCENTAGE_DELTA;   }
      else if (LAST_RPM_VALUE > FAN_THRESHOLD_SPEED_DOWN) {  fan2_pwm_percent -= FAN_PERCENTAGE_DELTA;   }
    }
    last_tach_check = now;
  }
}


/*******************************************************************************
* Link callbacks
*******************************************************************************/
void link_callback_state(ManuvrLink* cb_link) {
  StringBuilder log;
  log.concatf("Link (0x%x) entered state %s\n", cb_link->linkTag(), ManuvrLink::sessionStateStr(cb_link->getState()));
  //printf("%s\n\n", (const char*) log.string());
}


void link_callback_message(uint32_t tag, ManuvrMsg* msg) {
  StringBuilder log;
  KeyValuePair* kvps_rxd = nullptr;
  log.concatf("link_callback_message(0x%x): \n", tag, msg->uniqueId());
  msg->printDebug(&log);
  msg->getPayload(&kvps_rxd);
  if (kvps_rxd) {
    //kvps_rxd->printDebug(&log);
  }
  if (msg->expectsReply()) {
    int8_t ack_ret = msg->ack();
    log.concatf("\nlink_callback_message ACK'ing %u returns %d.\n", msg->uniqueId(), ack_ret);
  }
  //printf("%s\n\n", (const char*) log.string());
}



/*******************************************************************************
* Sensor service functions
*******************************************************************************/

/*
* Reads the GridEye sensor and adds the data to the pile.
*/
// int8_t read_thermopile_sensor() {
//   int8_t ret = 0;
//   for (uint8_t i = 0; i < 8; i++) {
//     for (uint8_t n = 0; n < 8; n++) {
//       uint8_t pix_idx = (7 - i) | (n << 3);  // Sensor is rotated 90-deg.
//       graph_array_therm_frame.feedFilter(grideye.getPixelTemperature(pix_idx));
//     }
//   }
//   graph_array_therm_mean.feedFilter(graph_array_therm_frame.value());
//   return ret;
// }


/*******************************************************************************
* Data aggregation and packaging
*******************************************************************************/
// /*
// * All packets
// */
// void pack_cbor_comm_packet(cbor::encoder* pkt) {
//   const uint8_t PROTO_VER = 0;
//   if (nullptr == peer) {
//     pkt->write_map(3);
//   }
//   else {
//     pkt->write_map(4);
//     peer->serializeCBOR(pkt, PROTO_VER);
//   }
//   pkt->write_string("comver");
//   pkt->write_int(PROTO_VER);   // The protocol version
//
//     pkt->write_string("orig");
//     pkt->write_map(4);
//       pkt->write_string("mod");
//       pkt->write_string("Motherflux0r");
//       pkt->write_string("ser");
//       pkt->write_int(1);
//       pkt->write_string("firm_ver");
//       pkt->write_string(TEST_PROG_VERSION);
//       pkt->write_string("ts");
//       pkt->write_tag(1);
//       pkt->write_int((uint) now());
//     pkt->write_string("pdu");
// }
//
//
// void package_sensor_data_cbor(StringBuilder* cbor_return) {
//   cbor::output_dynamic out;
//   cbor::encoder encoded(out);
//   encoded.write_map(2);
//     encoded.write_string("ds_ver");
//     encoded.write_int(0);
//
//     encoded.write_string("origin");
//     encoded.write_map(4);
//       encoded.write_string("model");
//       encoded.write_string("Motherflux0r");
//       encoded.write_string("ser");
//       encoded.write_int(1);
//       encoded.write_string("fm_ver");
//       encoded.write_string(TEST_PROG_VERSION);
//       encoded.write_string("ts");
//       encoded.write_tag(1);
//       encoded.write_int((uint) now());
//
//     encoded.write_string("meta");
//     encoded.write_map(2);
//       encoded.write_string("build_data");
//       encoded.write_tag(1);
//       encoded.write_int(1584023014);
//       encoded.write_string("cal_date");
//       encoded.write_tag(1);
//       encoded.write_int(1584035000);
//
//   cbor_return->concat(out.data(), out.size());
// }



/*******************************************************************************
* Console callbacks
*******************************************************************************/

int callback_link_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd = args->position_trimmed(0);
  if (0 == StringBuilder::strcasecmp(cmd, "info")) {
    m_link->printDebug(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    text_return->concatf("Link reset() returns %d\n", m_link->reset());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "hangup")) {
    text_return->concatf("Link hangup() returns %d\n", m_link->hangup());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "verbosity")) {
    switch (args->count()) {
      case 2:
        m_link->verbosity(0x07 & args->position_as_int(1));
      default:
        text_return->concatf("Link verbosity is %u\n", m_link->verbosity());
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "log")) {
    //if (1 < args->count()) {
      StringBuilder tmp_log("This is a remote log test.\n");
      int8_t ret_local = m_link->writeRemoteLog(&tmp_log, false);
      text_return->concatf("Remote log write returns %d\n", ret_local);
    //}
    //else text_return->concat("Usage: link log <logText>\n");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "desc")) {
    // Send a description request message.
    KeyValuePair a((uint32_t) millis(), "time_ms");
    a.append((uint32_t) randomUInt32(), "rand");
    int8_t ret_local = m_link->send(&a, true);
    text_return->concatf("Description request send() returns ID %u\n", ret_local);
  }
  else text_return->concat("Usage: [info|reset|hangup|verbosity|desc]\n");

  return ret;
}



int callback_i2c_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  int   bus_id = args->position_as_int(0);
  char* cmd    = args->position_trimmed(1);
  int   arg2   = args->position_as_int(2);
  I2CAdapter* bus = nullptr;
  switch (bus_id) {
    case 0:    bus = &i2c0;  break;
    case 1:    bus = &i2c1;  break;
    default:
      text_return->concatf("Unsupported bus: %d\n", bus_id);
      break;
  }
  if (nullptr != bus) {
    if (0 == StringBuilder::strcasecmp(cmd, "purge")) {
      bus->purge_current_job();
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "ragepurge")) {
      bus->purge_queued_work();
      bus->purge_current_job();
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "ping")) {
      bus->ping_slave_addr((args->count() > 2) ? arg2 : 1);
      text_return->concatf("i2c%d.ping_slave_addr(0x%02x) started.\n", bus_id, arg2);
    }
    else {
      bus->printDebug(text_return);
      bus->printPingMap(text_return);
    }
  }
  return ret;
}


int callback_help(StringBuilder* text_return, StringBuilder* args) {
  if (0 < args->count()) {
    console.printHelp(text_return, args->position_trimmed(0));
  }
  else {
    console.printHelp(text_return);
  }
  return 0;
}



int callback_sensor_tools(StringBuilder* text_return, StringBuilder* args) {
  int   ret = 0;
  char* cmd = args->position_trimmed(0);

  if (0 == StringBuilder::strcasecmp(cmd, "temp")) {
    //text_return->concatf("temperature sensor refresh returned %d.\n", console.localEcho()?"en":"dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "info")) {
    if (1 < args->count()) {
      int arg1 = args->position_as_int(1);
      switch (arg1) {
        case 0:   temp_sensor_0.printDebug(text_return);    break;
        case 1:   temp_sensor_1.printDebug(text_return);    break;
        case 2:   temp_sensor_2.printDebug(text_return);    break;
        case 3:   temp_sensor_3.printDebug(text_return);    break;
        case 4:   temp_sensor_4.printDebug(text_return);    break;
        case 5:   temp_sensor_5.printDebug(text_return);    break;
        case 6:   temp_sensor_6.printDebug(text_return);    break;
        case 7:   temp_sensor_7.printDebug(text_return);    break;
        default:
          break;
      }
    }
  }
  else {
    ret = -1;
  }
  return ret;
}


int callback_sensor_filter_info(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (0 < args->count()) {
    int   arg0 = args->position_as_int(0);
    char* cmd  = args->position_trimmed(1);
    SensorFilter<float>* sel_sen = nullptr;
    switch (arg0) {
      case 0:   sel_sen = &temperature_filter_0;    break;
      case 1:   sel_sen = &temperature_filter_1;    break;
      case 2:   sel_sen = &temperature_filter_2;    break;
      case 3:   sel_sen = &temperature_filter_3;    break;
      case 4:   sel_sen = &temperature_filter_4;    break;
      case 5:   sel_sen = &temperature_filter_5;    break;
      case 6:   sel_sen = &temperature_filter_6;    break;
      case 7:   sel_sen = &temperature_filter_7;    break;
      default:
        break;
    }
    if (nullptr != sel_sen) {
      if (0 == StringBuilder::strcasecmp(cmd, "info")) {
        sel_sen->printFilter(text_return);
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "purge")) {
        sel_sen->purge();
        text_return->concatf("Filter for SensorID %d purged.\n", arg0);
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "depth")) {
        if (2 < args->count()) {
          uint arg2 = (uint) args->position_as_int(2);
          text_return->concatf("Setting sample depth for filter %d returned %d.\n", arg0, sel_sen->windowSize(arg2));
        }
        text_return->concatf("Filter for SensorID %d is %u samples deep.\n", arg0, sel_sen->windowSize());
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "strat")) {
        if (2 < args->count()) {
          FilteringStrategy arg2 = (FilteringStrategy) args->position_as_int(2);
          text_return->concatf("Setting sample depth for filter %d returned %d.\n", arg0, sel_sen->setStrategy(arg2));
        }
        text_return->concatf("Filter strategy for SensorID %d is %s.\n", arg0, getFilterStr(sel_sen->strategy()));
      }
      else {
        text_return->concatf("Filter value for SensorID %d: %.3f.\n", arg0, (double) sel_sen->value());
      }
    }
    else {
      text_return->concatf("Invalid SensorID: %d\n", arg0);
      listAllSensors(text_return);
    }
    ret = 0;
  }
  else {  // No arguments means print the sensor index list.
    listAllSensors(text_return);
  }
  return ret;
}


int callback_fan_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd = args->position_trimmed(0);
  if (0 == StringBuilder::strcasecmp(cmd, "info")) {
    fan_speed_0.printFilter(text_return);
    fan_speed_1.printFilter(text_return);
    fan_speed_2.printFilter(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "speed")) {
    if (1 < args->count()) {
      float arg1 = args->position_as_double(1);
      if ((arg1 >= 0.0) & (arg1 <= 1.0)) {
        fan_pwm_ratio = arg1;
        analogWrite(FAN_PWM_PIN, arg1);
      }
      else {
        text_return->concat("Fan speed must be in the range [0, 1].\n");
      }
    }
    text_return->concatf("Fan speed set at %.2f%%\n", fan_pwm_ratio);
  }
  else {
    text_return->concatf("Fan0:\t%d RPM\n", (int) fan_speed_0.value()*60);
    text_return->concatf("Fan1:\t%d RPM\n", (int) fan_speed_1.value()*60);
    text_return->concatf("Fan2:\t%d RPM\n", (int) fan_speed_2.value()*60);
  }
  return ret;
}


int callback_pump_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  bool print_pump0_speed = false;
  bool print_pump1_speed = false;
  char* loop = args->position_trimmed(0);
  char* cmd  = args->position_trimmed(1);
  if (0 == StringBuilder::strcasecmp(loop, "i")) {
    if (1 < args->count()) {
      if (0 == StringBuilder::strcasecmp(cmd, "on")) {
        setPin(PUMP0_ENABLE_PIN, true);
        text_return->concat("Pump enabled.\n");
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "off")) {
        setPin(PUMP0_ENABLE_PIN, false);
        text_return->concat("Pump disabled.\n");
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "filter")) {
        pump_speed_0.printFilter(text_return);
      }
      else print_pump0_speed = true;
    }
    else print_pump0_speed = true;
  }
  else if (0 == StringBuilder::strcasecmp(loop, "e")) {
    if (1 < args->count()) {
      if (0 == StringBuilder::strcasecmp(cmd, "on")) {
        setPin(PUMP1_ENABLE_PIN, true);
        text_return->concat("Pump enabled.\n");
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "off")) {
        setPin(PUMP1_ENABLE_PIN, false);
        text_return->concat("Pump disabled.\n");
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "filter")) {
        pump_speed_1.printFilter(text_return);
      }
      else print_pump1_speed = true;
    }
    else print_pump1_speed = true;
  }
  else {
    text_return->concat("You must specify the pump for either the internal (i) or external (e) loop.\n");
    ret = -1;
  }
  if (print_pump0_speed) text_return->concatf("Pump0 (Internal):  %d RPM\n", (int) pump_speed_0.value()*60);
  if (print_pump1_speed) text_return->concatf("Pump1 (External):  %d RPM\n", (int) pump_speed_1.value()*60);
  return ret;
}


int callback_tec_tools(StringBuilder* text_return, StringBuilder* args) {
  int   ret  = 0;
  int   bank = args->position_as_int(0);
  char* cmd  = args->position_trimmed(1);
  if (0 < args->count()) {
    uint8_t bank_pin = TEC_BANK1_PIN;
    switch (bank) {
      case 0:
        bank_pin = TEC_BANK0_PIN;
      case 1:
        if (1 < args->count()) {
          if (0 == StringBuilder::strcasecmp(cmd, "on")) {
            setPin(bank_pin, true);
            text_return->concatf("TEC bank %d enabled.\n", bank);
          }
          else if (0 == StringBuilder::strcasecmp(cmd, "off")) {
            setPin(bank_pin, false);
            text_return->concatf("TEC bank %d disabled.\n", bank);
          }
        }
        else {
          // TDO: print bank states.
        }
        break;
      default:
        text_return->concat("You must specify the bank (0 or 1).\n");
        break;
    }
  }
  return ret;
}



int callback_console_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd    = args->position_trimmed(0);
  int   arg1   = args->position_as_int(1);
  bool  print_term_enum = false;
  if (0 == StringBuilder::strcasecmp(cmd, "echo")) {
    if (1 < args->count()) {
      console.localEcho(0 != arg1);
    }
    text_return->concatf("Console RX echo %sabled.\n", console.localEcho()?"en":"dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "history")) {
    if (1 < args->count()) {
      console.emitPrompt(0 != arg1);
      char* subcmd = args->position_trimmed(1);
      if (0 == StringBuilder::strcasecmp(subcmd, "clear")) {
        console.clearHistory();
        text_return->concat("History cleared.\n");
      }
      else if (0 == StringBuilder::strcasecmp(subcmd, "depth")) {
        if (2 < args->count()) {
          arg1 = args->position_as_int(2);
          console.maxHistoryDepth(arg1);
        }
        text_return->concatf("History depth: %u\n", console.maxHistoryDepth());
      }
      else if (0 == StringBuilder::strcasecmp(subcmd, "logerrors")) {
        if (2 < args->count()) {
          arg1 = args->position_as_int(2);
          console.historyFail(0 != arg1);
        }
        text_return->concatf("History %scludes failed commands.\n", console.historyFail()?"in":"ex");
      }
      else text_return->concat("Valid options are [clear|depth|logerrors]\n");
    }
    else console.printHistory(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "help-on-fail")) {
    if (1 < args->count()) {
      console.printHelpOnFail(0 != arg1);
    }
    text_return->concatf("Console prints command help on failure: %s.\n", console.printHelpOnFail()?"yes":"no");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "prompt")) {
    if (1 < args->count()) {
      console.emitPrompt(0 != arg1);
    }
    text_return->concatf("Console autoprompt %sabled.\n", console.emitPrompt()?"en":"dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "force")) {
    if (1 < args->count()) {
      console.forceReturn(0 != arg1);
    }
    text_return->concatf("Console force-return %sabled.\n", console.forceReturn()?"en":"dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "rxterm")) {
    if (1 < args->count()) {
      switch (arg1) {
        case 0:  case 1:  case 2:  case 3:
          console.setRXTerminator((LineTerm) arg1);
          break;
        default:
          print_term_enum = true;
          break;
      }
    }
    text_return->concatf("Console RX terminator: %s\n", ParsingConsole::terminatorStr(console.getRXTerminator()));
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "txterm")) {
    if (1 < args->count()) {
      switch (arg1) {
        case 0:  case 1:  case 2:  case 3:
          console.setTXTerminator((LineTerm) arg1);
          break;
        default:
          print_term_enum = true;
          break;
      }
    }
    text_return->concatf("Console TX terminator: %s\n", ParsingConsole::terminatorStr(console.getTXTerminator()));
  }
  else {
    ret = -1;
  }

  if (print_term_enum) {
    text_return->concat("Terminator options:\n");
    text_return->concat("\t0: ZEROBYTE\n");
    text_return->concat("\t1: CR\n");
    text_return->concat("\t2: LF\n");
    text_return->concat("\t3: CRLF\n");
  }
  return ret;
}



/*******************************************************************************
* Setup function
*******************************************************************************/
void setup() {
  platform_init();
  boot_time = millis();
  console_uart.init(&usb_comm_opts);
  console_uart.readCallback(&console);    // Attach the UART to console...
  console.setOutputTarget(&console_uart); // ...and console to UART.

  console.emitPrompt(true);
  console.setTXTerminator(LineTerm::CRLF);
  console.setRXTerminator(LineTerm::CR);
  console.localEcho(true);
  console.printHelpOnFail(true);
  console.setPromptString(console_prompt_str);

  pinMode(LED_R_PIN,      GPIOMode::INPUT);
  pinMode(LED_G_PIN,      GPIOMode::INPUT);
  pinMode(LED_B_PIN,      GPIOMode::INPUT);

  pinMode(FAN0_TACH_PIN,  GPIOMode::INPUT_PULLUP);
  pinMode(FAN1_TACH_PIN,  GPIOMode::INPUT_PULLUP);
  pinMode(FAN2_TACH_PIN,  GPIOMode::INPUT_PULLUP);
  pinMode(PUMP0_TACH_PIN, GPIOMode::INPUT_PULLUP);
  pinMode(PUMP1_TACH_PIN, GPIOMode::INPUT_PULLUP);

  pinMode(TEC_BANK0_PIN,    GPIOMode::OUTPUT);
  pinMode(TEC_BANK1_PIN,    GPIOMode::OUTPUT);
  pinMode(PUMP0_ENABLE_PIN, GPIOMode::OUTPUT);
  pinMode(PUMP1_ENABLE_PIN, GPIOMode::OUTPUT);
  pinMode(FAN_PWM_PIN,      GPIOMode::ANALOG_OUT);

  setPin(TEC_BANK0_PIN,    true);
  setPin(TEC_BANK1_PIN,    true);
  setPin(PUMP0_ENABLE_PIN, true);
  setPin(PUMP1_ENABLE_PIN, true);
  analogWrite(FAN_PWM_PIN, 0.0f);

  uint16_t serial_timeout = 0;
  while (!Serial && (100 > serial_timeout)) {
    serial_timeout++;
    delay(70);
  }

  spi0.init();
  i2c0.init();
  i2c1.init();

  comm_unit_uart.init(&comm_unit_uart_opts);

  /* Allocate memory for the filters. */
  if (0 != init_sensor_memory()) {
    console_uart.write("Failed to allocate memory for sensors.\n");
  }

  console.defineCommand("help",        '?', ParsingConsole::tcodes_str_1, "Prints help to console.", "", 0, callback_help);
  platform.configureConsole(&console);
  //console.defineCommand("disp",        'd', ParsingConsole::tcodes_uint_1, "Display test", "", 1, callback_display_test);
  console.defineCommand("sensor",      's',  ParsingConsole::tcodes_str_4, "Sensor tools", "", 0, callback_sensor_tools);
  console.defineCommand("filter",      '\0', ParsingConsole::tcodes_str_3, "Sensor filter info.", "", 0, callback_sensor_filter_info);
  console.defineCommand("fan",         'f',  ParsingConsole::tcodes_str_3, "Fan tools", "", 0, callback_fan_tools);
  console.defineCommand("pump",        'p',  ParsingConsole::tcodes_str_3, "Pump tools", "", 0, callback_pump_tools);
  console.defineCommand("tec",         't',  ParsingConsole::tcodes_str_3, "TEC tools", "", 0, callback_tec_tools);
  console.defineCommand("i2c",         '\0', ParsingConsole::tcodes_uint_3, "I2C tools", "Usage: i2c <bus> <action> [addr]", 1, callback_i2c_tools);
  console.defineCommand("console",     '\0', ParsingConsole::tcodes_str_3, "Console conf.", "[echo|prompt|force|rxterm|txterm]", 0, callback_console_tools);
  console.defineCommand("link",        'l', ParsingConsole::tcodes_str_4, "Linked device tools.", "", 0, callback_link_tools);
  console.init();

  StringBuilder ptc("HeatPump ");
  ptc.concat(TEST_PROG_VERSION);
  ptc.concat("\t Build date " __DATE__ " " __TIME__ "\n");
  console.printToLog(&ptc);

  //grideye.assignBusInstance(&i2c1);

  temp_sensor_0.init(&i2c0);
  temp_sensor_1.init(&i2c0);
  temp_sensor_2.init(&i2c0);
  temp_sensor_3.init(&i2c0);
  temp_sensor_4.init(&i2c1);
  temp_sensor_5.init(&i2c1);
  temp_sensor_6.init(&i2c1);
  temp_sensor_7.init(&i2c1);

  setPinFxn(FAN0_TACH_PIN,  IRQCondition::FALLING, isr_fan0_tach_fxn);
  setPinFxn(FAN1_TACH_PIN,  IRQCondition::FALLING, isr_fan1_tach_fxn);
  setPinFxn(FAN2_TACH_PIN,  IRQCondition::FALLING, isr_fan2_tach_fxn);
  setPinFxn(PUMP0_TACH_PIN, IRQCondition::FALLING, isr_pump0_tach_fxn);
  setPinFxn(PUMP1_TACH_PIN, IRQCondition::FALLING, isr_pump1_tach_fxn);

  config_time = millis();
}



/*******************************************************************************
* Main loop
*******************************************************************************/

void spi_spin() {
  int8_t polling_ret = spi0.poll();
  while (0 < polling_ret) {
    polling_ret = spi0.poll();
    //Serial.println("spi_spin() 0");
  }
  polling_ret = spi0.service_callback_queue();
  while (0 < polling_ret) {
    polling_ret = spi0.service_callback_queue();
    //Serial.println("spi_spin() 1");
  }
}


void loop() {
  StringBuilder output;
  uint32_t millis_now = millis();
  stopwatch_main_loop_time.markStart();

  timeoutCheckVibLED();

  spi_spin();
  i2c0.poll();
  i2c1.poll();

  /* Poll each sensor class. */
  //if (grideye.enabled()) {
  //  stopwatch_sensor_grideye.markStart();
  //  if (0 < grideye.poll()) {
  //    read_thermopile_sensor();
  //  }
  //  stopwatch_sensor_grideye.markStop();
  //}

  if (1 == temp_sensor_0.poll()) {
    temperature_filter_0.feedFilter(temp_sensor_0.temperature());
  }
  if (1 == temp_sensor_1.poll()) {
    temperature_filter_1.feedFilter(temp_sensor_1.temperature());
  }
  if (1 == temp_sensor_2.poll()) {
    temperature_filter_2.feedFilter(temp_sensor_2.temperature());
  }
  if (1 == temp_sensor_3.poll()) {
    temperature_filter_3.feedFilter(temp_sensor_3.temperature());
  }
  if (1 == temp_sensor_4.poll()) {
    temperature_filter_4.feedFilter(temp_sensor_4.temperature());
  }
  if (1 == temp_sensor_5.poll()) {
    temperature_filter_5.feedFilter(temp_sensor_5.temperature());
  }
  if (1 == temp_sensor_6.poll()) {
    temperature_filter_6.feedFilter(temp_sensor_6.temperature());
  }
  if (1 == temp_sensor_7.poll()) {
    temperature_filter_7.feedFilter(temp_sensor_7.temperature());
  }

  comm_unit_uart.poll();
  console.printToLog(&output);
  console_uart.poll();
  stopwatch_main_loop_time.markStop();
}
