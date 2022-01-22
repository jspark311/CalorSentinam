#include <math.h>

/* ManuvrPlatform */
#include <ESP32.h>

/* Local includes */
#include "HeatPump.h"
#include "uApp.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sys.h"

//#include "esp_ota_ops.h"
//#include "esp_flash_partitions.h"
#include "esp_http_client.h"
#include "mqtt_client.h"


#ifdef __cplusplus
}
#endif


/* OTA parameters that probably ought to be imparted at provisioning. */
#define EXAMPLE_SERVER_URL "ian-app.home.joshianlindsay.com"
//extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
//extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");


/*******************************************************************************
* Globals
*******************************************************************************/


esp_mqtt_client_handle_t client = nullptr;

static bool connected_have_ip   = false;
static bool connected_mqtt      = false;

// Assignment of pin combinations to TEC banks.
const uint8_t  BANK_PIN_ARRAY[4] = {TEC_BANK0_P_PIN, TEC_BANK0_N_PIN, TEC_BANK1_P_PIN, TEC_BANK1_N_PIN};

// This bus handles UI and the baro sensor.
const I2CAdapterOptions i2c0_opts(
  0,   // Device number
  SDA0_PIN,  // (sda)
  SCL0_PIN,  // (scl)
  0,   // No pullups.
  200000
);

// This bus handles 4 temperature sensors.
const I2CAdapterOptions i2c1_opts(
  1,   // Device number
  SDA1_PIN,  // (sda)
  SCL1_PIN,  // (scl)
  0,   // No pullups.
  400000
);

// Configurastions for temperature sensors.
const TMP102Opts temp_m_opts(
  0x49,             // i2c address
  TEMP_M_ALERT_PIN, // ALERT pin
  true,             // Extended mode
  true,             // Alert is active-low
  TMP102DataRate::RATE_4_HZ
);

const TMP102Opts temp_0_opts(
  0x48,             // i2c address
  TEMP_ALERT_0_PIN, // ALERT pin
  true,             // Extended mode
  true,             // Alert is active-low
  TMP102DataRate::RATE_4_HZ
);

const TMP102Opts temp_1_opts(
  0x49,             // i2c address
  255,              // ALERT pin
  true,             // Extended mode
  true,             // Alert is active-low
  TMP102DataRate::RATE_4_HZ
);

const TMP102Opts temp_2_opts(
  0x4A,             // i2c address
  TEMP_ALERT_1_PIN, // ALERT pin
  true,             // Extended mode
  true,             // Alert is active-low
  TMP102DataRate::RATE_4_HZ
);

const TMP102Opts temp_3_opts(
  0x4B,             // i2c address
  255,              // ALERT pin
  true,             // Extended mode
  true,             // Alert is active-low
  TMP102DataRate::RATE_4_HZ
);


/* Configuration for the display. */
const SSD13xxOpts disp_opts(
  ImgOrientation::ROTATION_180,
  DISPLAY_RST_PIN,
  DISPLAY_DC_PIN,
  DISPLAY_CS_PIN,
  SSDModel::SSD1331
);

const uint8_t sx1503_config[SX1503_SERIALIZE_SIZE] = {
  0x01, SX1503_IRQ_PIN, 0xff, 0x00, 0x00,
  0x00, 0x00,  // All inputs default to safe states after init.
  0xF0, 0x00,  // 0-7 are outputs into a 5V domain. 8-11 are debugging LEDs. 12-15 are INPU_PULLUP.
  0xF0, 0x00,  // All inputs have pull-ups.
  0x00, 0x00,  // No pull-downs.
  0x0F, 0xFF,  // Interrupts unmasked for all inputs.
  0xFF, 0x00,  // All inputs are sensitive to both edges.
  0x00, 0x00,  // All inputs are sensitive to both edges.
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x04
};

BME280Settings baro_settings(
  0x76,
  BME280OSR::X1,
  BME280OSR::X1,
  BME280OSR::X1,
  BME280Mode::Normal,
  BME280StandbyTime::StandbyTime_1000ms,
  BME280Filter::Off
);

/* Touch board */
const SX8634Opts _touch_opts(
  SX8634_DEFAULT_I2C_ADDR,  // i2c address
  SX8634_RESET_PIN,         // Reset pin. Output. Active low.
  SX8634_IRQ_PIN            // IRQ pin. Input. Active low. Needs pullup.
);
SX8634* touch = nullptr;


ManuvrLinkOpts link_opts(
  100,   // ACK timeout is 100ms.
  2000,  // Send a KA every 2s.
  2048,  // MTU for this link is 2 kibi.
  TCode::CBOR,   // Payloads should be CBOR encoded.
  // This side of the link will send a KA while IDLE, and
  //   allows remote log write.
  (MANUVRLINK_FLAG_SEND_KA | MANUVRLINK_FLAG_ALLOW_LOG_WRITE)
);



static const char* TAG         = "main-cpp";
const char* console_prompt_str = "HeatPump # ";
ParsingConsole console(128);
ESP32StdIO console_uart;
SPIAdapter spi_bus(1, SPICLK_PIN, SPIMOSI_PIN, 255, 8);   // NOTE: No MISO pin.
I2CAdapter i2c0(&i2c0_opts);
I2CAdapter i2c1(&i2c1_opts);

/* This object will contain our direct-link via TCP. */
ManuvrLink* m_link = nullptr;

SX1503 sx1503(sx1503_config, SX1503_SERIALIZE_SIZE);   // GPIO on the power control board.
SSD13xx display(&disp_opts);
BME280I2C baro(baro_settings);
TMP102 temp_sensor_m(&temp_m_opts);
TMP102 temp_sensor_0(&temp_0_opts);
TMP102 temp_sensor_1(&temp_1_opts);
TMP102 temp_sensor_2(&temp_2_opts);
TMP102 temp_sensor_3(&temp_3_opts);

/* Profiling data */
StopWatch stopwatch_main_loop_time;

/* User-provided operating parameters */
HomeostasisParams homeostasis;


/* Cheeseball async support stuff. */
uint32_t boot_time         = 0;      // millis() at boot.
uint32_t config_time       = 0;      // millis() at end of setup().
uint32_t last_interaction  = 0;      // millis() when the user last interacted.
uint32_t last_tach_check   = 0;      // millis() when the last tach check happened.
float    fan_pwm_ratio     = 0.0;

uint8_t  tach_fails[5]     = {0, 0, 0, 0, 0};    // Successive tachometer failures.

volatile static uint32_t tach_counters[5] = {0, 0, 0, 0, 0};

HomeostasisFSM homeostate = HomeostasisFSM::BOOT;


/*******************************************************************************
* ISRs
*******************************************************************************/
void IRAM_ATTR isr_fan0_tach_fxn() {     tach_counters[TACH_IDX_FAN0]++;    }
void IRAM_ATTR isr_fan1_tach_fxn() {     tach_counters[TACH_IDX_FAN1]++;    }
void IRAM_ATTR isr_fan2_tach_fxn() {     tach_counters[TACH_IDX_FAN2]++;    }
void IRAM_ATTR isr_pump0_tach_fxn() {    tach_counters[TACH_IDX_PUMP0]++;   }
void IRAM_ATTR isr_pump1_tach_fxn() {    tach_counters[TACH_IDX_PUMP1]++;   }


/*******************************************************************************
* Process functions called from the main service loop.
*******************************************************************************/

void update_tach_values() {
  const uint32_t now = millis();
  const uint32_t ms_delta_tach = wrap_accounted_delta(last_tach_check, now);
  // Every second or so, update the tach values.
  if (ms_delta_tach >= homeostasis.period_tach_check) {
    //printf("update_tach_values(): %u  %u  %u  %u  %u\n", tach_counters[0], tach_counters[1], tach_counters[2], tach_counters[3], tach_counters[4]);
    for (uint i = 0; i < 5; i++) {  // We have five tachometer values to update.
      uint32_t tmp_tach_count  = tach_counters[i];
      tach_counters[i] -= tmp_tach_count;     // Decrement by the amount we noted.
      // We want RPMs. We get two (four?) counts per full revolution, and we are dealing
      //   with sample periods in terms of milliseconds. Thus....
      uint16_t tmp_tach_rpm = (uint16_t) (tmp_tach_count * (15000.0 / (float) ms_delta_tach));
      //printf("tmp_tach_rpm:   %u\n", tmp_tach_rpm);
      SensorFilter<uint16_t>* filter = nullptr;
      uint8_t tach_alert_threshold = 0;
      switch (i) {
        case TACH_IDX_FAN0:     filter = &fan_speed_0;     break;
        case TACH_IDX_FAN1:     filter = &fan_speed_1;     break;
        case TACH_IDX_FAN2:     filter = &fan_speed_2;     break;
        case TACH_IDX_PUMP0:    filter = &pump_speed_0;    break;
        case TACH_IDX_PUMP1:    filter = &pump_speed_1;    break;
        default:  return;
      }
      switch (i) {
        case TACH_IDX_FAN0:
        case TACH_IDX_FAN1:
        case TACH_IDX_FAN2:
          tach_alert_threshold = homeostasis.hysteresis_fan_alert;
          break;
        case TACH_IDX_PUMP0:
        case TACH_IDX_PUMP1:
          tach_alert_threshold = homeostasis.hysteresis_pump_alert;
          break;
      }
      filter->feedFilter(tmp_tach_rpm);
      if (0 == tmp_tach_rpm) {
        tach_fails[i]++;
        if (tach_fails[i] >= tach_alert_threshold) {
          // TODO: Freak out over a mechanical failure.
        }
      }
      else {
        tach_fails[i] = 0;
      }
    }
    last_tach_check = now;
  }
}


int8_t report_fault_condition(int8_t fault) {
  int8_t ret = 0;
  tec_powered(0, false);
  tec_powered(1, false);
  tec_safety(true);
  ESP_LOGE(TAG, "Homeostate fault: %d\n", ret);
  return ret;
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
* SX1503 callbacks
*******************************************************************************/

void sx1503_callback_fxn(uint8_t pin, uint8_t level) {
  switch (pin) {
    case CIRCUIT_CONF1_PIN:
      homeostasis.conf_sw1_enable_subzero = (0 == level);
      break;
    case CIRCUIT_CONF2_PIN:
      homeostasis.conf_sw2_staged_tec_banks  = (0 == level);
      break;
    default:
      break;
  }
}


/*******************************************************************************
* TEC control
* NOTE: See schematic for calrification.
* TODO: Copy the schematic out of your head.
*
* The hardware will initialize into a safe state, with pull-downs on all pins
*   that drive NPN load switches. This was done to prevent timing and init
*   errors from becoming fires.
* The H-bridge is built in such a way as to air-gap the voltages in the H-bridge
*   from those of the elements controlling it. This makes the hardware safe to
*   use while the high-voltage domain is unpowered. This isolation is probably
*   good into the kilovolt range. But for this application, it only needs to be
*   good for a few hundred. PCB layout is the limiting factor.
* The H-bridge has built-in RC time constants to ensure break-before-make
*   operation of each half of the bridge. This eliminates the need for
*   error-prone firmware managment of each half of the H-bridge for the sake of
*   conflict avoidance, while also preventing transient short-circuits through
*   either leg of the H-bridge during state transitions.
* Since a TEC is a low-inductance load, the MOSFET body diodes were deemed
*   sufficient to handle flyback currents during state transitions. Do not use
*   this machine to drive inductive loads, JiC...
* There is a large relay before the H-bridges that functions as the master power
*   switch to the TEC banks. It only controls the high-voltage supply. The
*   lower operating voltages (12, 5, 3.3) are unaffected by this switch. This
*   was done to prevent logic and program flow mistakes from turning into a
*   situation where the machine is hot but unable to run the pumps or fans
*   without risking the TEC banks adding to the heat.
* Furthermore, the circuit was designed in such a way that the drive level on
*   the pin corresponds to the drive level on that half of the H-bridge. This
*   was done to avoid human error.
*******************************************************************************/

/**
* Enable or disable the TEC safe state. The TECs are considered "saftied" when
*   the relay that provides them high voltage is open.
* This function will refuse to unsafety the TECs if the H-bridge is not off.
*
* @param The ID of the TEC bank in question
* @param Pass true to enable the bank
* @return 0 on success (which includes "no action")
*        -1 on GPIO unreadiness
*        -2 on I/O failure
*        -3 on premature bank enablement
*/
int8_t tec_safety(bool en) {
  int8_t ret = -1;
  if (sx1503.initialized()) {
    // The safety being on is incompatible with the safety relay being driven.
    if (en == sx1503.digitalRead(TEC_HV_ENABLE_PIN)) {
      ret--;
      if ((!en) & (tec_powered(0) | tec_powered(1))) {
        ret--;
      }
      // Set the drive pin HIGH to apply voltage the the H-bridge. LOW to remove it.
      else if (0 == sx1503.digitalWrite(TEC_HV_ENABLE_PIN, !en)) {
        ret = 0;
      }
    }
    else ret = 0;  // TEC safety relay is already in the desired state.
  }
  return ret;
}


/**
* The semantics of this function call are such that reversal is not
*   considered. If you want to enable the bank in reverse, use
*   tec_reversed(bank_id, true); instead.
* Calling this function to enable a bank that is already enabled in reverse will
*   make no changes, and return no error. If you want to forward-bias a bank
*   which is presently running in reverse-bias, use
*   tec_reversed(bank_id, false); instead.
* The TEC array is considered "powered" when the P pin is driven high and the N
*   pin is driven low. This condition will cause the H-bridge to forward-bias
*   the TEC elements according to the color-coding on their wires.
* The TEC array is considered "unpowered" when the states of the P and N pins
*   match, since this condition will not allow current flow through the
*   H-bridge. Technically, the choice to idle the pins high or low is arbitrary,
*   but we will choose to idle them low, so that drive voltage (which is high
*   enough to cause pain and/or fire) doesn't needlessly leave the H-bridge.
* The hardware has it's own safety features that allow TEC bias to be reversed
*   without regard for the current power state.
*
* @param The ID of the TEC bank in question
* @param Pass true to enable the bank
* @return 0 on success (which includes "no action")
*        -1 on illegal bank ID
*        -2 on GPIO unreadiness
*        -3 on I/O failure
*/
int8_t tec_powered(const uint8_t BANK_ID, bool en) {
  int8_t ret = -1;
  if (BANK_ID < 2) {
    ret--;
    if (sx1503.initialized()) {
      const uint16_t GPIO_VAL     = sx1503.getPinValues();
      const uint8_t  BANK_PIN_P   = BANK_PIN_ARRAY[(BANK_ID<<1)+0];
      const uint8_t  BANK_PIN_N   = BANK_PIN_ARRAY[(BANK_ID<<1)+1];
      const bool     STATE_P      = (GPIO_VAL >> BANK_PIN_P) & 0x01;
      const bool     STATE_N      = (GPIO_VAL >> BANK_PIN_N) & 0x01;
      const bool     BANK_ENABLED = STATE_P ^ STATE_N;
      if (BANK_ENABLED ^ en) {
        ret--;  // I/O error is all that can happen from here.
        if (en) {
          // At this point, we know the relevent pin states match (and
          //   shouldn't), but we don't know the polarity.
          // They _should_ be LOW, but mistakes happen. So check for this
          //   condition anyway (see safety notes for clarification).
          if (STATE_N) {
            // This is the off-case.
            // Although (STATE_P == STATE_N == HIGH) is a valid way to achieve
            //   (en == false), doing this will maintain a high voltage on the
            //   TEC supply lines for no benefit, and marginally higher risk of
            //   fire and injury.
            // Set the N pin low to forward bias the bank.
            if (0 == sx1503.digitalWrite(BANK_PIN_N, 0)) {
              ret = 0;
            }
          }
          else {
            // Set the P pin high to forward bias the bank.
            if (0 == sx1503.digitalWrite(BANK_PIN_P, 1)) {
              ret = 0;
            }
          }
        }
        else {
          // At this point, we know the relevent pin states don't match (and
          //   should), but we don't know the polarity.
          // If the bank is presently reverse-biased, set the N pin low to
          //   remove the differential from the bank. Otherwise, it is
          //   forward-biased, and we should set the P pin low to achieve same.
          if (0 == sx1503.digitalWrite((STATE_N ? BANK_PIN_N : BANK_PIN_P), 0)) {
            ret = 0;
          }
        }
      }
      else ret = 0;  // Bank is already in the desired state.
    }
  }
  return ret;
}


/**
* Calling this function will turn on power to the TEC bank, if it wasn't
*   already.
*
* @param BANK_ID The ID of the TEC bank in question
* @param Pass true to enable the bank
* @return 0 on success, -1 on illegal bank ID, -2 on I/O failure
*/
int8_t tec_reversed(const uint8_t BANK_ID, bool rev) {
  int8_t ret = -1;
  if (BANK_ID < 2) {
    ret--;
    if (sx1503.initialized()) {
      const uint16_t GPIO_VAL     = sx1503.getPinValues();
      const uint8_t  BANK_PIN_P   = BANK_PIN_ARRAY[(BANK_ID<<1)+0];
      const uint8_t  BANK_PIN_N   = BANK_PIN_ARRAY[(BANK_ID<<1)+1];
      const bool     STATE_P      = (GPIO_VAL >> BANK_PIN_P) & 0x01;
      const bool     STATE_N      = (GPIO_VAL >> BANK_PIN_N) & 0x01;
      const bool     BANK_ENABLED = STATE_P ^ STATE_N;

      if (!BANK_ENABLED) {
        // At this point, we know the relevent pin states match (and
        //   shouldn't), but we don't know the polarity.
        // They _should_ be LOW, but mistakes happen. So check for this
        //   condition anyway (see safety notes for clarification).
        if (STATE_N) {
          // This is the off-case.
          // Although (STATE_P == STATE_N == HIGH) is a valid way to achieve
          //   (en == false), doing this will maintain a high voltage on the
          //   TEC supply lines for no benefit, and marginally higher risk of
          //   fire and injury.
          // Set the P pin low to reverse-bias the bank, or the N pin to
          //   forward-bias it.
          if (0 == sx1503.digitalWrite((rev ? BANK_PIN_P:BANK_PIN_N), 0)) {
            ret = 0;
          }
        }
        else {
          // Set the N pin high to reverse-bias the bank, or the P pin to
          //   forward-bias it.
          if (0 == sx1503.digitalWrite((rev ? BANK_PIN_N:BANK_PIN_P), 1)) {
            ret = 0;
          }
        }
      }
      else if (rev != STATE_N) {
        // At this point, we know the bank is powered, and that the bias
        //   direction is not what we want it to be. We need to set both pins.
        // The hardware allows us to do this safely in a single I/O operation
        //   using sx1503.setPinValues(new_pin_values); but we will do it one
        //   pin at a time for now.
        if (0 == sx1503.digitalWrite(BANK_PIN_P, !STATE_P)) {
          if (0 == sx1503.digitalWrite(BANK_PIN_N, !STATE_N)) {
            ret = 0;
          }
        }
      }
      else ret = 0;  // Bank is already in the desired state.
    }
  }
  return ret;
}


bool tec_safety() {
  bool ret = true;   // Hardware assurances make this safe to say.
  if (sx1503.initialized()) {
    // If the drive pin is LOW, the relay is off, and thus, the TECs are safetied.
    ret = (0 == sx1503.digitalRead(TEC_HV_ENABLE_PIN));
  }
  return ret;
}


/**
* Is there a voltage differential across the TEC bank (in any direction)?
*
* @param BANK_ID The ID of the TEC bank in question
* @return true if so. False otherwise.
*/
bool tec_powered(const uint8_t BANK_ID) {
  bool ret = false;
  if (BANK_ID < 2) {
    if (sx1503.initialized()) {
      const uint16_t GPIO_VAL     = sx1503.getPinValues();
      const uint8_t  BANK_PIN_P   = BANK_PIN_ARRAY[(BANK_ID<<1)+0];
      const uint8_t  BANK_PIN_N   = BANK_PIN_ARRAY[(BANK_ID<<1)+1];
      const bool     STATE_P      = (GPIO_VAL >> BANK_PIN_P) & 0x01;
      const bool     STATE_N      = (GPIO_VAL >> BANK_PIN_N) & 0x01;
      ret = (STATE_P ^ STATE_N);
    }
  }
  return ret;
}


/**
* Is there a reversed voltage differential across the TEC bank?
* NOTE: tec_reversed() -> tec_powered()
* That is: If the bank is reversed, it is also powered.
*
* @param BANK_ID The ID of the TEC bank in question
* @return true if so. False otherwise.
*/
bool tec_reversed(const uint8_t BANK_ID) {
  bool ret = false;
  if (BANK_ID < 2) {
    if (sx1503.initialized()) {
      const uint16_t GPIO_VAL     = sx1503.getPinValues();
      const uint8_t  BANK_PIN_P   = BANK_PIN_ARRAY[(BANK_ID<<1)+0];
      const uint8_t  BANK_PIN_N   = BANK_PIN_ARRAY[(BANK_ID<<1)+1];
      const bool     STATE_P      = (GPIO_VAL >> BANK_PIN_P) & 0x01;
      const bool     STATE_N      = (GPIO_VAL >> BANK_PIN_N) & 0x01;
      ret = (STATE_P ^ STATE_N) & STATE_N;
    }
  }
  return ret;
}


/*******************************************************************************
* Pump and fan control
*******************************************************************************/
int8_t pump_powered(const uint8_t PUMP_ID, bool en) {
  int8_t ret = -1;
  if (PUMP_ID < 2) {
    ret--;
    if (sx1503.initialized()) {
      const uint8_t PUMP_PIN = (0 == PUMP_ID) ? PUMP0_ENABLE_PIN : PUMP1_ENABLE_PIN;
      if (en != sx1503.digitalRead(PUMP_PIN)) {
        ret--;
        if (0 == sx1503.digitalWrite(PUMP_PIN, en)) {
          ret = 0;
        }
      }
      else ret = 0;
    }
  }
  return ret;
}


bool pump_powered(const uint8_t PUMP_ID) {
  bool ret = false;
  if (PUMP_ID < 2) {
    const uint8_t PUMP_PIN = (0 == PUMP_ID) ? PUMP0_ENABLE_PIN : PUMP1_ENABLE_PIN;
    if (sx1503.initialized()) {
      ret = sx1503.digitalRead(PUMP_PIN);
    }
  }
  return ret;
}



/*******************************************************************************
* Data aggregation and packaging
*******************************************************************************/

static int proc_mqtt_payload(const char* topic, uint8_t* buf, unsigned int len) {
  const char* tok1 = (const char*) topic;
  if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/ble-ping")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 5);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/ble-command")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 2);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/lora-message")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 4);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/lora-command")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 3);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/conf")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 1);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/ota-update")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 2);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }

  return 0;
}


/**
 * @brief event handler for MQTT
 *
 * @param ctx
 * @param event
 * @return esp_err_t
 */
static int mqtt_event_handler(esp_mqtt_event_t* event) {
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  // your_context_t *context = event->context;
  switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
      connected_mqtt = true;
      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit2", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit5", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit4", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit3", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit1", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit0", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/color", 0);
      //msg_id = esp_mqtt_client_subscribe(client, "#", 0);

      //msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
      //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
      break;

    case MQTT_EVENT_DISCONNECTED:
      connected_mqtt = false;
      ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
      break;

    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_PUBLISHED:
      ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_DATA:
      {
        char* safe_topic   = (char*) alloca(event->topic_len + 1);
        uint8_t* safe_data = (uint8_t*) alloca(event->data_len + 1);
        memcpy(safe_topic, event->topic, event->topic_len);
        memcpy(safe_data, event->data, event->data_len);
        *(safe_topic + event->topic_len) = 0;
        *(safe_data + event->data_len)   = 0;
        printf("TOPIC0: %s\n", safe_topic);
        printf("DATA=%.*s\r\n", event->data_len, safe_data);
        proc_mqtt_payload(safe_topic, safe_data, event->data_len);
      }
      break;

    case MQTT_EVENT_ERROR:
      ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
      break;
    default:
      ESP_LOGI(TAG, "MQTT_EVENT Default case");
      break;
  }
  return ESP_OK;
}


void mqtt_send_temperature() {
  //StringBuilder _log;
  //gpsptr->printDebug(&_log);
  if (connected_mqtt) {
    StringBuilder dat;
    //tmp102->issue_value_map(TCode::CBOR, &dat);
    // SensorError err = tmp102->readAsString(0, &dat);
    // if (SensorError::NO_ERROR == err) {
    //   printf("temperature (%s).\n", (const char*) dat.string());
    //   int msg_id = esp_mqtt_client_publish(client, "/theta/radio-relay/temperature", (const char*) dat.string(), dat.length(), 0, 0);
    // }
    // else {
    //   printf("Failed to get temperature (%s).\n", SensorWrapper::errorString(err));
    // }
  }
  //Kernel::log(&_log);
}


/*******************************************************************************
* OTA functions. Based on esp-idf example for native API use.
*******************************************************************************/
//
// static void http_cleanup(esp_http_client_handle_t client) {
//   esp_http_client_close(client);
//   esp_http_client_cleanup(client);
// }
//
//
// static void __attribute__((noreturn)) task_fatal_error() {
//   ESP_LOGE(TAG, "Exiting task due to fatal error...");
//   (void)vTaskDelete(NULL);
//   while (1) {}
// }
//
//
// static void ota_task(void *pvParameter) {
//   esp_err_t err;
//   char ota_write_data[1024 + 1] = { 0 };
//
//   /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
//   esp_ota_handle_t update_handle = 0;
//   const esp_partition_t *update_partition = NULL;
//   ESP_LOGI(TAG, "Starting OTA service thread...");
//
//   const esp_partition_t* configured = esp_ota_get_boot_partition();
//   const esp_partition_t* running = esp_ota_get_running_partition();
//
//     if (configured != running) {
//       ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x", configured->address, running->address);
//       ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
//     }
//     ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)", running->type, running->subtype, running->address);
//
//     /* Wait for the callback to set the CONNECTED_BIT in the event group. */
//     while (!connected_have_ip) {
//       vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
//     ESP_LOGI(TAG, "Checking for firmware update...");
//
//     esp_http_client_config_t config;
//     memset((void*) &config, 0, sizeof(config));
//     config.url = "https://" EXAMPLE_SERVER_URL "/";
//     config.cert_pem = (char *)server_cert_pem_start;
//
//
//     esp_http_client_handle_t client = esp_http_client_init(&config);
//     if (client == NULL) {
//         ESP_LOGE(TAG, "Failed to initialise HTTP connection");
//         task_fatal_error();
//     }
//     err = esp_http_client_open(client, 0);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
//         esp_http_client_cleanup(client);
//         task_fatal_error();
//     }
//     esp_http_client_fetch_headers(client);
//
//     update_partition = esp_ota_get_next_update_partition(NULL);
//     ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x", update_partition->subtype, update_partition->address);
//     assert(update_partition != NULL);
//
//     err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
//         http_cleanup(client);
//         task_fatal_error();
//     }
//     ESP_LOGI(TAG, "esp_ota_begin succeeded");
//
//     int binary_file_length = 0;
//     /*deal with all receive packet*/
//     while (1) {
//       int data_read = esp_http_client_read(client, ota_write_data, sizeof(ota_write_data)-1);
//       if (data_read < 0) {
//         ESP_LOGE(TAG, "Error: SSL data read error");
//         http_cleanup(client);
//         task_fatal_error();
//       }
//       else if (data_read > 0) {
//         err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
//         if (err != ESP_OK) {
//           http_cleanup(client);
//           ESP_LOGE(TAG, "Failed to write OTA image: %s", esp_err_to_name(err));
//           task_fatal_error();
//         }
//         binary_file_length += data_read;
//         ESP_LOGD(TAG, "Written image length %d", binary_file_length);
//       }
//       else if (data_read == 0) {
//         ESP_LOGI(TAG, "Connection closed,all data received");
//         break;
//       }
//     }
//     ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);
//
//     if (esp_ota_end(update_handle) != ESP_OK) {
//         ESP_LOGE(TAG, "esp_ota_end failed!");
//         http_cleanup(client);
//         task_fatal_error();
//     }
//
//     if (esp_partition_check_identity(esp_ota_get_running_partition(), update_partition) == true) {
//         ESP_LOGI(TAG, "The current running firmware is same as the firmware just downloaded");
//         int i = 0;
//         ESP_LOGI(TAG, "When a new firmware is available on the server, press the reset button to download it");
//         while(1) {
//           ESP_LOGI(TAG, "Waiting for a new firmware ... %d", ++i);
//           vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }
//     }
//
//   err = esp_ota_set_boot_partition(update_partition);
//   if (err != ESP_OK) {
//     ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
//     http_cleanup(client);
//     task_fatal_error();
//   }
//   ESP_LOGI(TAG, "Prepare to restart system!");
//   esp_restart();
// }
//
//
// void print_sha256(const uint8_t* image_hash, const char* label) {
//   char hash_print[65];
//   hash_print[64] = 0;
//   for (int i = 0; i < 32; ++i) {
//     sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
//   }
//   ESP_LOGI(TAG, "%s: %s", label, hash_print);
// }


/*******************************************************************************
* Touch callbacks
*******************************************************************************/

static void cb_button(int button, bool pressed) {
  last_interaction = millis();
  if (pressed) {
    //vibrateOn(19);
  }
  //ledOn(LED_B_PIN, 2, (4095 - graph_array_ana_light.value() * 3000));
  uint16_t value = touch->buttonStates();
  last_interaction = millis();
  uApp::appActive()->deliverButtonValue(value);
}


static void cb_slider(int slider, int value) {
  last_interaction = millis();
  //ledOn(LED_R_PIN, 30, (4095 - graph_array_ana_light.value() * 3000));
  uApp::appActive()->deliverSliderValue(value);
}


static void cb_longpress(int button, uint32_t duration) {
  //ledOn(LED_G_PIN, 30, (4095 - graph_array_ana_light.value() * 3000));
}


/*******************************************************************************
* Console callbacks
*******************************************************************************/

int callback_help(StringBuilder* text_return, StringBuilder* args) {
  return console.console_handler_help(text_return, args);
}

int callback_console_tools(StringBuilder* text_return, StringBuilder* args) {
  return console.console_handler_conf(text_return, args);
}

int callback_touch_tools(StringBuilder* text_return, StringBuilder* args) {
  return touch->console_handler(text_return, args);
}

int callback_display_test(StringBuilder* text_return, StringBuilder* args) {
  return display.console_handler(text_return, args);
}

int callback_homeostasis_tool(StringBuilder* text_return, StringBuilder* args) {
  return homeostasis.console_handler(text_return, args);
}

int callback_sx1503_test(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  char* cmd = args->position_trimmed(0);
  // We interdict if the command is something specific to this application.
  if (0 == StringBuilder::strcasecmp(cmd, "reconf")) {
    int8_t ret_local = sx1503.unserialize(sx1503_config, SX1503_SERIALIZE_SIZE);
    text_return->concatf("sx1503 reconf returns %d\n", ret_local);
    ret = 0;
  }
  else ret = sx1503.console_handler(text_return, args);

  return ret;
}

int callback_link_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  char* cmd = args->position_trimmed(0);
  // We interdict if the command is something specific to this application.
  if (0 == StringBuilder::strcasecmp(cmd, "desc")) {
    // Send a description request message.
    KeyValuePair a((uint32_t) millis(), "time_ms");
    a.append((uint32_t) randomUInt32(), "rand");
    int8_t ret_local = m_link->send(&a, true);
    text_return->concatf("Description request send() returns ID %u\n", ret_local);
    ret = 0;
  }
  else ret = m_link->console_handler(text_return, args);

  return ret;
}


int callback_spi_debug(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (0 < args->count()) {
    int bus_id = args->position_as_int(0);
    args->drop_position(0);
    switch (bus_id) {
      case 1:   ret = spi_bus.console_handler(text_return, args);  break;
      default:
        text_return->concatf("Unsupported bus: %d\n", bus_id);
        break;
    }
  }
  return ret;
}


int callback_i2c_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (0 < args->count()) {
    int bus_id = args->position_as_int(0);
    args->drop_position(0);
    switch (bus_id) {
      case 0:   ret = i2c0.console_handler(text_return, args);  break;
      case 1:   ret = i2c1.console_handler(text_return, args);  break;
      default:
        text_return->concatf("Unsupported bus: %d\n", bus_id);
        break;
    }
  }
  return ret;
}


int callback_active_app(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd = args->position_trimmed(0);

  if (0 == StringBuilder::strcasecmp(cmd, "stopwatch")) {
    uApp::appActive()->printStopwatch(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "poll")) {
    text_return->concatf("%s refresh() returns %d\n", uApp::appActive()->getAppIDString(), uApp::appActive()->refresh());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "set")) {
    if (1 < args->count()) {
      int arg = args->position_as_int(1);
      switch (arg) {
        case ((int) AppID::APP_SELECT):
        case ((int) AppID::TOUCH_TEST):
        case ((int) AppID::CONFIGURATOR):
        case ((int) AppID::COMMS_TEST):
        case ((int) AppID::CALORIMETER):
        case ((int) AppID::HOT_STANDBY):
        case ((int) AppID::SUSPEND):
          uApp::setAppActive((AppID) arg);
          text_return->concatf("App changed: %s->%s\n", uApp::appActive()->previousApp()->getAppIDString(), uApp::appActive()->getAppIDString());
          break;
        default:
          text_return->concatf("Unsupported app: %d\n", arg);
          return -1;
      }
    }
    else text_return->concat("Need an app ID.\n");
  }
  else text_return->concatf("Current app:  %s\n", uApp::appActive()->getAppIDString());
  return ret;
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
        case 0:   baro.printDebug(text_return);             break;
        case 1:   temp_sensor_m.printDebug(text_return);    break;
        case 2:   temp_sensor_0.printDebug(text_return);    break;
        case 3:   temp_sensor_1.printDebug(text_return);    break;
        case 4:   temp_sensor_2.printDebug(text_return);    break;
        case 5:   temp_sensor_3.printDebug(text_return);    break;
        default:
          break;
      }
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    if (1 < args->count()) {
      int8_t ret_local = 0;
      int arg1 = args->position_as_int(1);
      switch (arg1) {
        case 0:   ret = baro.init();             break;
        case 1:   ret = temp_sensor_m.init();    break;
        case 2:   ret = temp_sensor_0.init();    break;
        case 3:   ret = temp_sensor_1.init();    break;
        case 4:   ret = temp_sensor_2.init();    break;
        case 5:   ret = temp_sensor_3.init();    break;
        default:
          break;
      }
      text_return->concatf("Sensor init returned %d.\n", ret_local);
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
      case 1:   sel_sen = &temperature_filter_m;    break;
      case 2:   sel_sen = &temperature_filter_0;    break;
      case 3:   sel_sen = &temperature_filter_1;    break;
      case 4:   sel_sen = &temperature_filter_2;    break;
      case 5:   sel_sen = &temperature_filter_3;    break;
      //case 6:   sel_sen = &fan_speed_0;             break;
      //case 7:   sel_sen = &fan_speed_1;             break;
      //case 8:   sel_sen = &fan_speed_2;             break;
      //case 8:   sel_sen = &pump_speed_0;            break;
      //case 10:  sel_sen = &pump_speed_1;            break;
      case 0:   sel_sen = &air_temp_filter;         break;    // TODO: ugly
      case 11:  sel_sen = &pressure_filter;         break;    // TODO: ugly
      case 12:  sel_sen = &humidity_filter;         break;    // TODO: ugly
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
        text_return->concatf("analogWrite() returns %d\n", analogWrite(FAN_PWM_PIN, arg1));
      }
      else {
        text_return->concat("Fan speed must be in the range [0, 1].\n");
      }
    }
    text_return->concatf("Fan speed set at %.2f%%\n", fan_pwm_ratio);
  }
  else {
    text_return->concatf("Fan0:\t%d RPM\n", (int) fan_speed_0.value());
    text_return->concatf("Fan1:\t%d RPM\n", (int) fan_speed_1.value());
    text_return->concatf("Fan2:\t%d RPM\n", (int) fan_speed_2.value());
  }
  return ret;
}


int callback_pump_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  bool print_pump_speed = false;
  char* loop = args->position_trimmed(0);
  char* pump_str = "";
  uint8_t pump_id = 255;
  SensorFilter<uint16_t>* rpm_filter = nullptr;
  if (0 == StringBuilder::strcasecmp(loop, "i")) {
    pump_id = 0;
    pump_str = "Internal";
    rpm_filter = &pump_speed_0;
  }
  else if (0 == StringBuilder::strcasecmp(loop, "e")) {
    pump_id = 1;
    pump_str = "External";
    rpm_filter = &pump_speed_1;
  }

  if (pump_id < 2) {
    if (1 < args->count()) {
      char* cmd  = args->position_trimmed(1);
      if (0 == StringBuilder::strcasecmp(cmd, "on")) {
        text_return->concatf("pump_powered(%s, true) returns %d.\n", pump_str, pump_powered(pump_id, true));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "off")) {
        text_return->concatf("pump_powered(%s, false) returns %d.\n", pump_str, pump_powered(pump_id, false));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "filter")) {
        rpm_filter->printFilter(text_return);
      }
      else print_pump_speed = true;
    }
    else {
      print_pump_speed = true;
    }

    if (print_pump_speed) {
      text_return->concatf("Pump (%s): %3sabled  %4d RPM\n", pump_str, (pump_powered(pump_id) ? "En":"Dis"), rpm_filter->value()*60);
    }
  }
  else {
    text_return->concat("You must specify the pump for either the internal (i) or external (e) loop.\n");
    ret = -1;
  }

  return ret;
}


int callback_tec_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  if (0 < args->count()) {
    uint8_t bank = (uint8_t) args->position_as_int(0);
    if (1 < args->count()) {
      char*   cmd  = args->position_trimmed(1);
      if (0 == StringBuilder::strcasecmp(cmd, "on")) {
        text_return->concatf("tec_powered(%u, true) returns %d.\n", bank, tec_powered(bank, true));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "off")) {
        text_return->concatf("tec_powered(%u, false) returns %d.\n", bank, tec_powered(bank, false));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "reverse")) {
        bool crev = (2 < args->count()) ? (0 != args->position_as_int(2)) : !tec_reversed(bank);
        text_return->concatf("tec_reversed(%u, %s) returns %d.\n", bank, (crev?"true":"false"), tec_reversed(bank, crev));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "safety")) {
        if (2 < args->count()) {
          bool safety = (0 != args->position_as_int(2));
          text_return->concatf("tec_safety(%s) returns %d.\n", (safety?"true":"false"), tec_safety(safety));
        }
        else {
          text_return->concatf("TEC safety is %s.\n", (tec_safety()?"on":"off"));
        }
      }
      else ret = -1;
    }
    else {
      text_return->concatf("TEC safety is %s.\n", (tec_safety()?"on":"off"));
      text_return->concatf("TEC Bank%u: %3sabled  %s\n", bank, (tec_powered(bank) ? "En":"Dis"), tec_reversed(bank) ? "(Reversed)":"");
    }
  }
  else {
    text_return->concat("You must specify the bank (0 or 1).\n");
    ret = -1;
  }

  return ret;
}


#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Support functions                                                            *
*******************************************************************************/

int8_t test_homeostasis_program() {
  int8_t ret = 0;
  HomeostasisFSM prior_state = homeostate;
  if (HomeostasisFSM::PROG_RUNNING == homeostate) {
    bool orderly_return_to_idle = false;
    for (uint8_t i = 0; i < 6; i++) {    // Cycle through each temperature sensor, checking the values.
      SensorFilter<float>* filter = getTemperatureFilter(i);
      float current_temp = filter->value();

      switch (i) {
        case TMP_SENSE_IDX_H_BRIDGE:
          ret = (current_temp <= homeostasis.temperature_max_h_bridge) ? 0 : -1;
          break;

        case TMP_SENSE_IDX_EXT_AFF:
          break;
        case TMP_SENSE_IDX_EXT_EFF:
          break;

        case TMP_SENSE_IDX_INT_AFF:
        case TMP_SENSE_IDX_INT_EFF:
          if ((current_temp < homeostasis.temperature_min_internal_loop) | (current_temp > homeostasis.temperature_max_internal_loop)) {
            ret = -1;
          }
          else {
            ret = 0;
          }
          break;
        case TMP_SENSE_IDX_AIR:
          ret = (current_temp >= homeostasis.temperature_max_air) ? 0 : -1;
          break;
      }
      if (0 != ret) {
        homeostate = HomeostasisFSM::FAULT;
        report_fault_condition(ret);
        return ret;
      }
    }

    if (orderly_return_to_idle) {
      // TODO: Shut down the TECs, and (optionally) the AUX outlet.
      tec_powered(0, false);
      tec_powered(1, false);
      homeostate = HomeostasisFSM::IDLE;
    }
  }

  if (prior_state != homeostate) {
    ESP_LOGI(TAG, "Homeostate moved (%s --> %s)\n", HomeostasisParams::fsmToStr(prior_state), HomeostasisParams::fsmToStr(homeostate));
  }
  return ret;
}


/*******************************************************************************
* Main function and threads                                                    *
*******************************************************************************/

void manuvr_task(void* pvParameter) {
  //esp_mqtt_client_config_t mqtt_cfg;
  //memset((void*) &mqtt_cfg, 0, sizeof(mqtt_cfg));
  //mqtt_cfg.uri  = "mqtt://" EXAMPLE_SERVER_URL;
  //mqtt_cfg.port = 1883;
  //mqtt_cfg.event_handle = mqtt_event_handler;
  //mqtt_cfg.user_context = (void *)your_context

  //client = esp_mqtt_client_init(&mqtt_cfg);
  //esp_mqtt_client_start(client);

  while (1) {
    bool should_sleep = true;
    while (0 < spi_bus.poll()) {
      should_sleep = false;
    }
    while (0 < spi_bus.service_callback_queue()) {
      should_sleep = false;
    }

    timeoutCheckVibLED();

    if (touch->devFound()) {          touch->poll();           }
    if (sx1503.devFound()) {          sx1503.poll();           }
    if (baro.devFound()) {
      if (0 < baro.poll()) {
        humidity_filter.feedFilter(baro.hum());
        air_temp_filter.feedFilter(baro.temp());
        pressure_filter.feedFilter(baro.pres());
      }
    }
    if (temp_sensor_m.devFound()) {
      temp_sensor_m.poll();
      if (temp_sensor_m.dataReady()) {
        temperature_filter_m.feedFilter(temp_sensor_m.temperature());
      }
    }
    if (temp_sensor_0.devFound()) {
      temp_sensor_0.poll();
      if (temp_sensor_0.dataReady()) {
        temperature_filter_0.feedFilter(temp_sensor_0.temperature());
      }
    }
    if (temp_sensor_1.devFound()) {
      temp_sensor_1.poll();
      if (temp_sensor_1.dataReady()) {
        temperature_filter_1.feedFilter(temp_sensor_1.temperature());
      }
    }
    if (temp_sensor_2.devFound()) {
      temp_sensor_2.poll();
      if (temp_sensor_2.dataReady()) {
        temperature_filter_2.feedFilter(temp_sensor_2.temperature());
      }
    }
    if (temp_sensor_3.devFound()) {
      temp_sensor_3.poll();
      if (temp_sensor_3.dataReady()) {
        temperature_filter_3.feedFilter(temp_sensor_3.temperature());
      }
    }

    update_tach_values();
    test_homeostasis_program();

    uint32_t millis_now = millis();
    if ((last_interaction + 100000) <= millis_now) {
      // After 100 seconds, time-out the display.
      if (&app_standby != uApp::appActive()) {
        uApp::setAppActive(AppID::HOT_STANDBY);
      }
    }
    uApp::appActive()->refresh();

    if (0 < console_uart.poll()) {
      should_sleep = false;
    }

    platform.yieldThread();
  }
}


/*******************************************************************************
* Setup function
*******************************************************************************/

void app_main() {
  //StringBuilder boot_log;
  /*
  * The platform object is created on the stack, but takes no action upon
  *   construction. The first thing that should be done is to call the preinit
  *   function to setup the defaults of the platform.
  */
  platform_init();
  boot_time = millis();

    // uint8_t sha_256[32] = { 0 };
    // esp_partition_t partition;
    //
    // // get sha256 digest for the partition table
    // partition.address   = ESP_PARTITION_TABLE_OFFSET;
    // partition.size      = ESP_PARTITION_TABLE_MAX_LEN;
    // partition.type      = ESP_PARTITION_TYPE_DATA;
    // esp_partition_get_sha256(&partition, sha_256);
    // print_sha256(sha_256, "SHA-256 for the partition table: ");
    //
    // // get sha256 digest for bootloader
    // partition.address   = ESP_BOOTLOADER_OFFSET;
    // partition.size      = ESP_PARTITION_TABLE_OFFSET;
    // partition.type      = ESP_PARTITION_TYPE_APP;
    // esp_partition_get_sha256(&partition, sha_256);
    // print_sha256(sha_256, "SHA-256 for bootloader: ");
    //
    // // get sha256 digest for running partition
    // //esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    // print_sha256(sha_256, "SHA-256 for current firmware: ");

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // OTA app partition table has a smaller NVS partition size than the non-OTA
      // partition table. This size mismatch may cause NVS initialization to fail.
      // If this happens, we erase NVS partition and initialize NVS again.
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

  // Top-level pin responsibilities.
  pinMode(FAN0_TACH_PIN,  GPIOMode::INPUT_PULLUP);
  pinMode(FAN1_TACH_PIN,  GPIOMode::INPUT_PULLUP);
  pinMode(FAN2_TACH_PIN,  GPIOMode::INPUT_PULLUP);
  pinMode(PUMP0_TACH_PIN, GPIOMode::INPUT_PULLUP);
  pinMode(PUMP1_TACH_PIN, GPIOMode::INPUT_PULLUP);
  pinMode(FAN_PWM_PIN,      GPIOMode::ANALOG_OUT);
  analogWrite(FAN_PWM_PIN, 0.5f);

  /* Start the console UART and attach it to the console. */
  console_uart.readCallback(&console);    // Attach the UART to console...
  console.setOutputTarget(&console_uart); // ...and console to UART.
  console.setTXTerminator(LineTerm::CRLF); // Best setting for "idf.py monitor"
  console.setRXTerminator(LineTerm::LF);   // Best setting for "idf.py monitor"
  console.setPromptString(console_prompt_str);
  console.emitPrompt(true);
  console.localEcho(true);
  console.printHelpOnFail(true);

  console.defineCommand("help",        '?',  ParsingConsole::tcodes_str_1, "Prints help to console.", "[<specific command>]", 0, callback_help);
  console.defineCommand("console",     '\0', ParsingConsole::tcodes_str_3, "Console conf.", "[echo|prompt|force|rxterm|txterm]", 0, callback_console_tools);
  platform.configureConsole(&console);
  console.defineCommand("sx",          '\0', ParsingConsole::tcodes_str_4, "SX1503 test", "", 0, callback_sx1503_test);
  console.defineCommand("touch",       '\0', ParsingConsole::tcodes_str_4, "SX8634 tools", "", 0, callback_touch_tools);
  console.defineCommand("link",        'l',  ParsingConsole::tcodes_str_4, "Linked device tools.", "", 0, callback_link_tools);
  console.defineCommand("spi",         '\0', ParsingConsole::tcodes_str_3, "SPI debug.", "", 1, callback_spi_debug);
  console.defineCommand("i2c",         '\0', ParsingConsole::tcodes_str_4, "I2C tools", "i2c <bus> <action> [addr]", 1, callback_i2c_tools);
  console.defineCommand("homeostasis", 'h',  ParsingConsole::tcodes_str_4, "Homeostasis parameters", "", 0, callback_homeostasis_tool);

  console.defineCommand("disp",        'd',  ParsingConsole::tcodes_str_4, "Display test", "", 0, callback_display_test);
  console.defineCommand("app",         'a',  ParsingConsole::tcodes_str_4, "Select active application.", "", 0, callback_active_app);
  console.defineCommand("sensor",      's',  ParsingConsole::tcodes_str_4, "Sensor tools", "", 0, callback_sensor_tools);
  console.defineCommand("filter",      '\0', ParsingConsole::tcodes_str_3, "Sensor filter info.", "", 0, callback_sensor_filter_info);
  console.defineCommand("fan",         'f',  ParsingConsole::tcodes_str_3, "Fan tools", "", 0, callback_fan_tools);
  console.defineCommand("pump",        'p',  ParsingConsole::tcodes_str_3, "Pump tools", "", 0, callback_pump_tools);
  console.defineCommand("tec",         't',  ParsingConsole::tcodes_str_3, "TEC tools", "", 0, callback_tec_tools);

  console.init();

  StringBuilder ptc("HeatPump ");
  ptc.concat(TEST_PROG_VERSION);
  ptc.concat("\t Build date " __DATE__ " " __TIME__ "\n");
  console.printToLog(&ptc);

  /* Allocate memory for the filters. */
  if (0 != init_sensor_memory()) {
    console_uart.write("Failed to allocate memory for sensors.\n");
  }

  spi_bus.init();
  i2c0.init();
  i2c1.init();

  touch = new SX8634(&_touch_opts);

  // Assign i2c0 to devices attached to it.
  touch->assignBusInstance(&i2c0);
  sx1503.assignBusInstance(&i2c0);
  temp_sensor_m.assignBusInstance(&i2c0);

  // Assign i2c1 to devices attached to it.
  baro.assignBusInstance(&i2c1);
  temp_sensor_0.assignBusInstance(&i2c1);
  temp_sensor_1.assignBusInstance(&i2c1);
  temp_sensor_2.assignBusInstance(&i2c1);
  temp_sensor_3.assignBusInstance(&i2c1);

  // Callback setup for various drivers...
  touch->setButtonFxn(cb_button);
  touch->setSliderFxn(cb_slider);
  touch->setLongpressFxn(cb_longpress);

  sx1503.attachInterrupt(CIRCUIT_CONF1_PIN, sx1503_callback_fxn, IRQCondition::CHANGE);
  sx1503.attachInterrupt(CIRCUIT_CONF2_PIN, sx1503_callback_fxn, IRQCondition::CHANGE);

  // Enable interrupts for pins.
  setPinFxn(FAN0_TACH_PIN,  IRQCondition::FALLING, isr_fan0_tach_fxn);
  setPinFxn(FAN1_TACH_PIN,  IRQCondition::FALLING, isr_fan1_tach_fxn);
  setPinFxn(FAN2_TACH_PIN,  IRQCondition::FALLING, isr_fan2_tach_fxn);
  //setPinFxn(PUMP0_TACH_PIN, IRQCondition::FALLING, isr_pump0_tach_fxn);
  //setPinFxn(PUMP1_TACH_PIN, IRQCondition::FALLING, isr_pump1_tach_fxn);

  // Spawn worker threads, note the time, and terminate thread.
  xTaskCreate(manuvr_task, "_manuvr", 32768, NULL, (tskIDLE_PRIORITY), NULL);
  //xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
  config_time = millis();
}

#ifdef __cplusplus
}
#endif
