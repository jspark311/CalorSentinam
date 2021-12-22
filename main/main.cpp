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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp32/rom/ets_sys.h"
#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#include "esp_task_wdt.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_eth.h"

//#include "esp_ota_ops.h"
//#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "esp_http_client.h"
#include "mqtt_client.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sys.h"

#ifdef __cplusplus
}
#endif


/* OTA parameters that probably ought to be imparted at provisioning. */
#define EXAMPLE_SERVER_URL "ian-app.home.joshianlindsay.com"
//extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
//extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");


#define TACH_IDX_FAN0     0
#define TACH_IDX_FAN1     1
#define TACH_IDX_FAN2     2
#define TACH_IDX_PUMP0    3
#define TACH_IDX_PUMP1    4


/*******************************************************************************
* Globals
*******************************************************************************/

// Our thermal flow model needs some parameters of the machine.
const float MASS_OF_EXCHANGER        = 250.0;     // Grams
const float MASS_OF_INTERNAL_COOLANT = 100.0;     // Grams

esp_mqtt_client_handle_t client = nullptr;

static bool connected_have_ip   = false;
static bool connected_mqtt      = false;

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
  0x0f, 0x00,  // 0-7 are outputs into a 5V domain. 8-11 are inputs. 12-15 are LEDs.
  0x0F, 0x00,  // All inputs have pull-ups.
  0x00, 0x00,  // No pull-downs.
  0xF0, 0xFF,  // Interrupts unmasked for all inputs.
  0x00, 0x00,  // All inputs are sensitive to both edges.
  0xff, 0x00,  // All inputs are sensitive to both edges.
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

SX1503 sx1503(SX1503_IRQ_PIN, 255);   // GPIO on the power control board.
SSD13xx display(&disp_opts);
BME280I2C baro(baro_settings);
TMP102 temp_sensor_m(0x49, 255);
TMP102 temp_sensor_0(0x48, 255);
TMP102 temp_sensor_1(0x49, 255);
TMP102 temp_sensor_2(0x4A, 255);
TMP102 temp_sensor_3(0x4B, 255);

/* Profiling data */
StopWatch stopwatch_main_loop_time;

/* Cheeseball async support stuff. */
uint32_t boot_time         = 0;      // millis() at boot.
uint32_t config_time       = 0;      // millis() at end of setup().
uint32_t last_interaction  = 0;      // millis() when the user last interacted.

volatile static uint32_t tach_counters[5] = {0, 0, 0, 0, 0};
volatile static uint32_t last_tach_check  = 0;

// static uint16_t fan0_target_rpm   = 200;
// static uint16_t fan1_target_rpm   = 200;
// static uint16_t fan2_target_rpm   = 200;
// static uint8_t fan0_pwm_percent   = 100;
// static uint8_t fan1_pwm_percent   = 100;
// static uint8_t fan2_pwm_percent   = 100;

static float    fan_pwm_ratio     = 0.0;




/*******************************************************************************
* ISRs
*******************************************************************************/
void IRAM_ATTR isr_fan0_tach_fxn() {     tach_counters[TACH_IDX_FAN0]++;    }
void IRAM_ATTR isr_fan1_tach_fxn() {     tach_counters[TACH_IDX_FAN1]++;    }
void IRAM_ATTR isr_fan2_tach_fxn() {     tach_counters[TACH_IDX_FAN2]++;    }
void IRAM_ATTR isr_pump0_tach_fxn() {    tach_counters[TACH_IDX_PUMP0]++;   }
void IRAM_ATTR isr_pump1_tach_fxn() {    tach_counters[TACH_IDX_PUMP1]++;   }


void update_tach_values() {
  const uint32_t now = millis();
  const uint32_t ms_delta_tach = wrap_accounted_delta(last_tach_check, now);
  // Every second or so, update the tach values.
  if (ms_delta_tach >= 1000) {
    for (uint i = 0; i < 5; i++) {  // We have five tachometer values to update.
      uint32_t tmp_tach_count  = tach_counters[i];
      tach_counters[i] -= tmp_tach_count;     // Decrement by the amount we noted.
      // We want RPMs. We get two counts per full revolution, and we are dealing
      //   with sample periods in terms of milliseconds. Thus....
      uint16_t tmp_tach_rpm = (uint16_t) (tmp_tach_count * (30000.0 / (float) ms_delta_tach));
      SensorFilter<uint16_t>* filter = nullptr;
      switch (i) {
        case TACH_IDX_FAN0:     filter = &fan_speed_0;     break;
        case TACH_IDX_FAN1:     filter = &fan_speed_1;     break;
        case TACH_IDX_FAN2:     filter = &fan_speed_2;     break;
        case TACH_IDX_PUMP0:    filter = &pump_speed_0;    break;
        case TACH_IDX_PUMP1:    filter = &pump_speed_1;    break;
        default:  return;
      }
      filter->feedFilter(tmp_tach_rpm);
    }
    last_tach_check = now;
  }

  // const uint32_t FAN_RPM_HYSTERESIS   = 60;
  // const uint32_t FAN_PERCENTAGE_DELTA = 2;
  // if (ms_delta_tach >= 1000) {
  //   if (fan_speed_0.dirty()) {
  //     const uint16_t FAN_THRESHOLD_SPEED_UP   = fan0_target_rpm + FAN_RPM_HYSTERESIS;
  //     const uint16_t FAN_THRESHOLD_SPEED_DOWN = fan0_target_rpm - FAN_RPM_HYSTERESIS;   // TODO: magnitude check.
  //     const uint16_t LAST_RPM_VALUE           = fan_speed_0.value();
  //     if ((0 == LAST_RPM_VALUE) && (fan0_pwm_percent >= 100)) {   // Test for fan failure.
  //     }
  //     else if (LAST_RPM_VALUE < FAN_THRESHOLD_SPEED_UP) {    fan0_pwm_percent += FAN_PERCENTAGE_DELTA;   }
  //     else if (LAST_RPM_VALUE > FAN_THRESHOLD_SPEED_DOWN) {  fan0_pwm_percent -= FAN_PERCENTAGE_DELTA;   }
  //   }
  //   if (fan_speed_1.dirty()) {
  //     const uint16_t FAN_THRESHOLD_SPEED_UP   = fan1_target_rpm + FAN_RPM_HYSTERESIS;
  //     const uint16_t FAN_THRESHOLD_SPEED_DOWN = fan1_target_rpm - FAN_RPM_HYSTERESIS;   // TODO: magnitude check.
  //     const uint16_t LAST_RPM_VALUE           = fan_speed_1.value();
  //     if ((0 == LAST_RPM_VALUE) && (fan1_pwm_percent >= 100)) {   // Test for fan failure.
  //     }
  //     else if (LAST_RPM_VALUE < FAN_THRESHOLD_SPEED_UP) {    fan1_pwm_percent += FAN_PERCENTAGE_DELTA;   }
  //     else if (LAST_RPM_VALUE > FAN_THRESHOLD_SPEED_DOWN) {  fan1_pwm_percent -= FAN_PERCENTAGE_DELTA;   }
  //   }
  //   if (fan_speed_2.dirty()) {
  //     const uint16_t FAN_THRESHOLD_SPEED_UP   = fan2_target_rpm + FAN_RPM_HYSTERESIS;
  //     const uint16_t FAN_THRESHOLD_SPEED_DOWN = fan2_target_rpm - FAN_RPM_HYSTERESIS;   // TODO: magnitude check.
  //     const uint16_t LAST_RPM_VALUE           = fan_speed_2.value();
  //     if ((0 == LAST_RPM_VALUE) && (fan2_pwm_percent >= 100)) {   // Test for fan failure.
  //     }
  //     else if (LAST_RPM_VALUE < FAN_THRESHOLD_SPEED_UP) {    fan2_pwm_percent += FAN_PERCENTAGE_DELTA;   }
  //     else if (LAST_RPM_VALUE > FAN_THRESHOLD_SPEED_DOWN) {  fan2_pwm_percent -= FAN_PERCENTAGE_DELTA;   }
  //   }
  // }
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

int callback_sx1503_test(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd = args->position_trimmed(0);
  uint8_t arg0 = args->position_as_int(1);
  uint8_t arg1 = args->position_as_int(2);

  if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    text_return->concatf("sx1503.init() returns %d\n", sx1503.init());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    text_return->concatf("sx1503.reset() returns %d\n", sx1503.reset());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "mode")) {
    switch (args->count()) {
      case 3:
        switch ((GPIOMode) arg1) {
          case GPIOMode::INPUT:
          case GPIOMode::OUTPUT:
          case GPIOMode::INPUT_PULLUP:
          case GPIOMode::INPUT_PULLDOWN:
          case GPIOMode::OUTPUT_OD:
          case GPIOMode::BIDIR_OD:
          case GPIOMode::BIDIR_OD_PULLUP:
          case GPIOMode::ANALOG_OUT:
          case GPIOMode::ANALOG_IN:
            text_return->concatf("pinMode(%u, %s) Returns %d.\n", arg0, getPinModeStr((GPIOMode) arg1), sx1503.gpioMode(arg0, (GPIOMode)arg1));
            break;
          default:
            text_return->concat("Invalid GPIO mode.\n");
            break;
        }
        break;
      default:
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::INPUT,           getPinModeStr(GPIOMode::INPUT));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::OUTPUT,          getPinModeStr(GPIOMode::OUTPUT));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::INPUT_PULLUP,    getPinModeStr(GPIOMode::INPUT_PULLUP));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::INPUT_PULLDOWN,  getPinModeStr(GPIOMode::INPUT_PULLDOWN));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::OUTPUT_OD,       getPinModeStr(GPIOMode::OUTPUT_OD));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::BIDIR_OD,        getPinModeStr(GPIOMode::BIDIR_OD));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::BIDIR_OD_PULLUP, getPinModeStr(GPIOMode::BIDIR_OD_PULLUP));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::ANALOG_OUT,      getPinModeStr(GPIOMode::ANALOG_OUT));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::ANALOG_IN,       getPinModeStr(GPIOMode::ANALOG_IN));
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "val")) {
    text_return->concatf("GPIO %d ", arg0);
    switch (args->count()) {
      case 3:
        {
          int8_t ret0 = sx1503.digitalWrite(arg0, (0 != arg1));
          text_return->concatf("set to %s. Returns %d.\n", (0 != arg1) ? "high" : "low", ret0);
        }
        break;
      default:
        {
          int8_t ret0 = sx1503.digitalRead(arg0);
          text_return->concatf("reads %s.\n", ret0 ? "high" : "low");
        }
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "regs")) {
    sx1503.printRegs(text_return);
  }
  else {
    sx1503.printDebug(text_return);
  }

  return ret;
}


int callback_display_test(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd = args->position_trimmed(0);
  uint32_t millis_0 = millis();
  uint32_t millis_1 = millis_0;

  if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    text_return->concatf("display.init() returns %d\n", display.init());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    text_return->concatf("display.reset() returns %d\n", display.reset());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "commit")) {
    text_return->concatf("commitFrameBuffer() returns %d\n", display.commitFrameBuffer());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "fill")) {
    uint16_t color = (uint16_t) args->position_as_int(1);
    display.fill(color);
    text_return->concatf("display.fill(%u)\n", color);
  }
  else {
    display.printDebug(text_return);
    #ifdef SPI_HAS_TRANSFER_ASYNC
      text_return->concat("SPI_HAS_TRANSFER_ASYNC is set.\n");
    #else
      text_return->concat("SPI_HAS_TRANSFER_ASYNC is NOT set.\n");
    #endif
  }

  millis_1 = millis();
  text_return->concatf("Display update took %ums\n", millis_1-millis_0);
  return ret;
}


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


int callback_touch_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  char* cmd = args->position_trimmed(0);
  if (0 < args->count()) {
    ret = 0;
    if (0 == StringBuilder::strcasecmp(cmd, "info")) {
      touch->printDebug(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "poll")) {
      text_return->concatf("SX8634 poll() returns %d.\n", touch->poll());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
      text_return->concatf("SX8634 init() returns %d.\n", touch->init());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
      text_return->concatf("SX8634 reset() returns %d.\n", touch->reset());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "ping")) {
      text_return->concatf("SX8634 ping() returns %d\n", touch->ping());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "mode")) {
      if (1 < args->count()) {
        int mode_int = args->position_as_int(1);
        text_return->concatf("SX8634 setMode(%s) returns %d.\n", SX8634::getModeStr((SX8634OpMode) mode_int), touch->setMode((SX8634OpMode) mode_int));
      }
      text_return->concatf("SX8634 mode set to %s.\n", SX8634::getModeStr(touch->operationalMode()));
    }
  }
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
  else console.printHelp(text_return);
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
        if (0 == sx1503.digitalWrite(PUMP0_ENABLE_PIN, 1)) {
          text_return->concatf("Pump %s enabled.\n", loop);
        }
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "off")) {
        if (0 == sx1503.digitalWrite(PUMP0_ENABLE_PIN, 0)) {
          text_return->concatf("Pump %s disabled.\n", loop);
        }
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "filter")) {
        pump_speed_0.printFilter(text_return);
      }
      else print_pump0_speed = true;
    }
    else {
      print_pump0_speed = true;
    }
  }
  else if (0 == StringBuilder::strcasecmp(loop, "e")) {
    if (1 < args->count()) {
      if (0 == StringBuilder::strcasecmp(cmd, "on")) {
        if (0 == sx1503.digitalWrite(PUMP1_ENABLE_PIN, 1)) {
          text_return->concatf("Pump %s enabled.\n", loop);
        }
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "off")) {
        if (0 == sx1503.digitalWrite(PUMP1_ENABLE_PIN, 0)) {
          text_return->concatf("Pump %s disabled.\n", loop);
        }
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
  if (print_pump0_speed) text_return->concatf("Pump0 (Internal): %3sabled  %4d RPM\n", (sx1503.digitalRead(PUMP0_ENABLE_PIN) ? "En":"Dis"), (int) pump_speed_0.value()*60);
  if (print_pump1_speed) text_return->concatf("Pump1 (External): %3sabled  %4d RPM\n", (sx1503.digitalRead(PUMP1_ENABLE_PIN) ? "En":"Dis"), (int) pump_speed_1.value()*60);
  return ret;
}


int callback_tec_tools(StringBuilder* text_return, StringBuilder* args) {
  const uint16_t GPIO_VAL = sx1503.getPinValues();
  const bool BANK_0_P       = (GPIO_VAL >> TEC_BANK0_P_PIN) & 0x01;
  const bool BANK_1_P       = (GPIO_VAL >> TEC_BANK1_P_PIN) & 0x01;
  const bool BANK_0_N       = (GPIO_VAL >> TEC_BANK0_N_PIN) & 0x01;
  const bool BANK_1_N       = (GPIO_VAL >> TEC_BANK1_N_PIN) & 0x01;
  const bool BANK_0_ENABLED = BANK_0_P ^ BANK_0_N;
  const bool BANK_1_ENABLED = BANK_1_P ^ BANK_1_N;
  const bool BANK_0_REVERSE = BANK_0_ENABLED & BANK_0_N;
  const bool BANK_1_REVERSE = BANK_1_ENABLED & BANK_1_N;
  bool print_bank_0 = false;
  bool print_bank_1 = false;
  int  ret = 0;

  int   bank = args->position_as_int(0);
  char* cmd  = args->position_trimmed(1);
  if (0 < args->count()) {
    uint8_t bank_pin   = TEC_BANK1_P_PIN;
    uint8_t bank_pin_n = TEC_BANK1_N_PIN;
    switch (bank) {
      case 0:
        bank_pin   = TEC_BANK0_P_PIN;
        bank_pin_n = TEC_BANK0_N_PIN;
      case 1:
        if (1 < args->count()) {
          if (0 == StringBuilder::strcasecmp(cmd, "on")) {
            if (0 == sx1503.setPinValues((GPIO_VAL & ~(1 << bank_pin_n)) | (1 << bank_pin))) {
              text_return->concatf("TEC bank %d enabled.\n", bank);
            }
          }
          else if (0 == StringBuilder::strcasecmp(cmd, "off")) {
            if (0 == sx1503.setPinValues(GPIO_VAL & ~((1 << bank_pin_n) | (1 << bank_pin)))) {
              text_return->concatf("TEC bank %d disabled.\n", bank);
            }
          }
          else if (0 == StringBuilder::strcasecmp(cmd, "reverse")) {
            if (0 == sx1503.setPinValues(GPIO_VAL & ~((1 << bank_pin) | (1 << bank_pin_n)))) {
              text_return->concatf("TEC bank %d enabled in reverse.\n", bank);
            }
          }
        }
        else {
          print_bank_0 = true;
          print_bank_1 = true;
        }
        break;
      default:
        text_return->concat("You must specify the bank (0 or 1).\n");
        break;
    }
  }

  if (print_bank_0) text_return->concatf("TEC Bank0: %3sabled  %s\n", (BANK_0_ENABLED ? "En":"Dis"), BANK_0_REVERSE ? "(Reversed)":"");
  if (print_bank_1) text_return->concatf("TEC Bank1: %3sabled  %s\n", (BANK_1_ENABLED ? "En":"Dis"), BANK_1_REVERSE ? "(Reversed)":"");
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


#ifdef __cplusplus
extern "C" {
#endif


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
    if (temp_sensor_m.devFound()) {   temp_sensor_m.poll();    }
    if (temp_sensor_0.devFound()) {   temp_sensor_0.poll();    }
    if (temp_sensor_1.devFound()) {   temp_sensor_1.poll();    }
    if (temp_sensor_2.devFound()) {   temp_sensor_2.poll();    }
    if (temp_sensor_3.devFound()) {   temp_sensor_3.poll();    }

    uint32_t millis_now = millis();
    if ((last_interaction + 100000) <= millis_now) {
      // After 100 seconds, time-out the display.
      if (&app_standby != uApp::appActive()) {
        //uApp::setAppActive(AppID::HOT_STANDBY);
      }
    }
    update_tach_values();
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

  console.defineCommand("help",        '?', ParsingConsole::tcodes_str_1, "Prints help to console.", "", 0, callback_help);
  platform.configureConsole(&console);
  console.defineCommand("sx",          '\0', ParsingConsole::tcodes_str_4, "SX1503 test", "", 0, callback_sx1503_test);
  console.defineCommand("disp",        'd', ParsingConsole::tcodes_str_4, "Display test", "", 0, callback_display_test);
  console.defineCommand("app",         'a', ParsingConsole::tcodes_str_4, "Select active application.", "", 0, callback_active_app);
  console.defineCommand("touch",       '\0', ParsingConsole::tcodes_str_4, "SX8634 tools", "", 0, callback_touch_tools);
  console.defineCommand("sensor",      's',  ParsingConsole::tcodes_str_4, "Sensor tools", "", 0, callback_sensor_tools);
  console.defineCommand("filter",      '\0', ParsingConsole::tcodes_str_3, "Sensor filter info.", "", 0, callback_sensor_filter_info);
  console.defineCommand("fan",         'f',  ParsingConsole::tcodes_str_3, "Fan tools", "", 0, callback_fan_tools);
  console.defineCommand("pump",        'p',  ParsingConsole::tcodes_str_3, "Pump tools", "", 0, callback_pump_tools);
  console.defineCommand("tec",         't',  ParsingConsole::tcodes_str_3, "TEC tools", "", 0, callback_tec_tools);
  console.defineCommand("i2c",         '\0', ParsingConsole::tcodes_str_4, "I2C tools", "Usage: i2c <bus> <action> [addr]", 1, callback_i2c_tools);
  console.defineCommand("console",     '\0', ParsingConsole::tcodes_str_3, "Console conf.", "[echo|prompt|force|rxterm|txterm]", 0, callback_console_tools);
  console.defineCommand("link",        'l', ParsingConsole::tcodes_str_4, "Linked device tools.", "", 0, callback_link_tools);
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
  touch->setButtonFxn(cb_button);
  touch->setSliderFxn(cb_slider);
  touch->setLongpressFxn(cb_longpress);

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

  // Do any device configuration that needs to be imparted from the top level.
  if (0 != sx1503.unserialize(sx1503_config, SX1503_SERIALIZE_SIZE)) {
    console_uart.write("Failed to unserialize SX1503 config.\n");
  }

  // Enable interrupts for pins.
  setPinFxn(FAN0_TACH_PIN,  IRQCondition::FALLING, isr_fan0_tach_fxn);
  //setPinFxn(FAN1_TACH_PIN,  IRQCondition::FALLING, isr_fan1_tach_fxn);
  //setPinFxn(FAN2_TACH_PIN,  IRQCondition::FALLING, isr_fan2_tach_fxn);
  //setPinFxn(PUMP0_TACH_PIN, IRQCondition::FALLING, isr_pump0_tach_fxn);
  //setPinFxn(PUMP1_TACH_PIN, IRQCondition::FALLING, isr_pump1_tach_fxn);

  touch->reset();   // TODO: Why must this be done twice?

  // Spawn worker threads, note the time, and terminate thread.
  xTaskCreate(manuvr_task, "_manuvr", 32768, NULL, (tskIDLE_PRIORITY), NULL);
  //xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
  config_time = millis();
}

#ifdef __cplusplus
}
#endif
