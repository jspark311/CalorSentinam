#include <CppPotpourri.h>
#include <SensorFilter.h>
#include <SPIAdapter.h>
#include <I2CAdapter.h>

#include "HeatPump.h"
#include "uApp.h"


#define UAPP_BOOT_FLAG_INIT_DISPLAY         0x00000001
#define UAPP_BOOT_FLAG_INIT_SX1503          0x00000002
#define UAPP_BOOT_FLAG_INIT_TEMP_MOSFET     0x00000004
#define UAPP_BOOT_FLAG_INIT_BARO            0x00000008
#define UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_0  0x00000010  // Liquid temp sensor.
#define UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_1  0x00000020  // Liquid temp sensor.
#define UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_2  0x00000040  // Liquid temp sensor.
#define UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_3  0x00000080  // Liquid temp sensor.
#define UAPP_BOOT_FLAG_INIT_TOUCH           0x00000100
#define UAPP_BOOT_FLAG_INIT_WIRELESS        0x00000200
#define UAPP_BOOT_FLAG_INIT_LOOP_CONFIG     0x00000400  // Switches on the power board.
#define UAPP_BOOT_FLAG_INIT_STORAGE         0x00000800
#define UAPP_BOOT_FLAG_INIT_CONF_LOADED     0x00001000

#define UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE   0x80000000

typedef struct {
  const char* const str;
  const uint32_t flag_mask;
} UAppInitPoint;

// Items on the init list should be in order of desired initialization.
const UAppInitPoint INIT_LIST[] = {
  {"Display",          UAPP_BOOT_FLAG_INIT_DISPLAY         },
  {"SX1503",           UAPP_BOOT_FLAG_INIT_SX1503          },
  {"MOSFET Temp",      UAPP_BOOT_FLAG_INIT_TEMP_MOSFET     },
  {"Barometer",        UAPP_BOOT_FLAG_INIT_BARO            },
  {"Liquid Temp 0",    UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_0  },
  {"Liquid Temp 1",    UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_1  },
  {"Liquid Temp 2",    UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_2  },
  {"Liquid Temp 3",    UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_3  },
  {"Touchpad",         UAPP_BOOT_FLAG_INIT_TOUCH           },
  {"Wireless comms",   UAPP_BOOT_FLAG_INIT_WIRELESS        },
  {"Loop Config",      UAPP_BOOT_FLAG_INIT_LOOP_CONFIG     },
  {"NV Storage",       UAPP_BOOT_FLAG_INIT_STORAGE         },
  {"Conf Load",        UAPP_BOOT_FLAG_INIT_CONF_LOADED     },
  {"Boot Complete",    UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE   }
};

uint16_t serial_timeout = 0;



uAppBoot::uAppBoot() : uApp("Boot", (Image*) &display) {}

uAppBoot::~uAppBoot() {}


/*******************************************************************************
* Lifecycle callbacks from the super class
*******************************************************************************/
/**
* Called by superclass on activation of lifecycle. This function should prepare
*   the class as if it were freshly instantiated. Redraw will not happen until
*   the tick following this function returning 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppBoot::_lc_on_preinit() {
  int8_t ret = 1;
  display.init(&spi_bus);
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppBoot::_lc_on_active() {
  int8_t ret = 0;
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppBoot::_lc_on_teardown() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppBoot::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppBoot::_process_user_input() {
  int8_t ret = 1;

  if (_slider_current != _slider_pending) {
    _slider_current = _slider_pending;
  }
  if (_buttons_current != _buttons_pending) {
    if (_init_done_flags.value(UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE)) {
      // Interpret a button press as exiting APP_BOOT.
      uApp::setAppActive(AppID::APP_SELECT);
      ret = -1;
    }
    _buttons_current = _buttons_pending;
  }
  return ret;
}



/*
* Draws the app.
*/
void uAppBoot::_redraw_window() {
  const uint8_t INIT_LIST_LEN = sizeof(INIT_LIST) / sizeof(UAppInitPoint);
  const float   INIT_LIST_PERCENT = 100.0 / (float) INIT_LIST_LEN;
  const uint8_t BLOCK_WIDTH  = (96/INIT_LIST_LEN);  //INIT_LIST_PERCENT * 96;
  const uint8_t BLOCK_HEIGHT = 11;
  bool     continue_looping = display.enabled();   // Don't loop if no display.
  uint8_t  i = 0;

  while (continue_looping & (i < INIT_LIST_LEN)) {
    if (!_init_sent_flags.value(INIT_LIST[i].flag_mask)) {
      // This item has not seen an init call succeed.
      bool ret_local = false;
      switch (INIT_LIST[i].flag_mask) {
        case UAPP_BOOT_FLAG_INIT_DISPLAY:
          FB->fill(BLACK);
          FB->setTextColor(WHITE, BLACK);
          FB->setCursor(0, 0);
          FB->setTextSize(1);
          FB->writeString("Calor Sentinam");
          FB->setTextSize(0);
          FB->fillRect(0, 11, BLOCK_WIDTH*INIT_LIST_LEN, BLOCK_HEIGHT, YELLOW);
          FB->fillRect(0, 24, BLOCK_WIDTH*INIT_LIST_LEN, BLOCK_HEIGHT, YELLOW);
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_SX1503:
          if (sx1503.getAdapter()->busIdle()) {
            ret_local = (0 == sx1503.init());
          }
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_MOSFET:
          if (temp_sensor_m.getAdapter()->busIdle()) {
            ret_local = (0 == temp_sensor_m.init());
          }
          break;
        case UAPP_BOOT_FLAG_INIT_BARO:
          if (baro.getAdapter()->busIdle()) {
            ret_local = (0 == baro.init());
          }
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_0:
          if (temp_sensor_0.getAdapter()->busIdle()) {
            ret_local = (0 == temp_sensor_0.init());
          }
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_1:
          if (temp_sensor_1.getAdapter()->busIdle()) {
            ret_local = (0 == temp_sensor_1.init());
          }
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_2:
          if (temp_sensor_2.getAdapter()->busIdle()) {
            ret_local = (0 == temp_sensor_2.init());
          }
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_3:
          if (temp_sensor_3.getAdapter()->busIdle()) {
            ret_local = (0 == temp_sensor_3.init());
          }
          break;
        case UAPP_BOOT_FLAG_INIT_TOUCH:
          //if (touch->getAdapter()->busIdle()) {
            ret_local = (0 == touch->reset());
          //}
          break;
        case UAPP_BOOT_FLAG_INIT_WIRELESS:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_LOOP_CONFIG:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_STORAGE:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_CONF_LOADED:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE:
          ret_local = true;
          break;
        default:  return;  // TODO: Failure
      }
      if (ret_local) {
        // If calling the init sequence succeeded, mark it as having been done.
        //draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, (0 == i), false, (INIT_LIST_PERCENT*(i+1)));
        display.fillRect(BLOCK_WIDTH*i, 11, BLOCK_WIDTH, BLOCK_HEIGHT, BLUE);
        _init_sent_flags.set(INIT_LIST[i].flag_mask);
        continue_looping = false;
        _last_init_sent = millis();
      }
    }
    else {
      display.fillRect(BLOCK_WIDTH*i, 11, BLOCK_WIDTH, BLOCK_HEIGHT, GREEN);
    }

    if (!_init_done_flags.value(INIT_LIST[i].flag_mask)) {
      // This item has not seen an init succeed.
      bool ret_local = false;
      switch (INIT_LIST[i].flag_mask) {
        case UAPP_BOOT_FLAG_INIT_DISPLAY:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_SX1503:
          ret_local = sx1503.initialized();
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_MOSFET:
          ret_local = temp_sensor_m.initialized();
          break;
        case UAPP_BOOT_FLAG_INIT_BARO:
          ret_local = baro.initialized();
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_0:
          ret_local = temp_sensor_0.initialized();
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_1:
          ret_local = temp_sensor_1.initialized();
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_2:
          ret_local = temp_sensor_2.initialized();
          break;
        case UAPP_BOOT_FLAG_INIT_TEMP_XCHANGE_3:
          ret_local = temp_sensor_3.initialized();
          break;
        case UAPP_BOOT_FLAG_INIT_TOUCH:
          if (touch->devFound()) {
            if (touch->deviceReady()) {
              touch->setLongpress(800, 0);   // 800ms is a long-press. No rep.
              touch->setMode(SX8634OpMode::ACTIVE);
              ret_local = true;
              //ret_local = (0 == touch->setMode(SX8634OpMode::ACTIVE));
            }
          }
          break;
        case UAPP_BOOT_FLAG_INIT_WIRELESS:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_LOOP_CONFIG:
          if (sx1503.initialized()) {
            homeostasis.conf_sw1_enable_subzero    = !sx1503.digitalRead(CIRCUIT_CONF1_PIN);
            homeostasis.conf_sw2_staged_tec_banks  = !sx1503.digitalRead(CIRCUIT_CONF2_PIN);
            ret_local = true;
          }
          break;
        case UAPP_BOOT_FLAG_INIT_STORAGE:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_CONF_LOADED:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE:
          ret_local = (_init_sent_flags.raw == (_init_done_flags.raw | UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE));
          break;
        default:  return;  // TODO: Failure
      }
      if (ret_local) {
        // If init succeeded, mark it as complete.
        _init_done_flags.set(INIT_LIST[i].flag_mask);
        //draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, (0 == i), false, (INIT_LIST_PERCENT*(i+1)));
        display.fillRect(BLOCK_WIDTH*i, 24, BLOCK_WIDTH, BLOCK_HEIGHT, BLUE);
        const uint UNFILLED_STR_LEN = strlen(INIT_LIST[i].str);
        char filled_list_str[17] = {0, };
        for (uint8_t stri = 0; stri < sizeof(filled_list_str); stri++) {
          filled_list_str[stri] = (stri < UNFILLED_STR_LEN) ? *(INIT_LIST[i].str + stri) : ' ';
        }
        FB->setTextColor(WHITE, BLACK);
        FB->setCursor(0, 36);
        FB->writeString(filled_list_str);
        continue_looping = false;
      }
    }
    else {
      display.fillRect(BLOCK_WIDTH*i, 24, BLOCK_WIDTH, BLOCK_HEIGHT, GREEN);
    }
    i++;
  }

  if (wrap_accounted_delta(_last_init_sent, millis()) >= UAPP_BOOT_INIT_TIMEOUT) {
    for (uint8_t n = 0; n < INIT_LIST_LEN; n++) {
      if (!_init_sent_flags.value(INIT_LIST[n].flag_mask)) {
        _init_sent_flags.set(INIT_LIST[n].flag_mask);
        display.fillRect(BLOCK_WIDTH*n, 11, BLOCK_WIDTH, BLOCK_HEIGHT, RED);
      }
      if (!_init_done_flags.value(INIT_LIST[n].flag_mask)) {
        _init_done_flags.set(INIT_LIST[n].flag_mask);
        display.fillRect(BLOCK_WIDTH*n, 24, BLOCK_WIDTH, BLOCK_HEIGHT, RED);
      }
    }
  }
  return;
}
