#include <CppPotpourri.h>
#include "uApp.h"
#include "HeatPump.h"


/* UI object declarations */
uAppBoot app_boot;
uAppRoot app_root;
uAppTouchTest app_touch_test;
uAppConfigurator app_config;
uAppComms app_comms;
uAppCalorimeter app_calorimeter;
uAppStandby app_standby;


/*******************************************************************************
* Statics
*******************************************************************************/
volatile static uApp* previous_app_ptr = &app_boot;
volatile static uApp* drawn_app_ptr    = &app_boot;
volatile static uApp* active_app_ptr   = &app_boot;
volatile static uApp* pending_app_ptr  = &app_boot;

uApp* uApp::appActive() {      return (uApp*) active_app_ptr;     }
uApp* uApp::drawnApp() {       return (uApp*) drawn_app_ptr;      }
uApp* uApp::previousApp() {    return (uApp*) previous_app_ptr;   }


void  uApp::setAppActive(AppID new_app) {
  switch (new_app) {
    case AppID::APP_BOOT:       pending_app_ptr = &app_boot;           break;
    case AppID::APP_SELECT:     pending_app_ptr = &app_root;           break;
    case AppID::TOUCH_TEST:     pending_app_ptr = &app_touch_test;     break;
    case AppID::CONFIGURATOR:   pending_app_ptr = &app_config;         break;
    case AppID::COMMS_TEST:     pending_app_ptr = &app_comms;          break;
    case AppID::CALORIMETER:    pending_app_ptr = &app_calorimeter;    break;
    case AppID::HOT_STANDBY:    pending_app_ptr = &app_standby;        break;
    case AppID::SUSPEND:        pending_app_ptr = &app_standby;        break;
  }
}

/*******************************************************************************
* Base class functions
*******************************************************************************/


// TODO: This is the nexus point for an idea that is still full of logical holes,
//   overly-complicated premises, and code written under duress.
int8_t uApp::refresh() {
  int8_t ret = 0;
  uint32_t now = millis();
  if (millis_since(_last_draw) >= APP_POLLING_PERIOD) {
    _stopwatch.markStart();
    switch (_lc_current_position()) {
      case AppLifecycle::UNINITIALIZED:
        // Adds a one-tick delay so the superclass can march the state forward in
        //   a predictable manner.
        _uapp_clear_flag(UAPP_FLAG_IS_ACTIVE);
        ret = 1;
        break;
      case AppLifecycle::PREINIT:
        drawn_app_ptr = active_app_ptr;
        _buttons_current = _buttons_pending;  // TODO: Not sure if this is universally desirable.
        ret = _lc_on_preinit();
        display.commitFrameBuffer();
        break;
      case AppLifecycle::ACTIVE:
        if (_uapp_flag(UAPP_FLAG_IS_ACTIVE)) {
          switch (_process_user_input()) {
            case -1:
              ret = 1;
              if (pending_app_ptr == active_app_ptr) {
                // If the implementing class didn't set a new active app, and yet
                //   is asking for us for teardown, go back to the prior uApp.
                pending_app_ptr = previous_app_ptr;
              }
              break;
            case 1:
              _redraw_window();
              display.commitFrameBuffer();
              break;
            case 0:
            default:
              break;
          }
          if (pending_app_ptr != active_app_ptr) {
            ret = 1;
          }
        }
        else {
          ret = _lc_on_active();
          _uapp_set_flag(UAPP_FLAG_IS_ACTIVE);
          display.commitFrameBuffer();
        }
        _last_draw = now;
        break;
      case AppLifecycle::TEARDOWN:
        ret = _lc_on_teardown();
        break;
      case AppLifecycle::INACTIVE:
        ret = _lc_on_inactive();
        if (1 == ret) {
          // Relinquish the active app slot and allow the next tick to service
          //   the next application.
          previous_app_ptr = this;
          active_app_ptr = pending_app_ptr;
        }
        break;
    }

    if (1 == ret) {
      _lc_increment_position();
    }
    _stopwatch.markStop();
  }
  return ret;
}



/*
* Draws the basics of the UI.
* 96x64
*/
void uApp::redraw_app_window(const char* title, uint8_t pages, uint8_t active_page) {
  display.fill(0);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(0xFFFF);
  display.writeString(title);
  //display.drawLine(0, 10, display.x()-1, 10, 0xFFFF);
  //display.drawLine(0, display.y()-1, display.x()-1, display.y()-1, 0xFFFF);
  //display.drawLine(0, 10, 0, display.y()-1, 0xFFFF);
  //display.drawLine(display.x()-1, 10, display.x()-1, display.y()-1, 0xFFFF);
  //display.drawRect(0, 14, display.x()-1, display.y()-15, 0xFFFF);
  display.drawFastHLine(0, 9, display.x(), 0xFFFF);
}


/**
* Increment the Lifecycle state machine and ping the notification function in
*   the extending class.
*/
void uApp::_lc_increment_position() {
  uint32_t fsm_pos = (_flags & UAPP_FLAG_LIFECYC_SM_MASK);
  uint32_t nu_fsm_pos = (4 <= fsm_pos) ? 0 : (fsm_pos + 1);
  _flags = (_flags & ~UAPP_FLAG_LIFECYC_SM_MASK) | nu_fsm_pos;
}
