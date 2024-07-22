#include "uApp.h"
#include "HeatPump.h"


uAppTouchTest::uAppTouchTest() : uApp("Touch Test", (Image*) &display) {}

uAppTouchTest::~uAppTouchTest() {}



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
int8_t uAppTouchTest::_lc_on_preinit() {
  int8_t ret = 1;
  redraw_app_window();
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppTouchTest::_lc_on_active() {
  int8_t ret = 0;
  FB->setTextSize(0);
  FB->setTextColor(0xFFFF, 0);
  FB->setCursor(0, 12);
  FB->writeString("Mode");
  FB->setCursor(0, 28);
  FB->writeString("Slider");
  _redraw_window();
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppTouchTest::_lc_on_teardown() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppTouchTest::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppTouchTest::_process_user_input() {
  int8_t ret = 0;
  if (_slider_current != _slider_pending) {
    _slider_current = _slider_pending;
    ret = 1;
  }
  if (_buttons_current != _buttons_pending) {
    ret = 1;
    uint16_t diff = _buttons_current ^ _buttons_pending;
    if (_buttons_pending & 0x0001) {   // Interpret a cancel press at full slider right as a return to APP_SELECT.
      if (_slider_current == 0) {
        uApp::setAppActive(AppID::APP_SELECT);
        ret = -1;
      }
    }
    _buttons_current = _buttons_pending;
  }
  return ret;
}


/*
* Draws the touch test app.
*/
void uAppTouchTest::_redraw_window() {
  uint16_t sval = 60 - _slider_current;
  //FB->fillRect(0, 45, 28, 8, 0);
  FB->setTextSize(0);
  FB->setTextColor(0x001F, 0x0000);
  FB->setCursor(0, 20);
  FB->writeString(touch->getModeStr(touch->operationalMode()));
  FB->setCursor(0, 36);
  StringBuilder output;
  output.concatf("%02u", sval);
  FB->writeString(&output);

  uint8_t slider_pix = 2 + (((1+sval) / 61.0f) * (FB->x()-5));
  FB->fillRect(0, 55, FB->x()-1, 7, 0x0000);
  FB->drawRoundRect(0, 54, FB->x(), 9, 3, 0xFFFF);
  FB->fillRect(slider_pix-1, 55, 3, 7, 0xF800);

  const uint8_t x_coords[] = {37, 57, 77, 77, 57, 37};
  const uint8_t y_coords[] = {14, 14, 14, 34, 34, 34};
  for (uint8_t i = 0; i < 6; i++) {
    if (touch->buttonPressed(i)) {
      FB->fillRoundRect(x_coords[i], y_coords[i], 18, 18, 4, 0xFFFF);
    }
    else {
      FB->fillRect(x_coords[i], y_coords[i], 18, 18, 0);
      FB->drawRoundRect(x_coords[i], y_coords[i], 18, 18, 4, 0x07E0);
    }
  }
  FB->setCursor(0, 44);
  FB->fillRect(0, 44, 28, 8, 0x0000);
  FB->setTextColor(0xF800, 0x0000);
  output.clear();
  output.concatf("%02x", _buttons_pending);
  FB->writeString(&output);
}
