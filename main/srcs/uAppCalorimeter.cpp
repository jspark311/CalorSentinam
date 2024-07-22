#include "uApp.h"
#include "HeatPump.h"


void redraw_calorimeter_window();


uAppCalorimeter::uAppCalorimeter() : uApp("Calorimeter", (Image*) &display) {}

uAppCalorimeter::~uAppCalorimeter() {}



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
int8_t uAppCalorimeter::_lc_on_preinit() {
  int8_t ret = 1;
  FB->fill(BLACK);
  redraw_app_window();
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppCalorimeter::_lc_on_active() {
  int8_t ret = 0;
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppCalorimeter::_lc_on_teardown() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppCalorimeter::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppCalorimeter::_process_user_input() {
  int8_t ret = 1;
  if (_slider_current != _slider_pending) {
    FB->fill(BLACK);
    FB->setTextSize(0);
    FB->setCursor(0, 0);
    if (_slider_pending <= 7) {
      if (_render_lock_range()) {
        FB->setTextColor(0x03E0, BLACK);
        FB->writeString("Pressure (Pa)");
      }
      else {
        FB->setTextColor(0x3EE3, BLACK);
        FB->writeString("RelH%");
        FB->setTextColor(WHITE, BLACK);
        FB->writeString(" / ");
        FB->setTextColor(0x83D0, BLACK);
        FB->writeString("Temp  ");
      }
    }
    else if (_slider_pending <= 15) {
      FB->writeString("Unimplemented");
    }
    else if (_slider_pending <= 22) {
      FB->writeString("Unimplemented");
    }
    else if (_slider_pending <= 30) {
      FB->writeString("Unimplemented");
    }
    else if (_slider_pending <= 37) {
      FB->writeString("Unimplemented");
    }
    else if (_slider_pending <= 45) {
      redraw_app_window();
    }
    else if (_slider_pending <= 52) {
    }
    else {
      //FB->fillRect(0, 11, FB->width()-1, FB->y()-12, BLACK);
    }
    _slider_current = _slider_pending;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    if (diff & 0x0001) {   // Interpret a cancel press as a return to APP_SELECT.
      uApp::setAppActive(AppID::APP_SELECT);
      ret = -1;
    }
    if (diff & 0x0002) {   // Cluttered display toggle.
      if (_buttons_pending & 0x0002) {
        _cluttered_display(!_cluttered_display());
      }
    }
    if (diff & 0x0008) {   // Text of actual value.
      if (_buttons_pending & 0x0008) {
        _render_text_value(!_render_text_value());
      }
    }
    if (diff & 0x0010) {   // Button 5
      if (_buttons_pending & 0x0010) {
        _render_lock_range(!_render_lock_range());
      }
    }
    _buttons_current = _buttons_pending;
  }
  return ret;
}



/*
* Draws the calorimeter app.
*/
void uAppCalorimeter::_redraw_window() {
  StringBuilder tmp_val_str;
  if (_slider_current <= 7) {
    // Baro
    if (_render_lock_range()) {
      if (humidity_filter.dirty()) {
        float altitude  = baro.Altitude(baro.pres());
        float dew_point = baro.DewPoint(baro.temp(), baro.hum());
        float sea_level = baro.EquivalentSeaLevelPressure(altitude, baro.temp(), baro.pres());
        draw_graph_obj(
          0, 10, 96, 36, 0x03E0,
          true, _cluttered_display(), _render_text_value(),
          &pressure_filter
        );
        FB->setTextSize(0);
        FB->setCursor(0, 47);
        FB->setTextColor(WHITE, BLACK);
        FB->writeString("Alt: ");
        FB->setTextColor(GREEN, BLACK);
        tmp_val_str.clear();
        tmp_val_str.concatf("%.3f ft", altitude);
        FB->writeString(&tmp_val_str);
        FB->setTextColor(WHITE, BLACK);
        FB->writeString("Dew Point: ");
        FB->setTextColor(0x03E0, BLACK);
        tmp_val_str.clear();
        tmp_val_str.concatf("%.2fC", dew_point);
        FB->writeString(&tmp_val_str);
      }
    }
    else {
      if (air_temp_filter.dirty()) {
        draw_graph_obj(
          0, 10, 96, 37, 0x83D0, 0x3EE3,
          true, _cluttered_display(), _render_text_value(),
          &air_temp_filter, &humidity_filter
        );
      }
    }
  }
  else if (_slider_current <= 15) {
  }
  else if (_slider_current <= 22) {
  }
  else if (_slider_current <= 30) {
  }
  else if (_slider_current <= 37) {
  }
  else if (_slider_current <= 45) {
  }
  else if (_slider_current <= 52) {
  }
  else {
  }
}
