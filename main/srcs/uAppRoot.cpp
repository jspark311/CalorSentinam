#include "uApp.h"
#include "HeatPump.h"


uAppRoot::uAppRoot() : uApp("Root menu", (Image*) &display) {}

uAppRoot::~uAppRoot() {}


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
int8_t uAppRoot::_lc_on_preinit() {
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
int8_t uAppRoot::_lc_on_active() {
  int8_t ret = 0;
  _redraw_window();
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppRoot::_lc_on_teardown() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppRoot::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppRoot::_process_user_input() {
  int8_t ret = 1;

  if (_slider_current != _slider_pending) {
    _slider_current = _slider_pending;
    FB->fill(0);  // Wipe the screen on slider change.
    ret = 1;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    ret = 1;
    switch (modal_ctrl) {
      case 0:   // No active modal
        if (diff & 0x0001) {   // Interpret a cancel press as a command to enter standby.
          uApp::setAppActive(AppID::HOT_STANDBY);
          ret = -1;
        }
        if (diff & 0x0004) {   // Button 3
          if (_buttons_pending & 0x0004) {   // Interpret an ok press as an app selection.
            uApp::setAppActive(app_page);
            ret = -1;
          }
        }
        break;

      case 1:   // Temperature modal
        break;
      case 2:   // Pump/fan modal
        if (diff & 0x0001) {   // CANCEL: Fans off
          if (_buttons_pending & 0x0001) {
            fan_pwm_ratio = 0.0f;
            analogWrite(FAN_PWM_PIN, fan_pwm_ratio);
          }
        }
        if (diff & 0x0002) {   // UP:     Increase fan speed
          if (_buttons_pending & 0x0002) {
            fan_pwm_ratio = strict_min(fan_pwm_ratio+0.1, 1.0);
            analogWrite(FAN_PWM_PIN, fan_pwm_ratio);
          }
        }
        if (diff & 0x0004) {   // ACCEPT: Fans on
          if (_buttons_pending & 0x0004) {
            fan_pwm_ratio = 0.5f;
            analogWrite(FAN_PWM_PIN, fan_pwm_ratio);
          }
        }
        if (diff & 0x0008) {   // RIGHT:  Pump(I) toggle
          if (_buttons_pending & 0x0008) {
            pump_powered(0, !pump_powered(0));
          }
        }
        if (diff & 0x0010) {   // DOWN:   Decrease fan speed
          if (_buttons_pending & 0x0010) {
            fan_pwm_ratio = strict_max(fan_pwm_ratio-0.1, 0.0);
            analogWrite(FAN_PWM_PIN, fan_pwm_ratio);
          }
        }
        if (diff & 0x0020) {   // LEFT:   Pump(E) toggle
          if (_buttons_pending & 0x0020) {
            pump_powered(1, !pump_powered(1));
          }
        }
        break;
      case 3:   // TEC modal
        if (diff & 0x0001) {   // CANCEL: Disarm TECs
          if (_buttons_pending & 0x0001) {
            tec_safety(true);
          }
        }
        if (diff & 0x0002) {   // UP:     Power on bank0
          if (_buttons_pending & 0x0002) {
            tec_powered(0, !tec_powered(0));
          }
        }
        if (diff & 0x0004) {   // ACCEPT: Arm TECs
          if (_buttons_pending & 0x0004) {
            tec_safety(false);
          }
        }
        if (diff & 0x0008) {   // RIGHT:  Power on bank1
          if (_buttons_pending & 0x0008) {
            tec_powered(1, !tec_powered(1));
          }
        }
        if (diff & 0x0010) {   // DOWN:   Reverse on bank0
          if (_buttons_pending & 0x0010) {
            tec_reversed(0, !tec_reversed(0));
          }
        }
        if (diff & 0x0020) {   // LEFT:   Reverse on bank1
          if (_buttons_pending & 0x0020) {
            tec_reversed(1, !tec_reversed(1));
          }
        }
        break;
      default:
        break;
    }
    _buttons_current = _buttons_pending;
  }
  return ret;
}



/*
* Draws the app.
*/
void uAppRoot::_redraw_window() {
  StringBuilder output;
  FB->setCursor(0, 0);
  FB->setTextColor(WHITE, BLACK);
  if (_slider_current <= 7) {
    FB->writeString("Fxn: ");
    FB->setTextColor(MAGENTA, BLACK);
    FB->writeString("Touch Diag  ");
    app_page = AppID::TOUCH_TEST;
    modal_ctrl = 0;
  }
  else if (_slider_current <= 15) {
    FB->writeString("Fxn: ");
    FB->setTextColor(MAGENTA, BLACK);
    FB->writeString("Settings    ");
    app_page = AppID::CONFIGURATOR;
    modal_ctrl = 0;
  }
  else if (_slider_current <= 22) {
    FB->writeString("Fxn: ");
    FB->setTextColor(MAGENTA, BLACK);
    FB->writeString("Comms       ");
    app_page = AppID::COMMS_TEST;
    modal_ctrl = 0;
  }
  else if (_slider_current <= 30) {
    render_button_icon(ICON_THERMO, 0, 11, 0xFFFF);
    FB->writeString("Fxn: ");
    FB->setTextColor(MAGENTA, BLACK);
    FB->writeString("Calorimeter ");
    app_page = AppID::CALORIMETER;
    modal_ctrl = 0;
  }
  else if (_slider_current <= 37) {
    FB->writeString("Temperatures");
    FB->setCursor(0, 12);
    FB->writeString("H-bridge: ");
    if (temp_sensor_m.initialized()) {
      output.concatf("%.1fC\n", (double) temperature_filter_m.value());
      FB->writeString(&output);
      output.clear();
    }
    else {
      FB->setTextColor(RED, BLACK);
      FB->writeString("N/A\n");
      FB->setTextColor(WHITE, BLACK);
    }

    FB->writeString("0:        ");
    if (temp_sensor_0.initialized()) {
      output.concatf("%.1fC\n", (double) temperature_filter_0.value());
      FB->writeString(&output);
      output.clear();
    }
    else {
      FB->setTextColor(RED, BLACK);
      FB->writeString("N/A\n");
      FB->setTextColor(WHITE, BLACK);
    }

    FB->writeString("1:        ");
    if (temp_sensor_1.initialized()) {
      output.concatf("%.1fC\n", (double) temperature_filter_1.value());
      FB->writeString(&output);
      output.clear();
    }
    else {
      FB->setTextColor(RED, BLACK);
      FB->writeString("N/A\n");
      FB->setTextColor(WHITE, BLACK);
    }

    FB->writeString("2:        ");
    if (temp_sensor_2.initialized()) {
      output.concatf("%.1fC\n", (double) temperature_filter_2.value());
      FB->writeString(&output);
      output.clear();
    }
    else {
      FB->setTextColor(RED, BLACK);
      FB->writeString("N/A\n");
      FB->setTextColor(WHITE, BLACK);
    }

    FB->writeString("3:        ");
    if (temp_sensor_3.initialized()) {
      output.concatf("%.1fC\n", (double) temperature_filter_3.value());
      FB->writeString(&output);
      output.clear();
    }
    else {
      FB->setTextColor(RED, BLACK);
      FB->writeString("N/A\n");
      FB->setTextColor(WHITE, BLACK);
    }

    FB->writeString("Air:      ");
    if (baro.initialized()) {
      output.concatf("%.2fC\n", (double) air_temp_filter.value());
      FB->writeString(&output);
      output.clear();
    }
    else {
      FB->setTextColor(RED, BLACK);
      FB->writeString("N/A\n");
      FB->setTextColor(WHITE, BLACK);
    }
    modal_ctrl = 1;
  }

  else if (_slider_current <= 45) {
    FB->setTextColor(WHITE, BLACK);
    FB->writeString("Fans:\n");
    FB->setTextColor(CYAN, BLACK);
    uint8_t drive_percentage = (uint8_t) (fan_pwm_ratio * 100);
    output.concatf("0: %3u%% %4d RPM\n", drive_percentage, (int) fan_speed_0.value());
    output.concatf("1: %3u%% %4d RPM\n", drive_percentage, (int) fan_speed_1.value());
    output.concatf("2: %3u%% %4d RPM\n", drive_percentage, (int) fan_speed_2.value());
    FB->writeString(&output);
    output.clear();

    FB->setTextColor(WHITE, BLACK);
    FB->writeString("Pumps:\n");
    FB->setTextColor(YELLOW, BLACK);
    output.concatf("I: o%s %4d RPM\n", (pump_powered(0) ? "n ":"ff"), pump_speed_0.value()*60);
    output.concatf("E: o%s %4d RPM", (pump_powered(1) ? "n ":"ff"), pump_speed_1.value()*60);
    FB->writeString(&output);
    output.clear();
    modal_ctrl = 2;
  }
  else if (_slider_current <= 52) {
    FB->writeString("TEC Banks ");
    if (tec_safety()) {
      FB->setTextColor(GREEN, BLACK);
      FB->writeString("Safed");
    }
    else {
      FB->setTextColor(YELLOW, BLACK);
      FB->writeString("Armed");
    }
    FB->setTextColor(WHITE, BLACK);

    FB->setCursor(0, 12);
    FB->writeString("0:  ");
    if (tec_powered(0)) {
      FB->setTextColor(GREEN, BLACK);
      FB->writeString(tec_reversed(0) ? "REVERSED" : "      ON");
    }
    else {
      FB->setTextColor(RED, BLACK);
      FB->writeString("     OFF");
    }
    FB->setTextColor(WHITE, BLACK);

    FB->setCursor(0, 20);
    FB->writeString("1:  ");
    if (tec_powered(1)) {
      FB->setTextColor(GREEN, BLACK);
      FB->writeString(tec_reversed(1) ? "REVERSED" : "      ON");
    }
    else {
      FB->setTextColor(RED, BLACK);
      FB->writeString("     OFF");
    }
    FB->setTextColor(WHITE, BLACK);
    modal_ctrl = 3;
  }
  else {
    // This is the default view. Show the dashboard.
    FB->writeString("Unimp       ");
    modal_ctrl = 4;
  }
}
