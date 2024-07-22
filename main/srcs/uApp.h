/*
* User interface types, structures and classes.
*
*                                                  ---J. Ian Lindsay  2021.12.19
*/
#include "AbstractPlatform.h"
#include "TimerTools/TimerTools.h"
#include "Image/Image.h"
#include "Image/ImageUtils.h"
#include "FlagContainer.h"

#ifndef __U_APP_H_
#define __U_APP_H_

// TODO: Might make this a config or load-contingent value.
#define APP_POLLING_PERIOD        20    // App runs at ~50fps.
#define UAPP_BOOT_INIT_TIMEOUT  1500    // Number of milliseconds to wait for init.


/*******************************************************************************
* Types
*******************************************************************************/
enum class AppID : uint8_t {
  APP_BOOT     =  0,  // Should run once on startup.
  APP_SELECT   =  1,  // For choosing the app.
  TOUCH_TEST   =  2,  // For diagnostics of the touch pad.
  CONFIGURATOR =  3,  // For tuning all the things.
  COMMS_TEST   =  4,  // Connecting to the outside world.
  CALORIMETER  =  5,  // Calorimeter
  HOT_STANDBY  =  6,  // Full operation with powered-down UI elements.
  SUSPEND      =  7   // Minimal power without an obligatory reboot.
};

/* Application lifecycle state machine positions. */
enum class AppLifecycle : uint8_t {
  UNINITIALIZED = 0,  //
  PREINIT       = 1,  //
  ACTIVE        = 2,  //
  TEARDOWN      = 3,  //
  INACTIVE      = 4   //
};


/* Class flags */
#define UAPP_FLAG_LIFECYC_SM_MASK  0x00000007  //
#define UAPP_FLAG_IS_ACTIVE        0x00000010  // This application is active.
#define UAPP_FLAG_CLUTTER_DISPLAY  0x00000020  // User wants more information on the display.
#define UAPP_FLAG_TEXT_OF_VALUE    0x00000040  // User wants more information on the display.
#define UAPP_FLAG_LOCK_RANGE       0x00000080  // Lock the output range.

#define UAPP_FLAG_PRESS_UP         0x04000000  //
#define UAPP_FLAG_PRESS_DOWN       0x08000000  //
#define UAPP_FLAG_PRESS_LEFT       0x10000000  //
#define UAPP_FLAG_PRESS_RIGHT      0x20000000  //
#define UAPP_FLAG_PRESS_OK         0x40000000  //
#define UAPP_FLAG_PRESS_CANCEL     0x80000000  //


/*******************************************************************************
*******************************************************************************/
class uApp {
  public:
    int8_t refresh();  // TODO: Might-should be static?

    inline void deliverSliderValue(uint16_t val) {    _slider_pending   = val;  };
    inline void deliverButtonValue(uint16_t val) {    _buttons_pending  = val;  };
    inline void printStopwatch(StringBuilder* out) {  _stopwatch.printDebug(_UA_NAME, out);   };
    inline void resetStopwatch() {                    _stopwatch.reset();                     };
    inline bool isActive() {     return _uapp_flag(UAPP_FLAG_IS_ACTIVE);  };
    inline const char* getAppIDString() {   return _UA_NAME;    };

    static uApp* appActive();
    static uApp* drawnApp();
    static uApp* previousApp();
    static void setAppActive(AppID);


  protected:
    const char*   _UA_NAME;
    Image*        FB;
    StopWatch     _stopwatch;
    uint16_t      _slider_current  = 0;
    uint16_t      _slider_pending  = 0;
    uint16_t      _buttons_current = 0;
    uint16_t      _buttons_pending = 0;

    uApp(const char* _n, Image* img) : _UA_NAME(_n), FB(img) {};
    virtual ~uApp() {};

    virtual int8_t _lc_on_preinit()      =0;
    virtual int8_t _lc_on_active()       =0;
    virtual int8_t _lc_on_teardown()     =0;
    virtual int8_t _lc_on_inactive()     =0;
    virtual int8_t _process_user_input() =0;
    virtual void   _redraw_window()      =0;

    inline void redraw_app_window() {         redraw_app_window(_UA_NAME, 0, 0);  };

    inline bool _cluttered_display() {        return _uapp_flag(UAPP_FLAG_CLUTTER_DISPLAY);   };
    inline void _cluttered_display(bool x) {  _uapp_set_flag(UAPP_FLAG_CLUTTER_DISPLAY, x);   };
    inline bool _render_text_value() {        return _uapp_flag(UAPP_FLAG_TEXT_OF_VALUE);     };
    inline void _render_text_value(bool x) {  _uapp_set_flag(UAPP_FLAG_TEXT_OF_VALUE, x);     };
    inline bool _render_lock_range() {        return _uapp_flag(UAPP_FLAG_LOCK_RANGE);        };
    inline void _render_lock_range(bool x) {  _uapp_set_flag(UAPP_FLAG_LOCK_RANGE, x);        };

    inline bool _button_pressed_up() {        return _uapp_flag(UAPP_FLAG_PRESS_UP);        };
    inline void _button_pressed_up(bool x) {  _uapp_set_flag(UAPP_FLAG_PRESS_UP, x);        };
    inline bool _button_pressed_dn() {        return _uapp_flag(UAPP_FLAG_PRESS_DOWN);      };
    inline void _button_pressed_dn(bool x) {  _uapp_set_flag(UAPP_FLAG_PRESS_DOWN, x);      };
    inline bool _button_pressed_l() {         return _uapp_flag(UAPP_FLAG_PRESS_LEFT);      };
    inline void _button_pressed_l(bool x) {   _uapp_set_flag(UAPP_FLAG_PRESS_LEFT, x);      };
    inline bool _button_pressed_u() {         return _uapp_flag(UAPP_FLAG_PRESS_RIGHT);     };
    inline void _button_pressed_u(bool x) {   _uapp_set_flag(UAPP_FLAG_PRESS_RIGHT, x);     };
    inline bool _button_pressed_ok() {        return _uapp_flag(UAPP_FLAG_PRESS_OK);        };
    inline void _button_pressed_ok(bool x) {  _uapp_set_flag(UAPP_FLAG_PRESS_OK, x);        };
    inline bool _button_pressed_x() {         return _uapp_flag(UAPP_FLAG_PRESS_CANCEL);    };
    inline void _button_pressed_x(bool x) {   _uapp_set_flag(UAPP_FLAG_PRESS_CANCEL, x);    };

    void _lc_increment_position();
    inline AppLifecycle _lc_current_position() {
      return (AppLifecycle) (_flags & UAPP_FLAG_LIFECYC_SM_MASK);
    };

    /* Flag manipulation inlines */
    inline uint32_t _uapp_flags() {                return _flags;           };
    inline bool _uapp_flag(uint32_t _flag) {       return (_flags & _flag); };
    inline void _uapp_clear_flag(uint32_t _flag) { _flags &= ~_flag;        };
    inline void _uapp_set_flag(uint32_t _flag) {   _flags |= _flag;         };
    inline void _uapp_set_flag(uint32_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };

    static void redraw_app_window(const char* title, uint8_t pages, uint8_t active_page);


  private:
    uint32_t _flags     = 0;
    uint32_t _last_draw = 0;
};


/*******************************************************************************
* Specific derived classes
*******************************************************************************/

class uAppBoot : public uApp {
  public:
    uAppBoot();
    ~uAppBoot();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();

  private:
    uint32_t _last_init_sent = 0;
    FlagContainer32  _init_sent_flags;
    FlagContainer32  _init_done_flags;
};


class uAppCalorimeter : public uApp {
  public:
    uAppCalorimeter();
    ~uAppCalorimeter();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppTouchTest : public uApp {
  public:
    uAppTouchTest();
    ~uAppTouchTest();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppRoot : public uApp {
  public:
    uAppRoot();
    ~uAppRoot();

  protected:
    AppID   app_page   = AppID::CALORIMETER;
    uint8_t modal_ctrl = 0;   // Set to nonzero for modal usage.

    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppStandby : public uApp {
  public:
    uAppStandby();
    ~uAppStandby();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppConfigurator : public uApp {
  public:
    uAppConfigurator();
    ~uAppConfigurator();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppComms : public uApp {
  public:
    uAppComms();
    ~uAppComms();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


/*******************************************************************************
* Externs to UI object declarations
*******************************************************************************/
extern uAppBoot app_boot;
extern uAppRoot app_root;
extern uAppTouchTest app_touch_test;
extern uAppConfigurator app_config;
extern uAppComms app_comms;
extern uAppCalorimeter app_calorimeter;
extern uAppStandby app_standby;

#endif   // __U_APP_H_
