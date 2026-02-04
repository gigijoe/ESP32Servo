/*
 * ESP32PWM.h - Enhanced PWM Library for ESP32
 *
 * This library provides PWM functionality for ESP32 chips with support for:
 * - ESP32S3: MCPWM hardware acceleration for optimal servo performance
 * - Variable frequency mode: LEDC preferred, MCPWM fallback for flexibility
 * - Fixed frequency mode: MCPWM preferred, LEDC fallback for shared timers
 * - Automatic hardware allocation with intelligent fallbacks
 *
 * Key Features:
 * - Dual hardware support (LEDC + MCPWM on S3)
 * - 20 total PWM channels on ESP32S3 (8 LEDC + 12 MCPWM)
 * - Frequency locking for fixed-frequency applications (servos)
 * - Seamless hardware fallback when preferred hardware unavailable
 *
 * Usage:
 * - ESP32PWM pwm;                    // Variable frequency (default)
 * - ESP32PWM pwm(true);              // Variable frequency (explicit)
 * - ESP32PWM pwm(false);             // Fixed frequency
 *
 * Created on: Sep 22, 2018
 * Author: hephaestus
 * Enhanced for ESP32S3 MCPWM support
 */
/*
 * ESP32PWM.h - Enhanced PWM Library for ESP32
 *
 * ESP-IDF v5.x MCPWM migrated to prelude (handle-based) driver.
 * Fixes:
 *  - checkFrequencyForSideEffects() no longer allocates resources
 *  - allocatenext() returns -1 instead of while(1)
 *  - Correct LEDC channels-per-timer for ESP32S2/S3 (2 per timer)
 */

#ifndef LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_
#define LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_

#include "esp32-hal-ledc.h"

#if defined(ARDUINO)
  #include "Arduino.h"
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)
  #define NUM_PWM 6
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
  #define NUM_PWM 8
#else
  #define NUM_PWM 16
#endif

// MCPWM support for ESP32S3 (ESP-IDF v5.x prelude)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #include "driver/mcpwm_prelude.h"
  #define MCPWM_NUM_UNITS 2
  #define MCPWM_NUM_TIMERS_PER_UNIT 3
  #define MCPWM_NUM_OPERATORS_PER_TIMER 2 // two outputs per timer (A/B style)
  class ESP32PWM; // Forward declaration

  struct MCPWMTimerInfo {
    bool initialized = false;
    long freq = -1;

    // shared resources for this (group,timer)
    uint32_t resolution_hz = 1000000; // 1MHz tick
    uint32_t period_ticks = 0;

    int genCount = 0; // how many outputs attached to this timer (0..2)
    ESP32PWM* users[MCPWM_NUM_OPERATORS_PER_TIMER] = {nullptr, nullptr};

    // shared handles
    mcpwm_timer_handle_t timer = nullptr;
    mcpwm_oper_handle_t  oper  = nullptr;
  };
#endif

#define PWM_BASE_INDEX 0
#define USABLE_ESP32_PWM (NUM_PWM - PWM_BASE_INDEX)

#include <cstdint>

class ESP32PWM {
private:
  void attach(int pin);

  int pwmChannel = 0;
  bool attachedState = false;
  int pin = -1;
  uint8_t resolutionBits = 8;
  double myFreq = -1;
  bool useVariableFrequency = false;
  bool isMCPWM = false;

  int allocatenext(double freq);

  // LEDC bookkeeping
  int timerNum = -1;
  uint32_t myDuty = 0;

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  // MCPWM allocation selection
  int mcpwmGroup = -1;    // 0..1
  int mcpwmTimerIdx = -1; // 0..2
  int mcpwmSlot = -1;     // 0..1 output slot

  // per-instance handles (owned by this channel/pin)
  mcpwm_cmpr_handle_t mcpwmComparator = nullptr;
  mcpwm_gen_handle_t  mcpwmGenerator  = nullptr;
#endif

  static double _ledcSetupTimerFreq(uint8_t pin, double freq, uint8_t bit_num, uint8_t channel);

  bool checkFrequencyForSideEffects(double freq);
  void adjustFrequencyLocal(double freq, double dutyScaled);

  static double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
    if (x > in_max) return out_max;
    if (x < in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  double setup(double freq, uint8_t resolution_bits = 10);
  void attachPin(uint8_t pin);
  void deallocate();

  static int ledcChannelsPerTimer();

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  static uint32_t _mcpwmCalcPeriodTicks(uint32_t resolution_hz, double freq_hz) {
    if (freq_hz <= 0) return 0;
    double period = (double)resolution_hz / freq_hz;
    if (period < 2.0) period = 2.0;
    if (period > (double)0xFFFFFFFFu) period = (double)0xFFFFFFFFu;
    return (uint32_t)(period + 0.5);
  }

  esp_err_t _mcpwmEnsureTimerStarted();
  esp_err_t _mcpwmSetupOutputGPIO(int gpio_num);
  esp_err_t _mcpwmSetDutyTicks(uint32_t duty_ticks);
#endif

public:
  ESP32PWM(bool variableFrequency = true);
  virtual ~ESP32PWM();

  void detachPin(int pin);
  void attachPin(uint8_t pin, double freq, uint8_t resolution_bits = 10);

  bool attached() { return attachedState; }

  void write(uint32_t duty);
  void writeScaled(double duty);

  double writeTone(double freq);
  double writeNote(note_t note, uint8_t octave);
  void adjustFrequency(double freq, double dutyScaled = -1);

  uint32_t read();
  double readFreq();
  double getDutyScaled();

  static int timerAndIndexToChannel(int timer, int index);

  static void allocateTimer(int timerNumber);
  static bool explicateAllocationMode;

  int getTimer() { return timerNum; }
  int getChannel();

  static int PWMCount;
  static int timerCount[4];
  static ESP32PWM* ChannelUsed[NUM_PWM];
  static long timerFreqSet[4];

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  static MCPWMTimerInfo mcpwmTimers[MCPWM_NUM_UNITS][MCPWM_NUM_TIMERS_PER_UNIT];
#endif

  int getPin() { return pin; }

  static bool hasPwm(int pin) {
#if defined(CONFIG_IDF_TARGET_ESP32S2)
    if ((pin >= 1 && pin <= 21) || (pin == 26) || (pin >= 33 && pin <= 42))
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    if ((pin >= 1 && pin <= 21) || (pin >= 35 && pin <= 45) || (pin == 47) || (pin == 48))
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
    if ((pin >= 0 && pin <= 10) || (pin >= 18 && pin <= 21))
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
    if ((pin >= 0 && pin <= 9) || (pin >= 12 && pin <= 23))
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
    if ((pin >= 0 && pin <= 5) || (pin >= 8 && pin <= 14) || (pin >= 22 && pin <= 27))
#else
    if ((pin == 2) || (pin == 4) || (pin == 5) ||
        ((pin >= 12) && (pin <= 19)) ||
        ((pin >= 21) && (pin <= 23)) ||
        ((pin >= 25) && (pin <= 27)) ||
        (pin == 32) || (pin == 33))
#endif
      return true;
    return false;
  }

  static int channelsRemaining() {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    return NUM_PWM + (MCPWM_NUM_UNITS * MCPWM_NUM_TIMERS_PER_UNIT * MCPWM_NUM_OPERATORS_PER_TIMER) - PWMCount;
#else
    return NUM_PWM - PWMCount;
#endif
  }

  static boolean DISABLE_DAC;
};

ESP32PWM* pwmFactory(int pin);

#endif /* LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_ */
