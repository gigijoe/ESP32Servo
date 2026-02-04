/*
 * ESP32PWM.cpp
 *
 *  Created on: Sep 22, 2018
 *      Author: hephaestus
 */
#include <ESP32PWM.h>
#include "esp32-hal-ledc.h"

#include "esp_log.h"
static const char* TAG = "ESP32PWM";

// static members
int  ESP32PWM::PWMCount = -1;
bool ESP32PWM::explicateAllocationMode = false;
ESP32PWM* ESP32PWM::ChannelUsed[NUM_PWM];
long ESP32PWM::timerFreqSet[4] = { -1, -1, -1, -1 };
int  ESP32PWM::timerCount[4]   = {  0,  0,  0,  0 };

#if defined(CONFIG_IDF_TARGET_ESP32S3)
MCPWMTimerInfo ESP32PWM::mcpwmTimers[MCPWM_NUM_UNITS][MCPWM_NUM_TIMERS_PER_UNIT];
#endif

// ---------- helpers ----------
int ESP32PWM::ledcChannelsPerTimer() {
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
  // ESP32S2/S3: 8 LEDC channels total -> 2 per timer (4 timers)
  return 2;
#else
  // Legacy behavior used by this library: 4 per timer
  return 4;
#endif
}

void ESP32PWM::allocateTimer(int timerNumber) {
  if (timerNumber < 0 || timerNumber > 3) return;

  if (!ESP32PWM::explicateAllocationMode) {
    ESP32PWM::explicateAllocationMode = true;
    // Mark all timers as "full", then open requested timer.
    for (int i = 0; i < 4; i++) ESP32PWM::timerCount[i] = ledcChannelsPerTimer();
  }
  ESP32PWM::timerCount[timerNumber] = 0;
}

ESP32PWM::ESP32PWM(bool variableFrequency) : useVariableFrequency(variableFrequency) {
  resolutionBits = 8;
  pwmChannel = -1;
  pin = -1;
  myFreq = -1;
  isMCPWM = false;
  timerNum = -1;
  myDuty = 0;

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  mcpwmGroup = -1;
  mcpwmTimerIdx = -1;
  mcpwmSlot = -1;
  mcpwmComparator = nullptr;
  mcpwmGenerator = nullptr;
#endif

  if (PWMCount == -1) {
    for (int i = 0; i < NUM_PWM; i++) ChannelUsed[i] = NULL;
    PWMCount = PWM_BASE_INDEX;

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_LOGI(TAG, "ESP32S3: LEDC + MCPWM(prelude) enabled");
#else
    ESP_LOGI(TAG, "LEDC PWM enabled");
#endif
  }
}

ESP32PWM::~ESP32PWM() {
  if (attached()) {
    if (!isMCPWM) {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
      ledcDetach(pin);
#else
      ledcDetachPin(pin);
#endif
#else
      ledcDetachPin(pin);
#endif
    }
  }
  deallocate();
}

double ESP32PWM::_ledcSetupTimerFreq(uint8_t pin, double freq, uint8_t bit_num, uint8_t channel) {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  return ledcAttachChannel(pin, freq, bit_num, channel);
#else
  return ledcSetup(channel, freq, bit_num);
#endif
#else
  return ledcSetup(channel, freq, bit_num);
#endif
}

int ESP32PWM::timerAndIndexToChannel(int timerNumLocal, int index) {
  int localIndex = 0;
  for (int j = 0; j < NUM_PWM; j++) {
    if (((j / 2) % 4) == timerNumLocal) {
      if (localIndex == index) return j;
      localIndex++;
    }
  }
  return -1;
}

// ---------- MCPWM (ESP32S3) ----------
#if defined(CONFIG_IDF_TARGET_ESP32S3)

esp_err_t ESP32PWM::_mcpwmEnsureTimerStarted() {
  if (mcpwmGroup < 0 || mcpwmTimerIdx < 0) return ESP_ERR_INVALID_STATE;
  auto& ti = mcpwmTimers[mcpwmGroup][mcpwmTimerIdx];
  if (!ti.initialized || !ti.timer || !ti.oper) return ESP_ERR_INVALID_STATE;

  ESP_ERROR_CHECK(mcpwm_timer_enable(ti.timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(ti.timer, MCPWM_TIMER_START_NO_STOP));
  return ESP_OK;
}

esp_err_t ESP32PWM::_mcpwmSetupOutputGPIO(int gpio_num) {
  if (mcpwmGroup < 0 || mcpwmTimerIdx < 0) return ESP_ERR_INVALID_STATE;
  auto& ti = mcpwmTimers[mcpwmGroup][mcpwmTimerIdx];
  if (!ti.oper) return ESP_ERR_INVALID_STATE;

  mcpwm_comparator_config_t cmp_cfg = {};
  cmp_cfg.flags.update_cmp_on_tez = 1; // update on timer zero
  ESP_ERROR_CHECK(mcpwm_new_comparator(ti.oper, &cmp_cfg, &mcpwmComparator));

  mcpwm_generator_config_t gen_cfg = {};
  gen_cfg.gen_gpio_num = gpio_num;
  ESP_ERROR_CHECK(mcpwm_new_generator(ti.oper, &gen_cfg, &mcpwmGenerator));

  // PWM: HIGH at timer empty (zero), LOW at compare match (UP count)
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
      mcpwmGenerator,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
      mcpwmGenerator,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwmComparator, MCPWM_GEN_ACTION_LOW)));

  return ESP_OK;
}

esp_err_t ESP32PWM::_mcpwmSetDutyTicks(uint32_t duty_ticks) {
  if (mcpwmGroup < 0 || mcpwmTimerIdx < 0) return ESP_ERR_INVALID_STATE;
  auto& ti = mcpwmTimers[mcpwmGroup][mcpwmTimerIdx];
  if (!mcpwmComparator || ti.period_ticks == 0) return ESP_ERR_INVALID_STATE;

  if (duty_ticks >= ti.period_ticks) duty_ticks = (ti.period_ticks > 1) ? (ti.period_ticks - 1) : 0;
  return mcpwm_comparator_set_compare_value(mcpwmComparator, duty_ticks);
}

#endif // CONFIG_IDF_TARGET_ESP32S3

// ---------- allocation ----------
int ESP32PWM::allocatenext(double freq) {
  long freqlocal = (long)freq;
  const int perTimer = ledcChannelsPerTimer();

  // already allocated
  if (isMCPWM) return 0;
  if (pwmChannel >= 0) return pwmChannel;

  if (useVariableFrequency) {
    // Variable frequency: prefer LEDC (independent freq per timer)
    for (int i = 0; i < 4; i++) {
      bool freqAllocated = ((timerFreqSet[i] == freqlocal) || (timerFreqSet[i] == -1));
      if (freqAllocated && timerCount[i] < perTimer) {
        if (timerFreqSet[i] == -1) timerFreqSet[i] = freqlocal;
        timerNum = i;

        for (int index = 0; index < perTimer; ++index) {
          int ch = timerAndIndexToChannel(timerNum, index);
          if (ch >= 0 && !ChannelUsed[ch]) {
            pwmChannel = ch;
            ChannelUsed[pwmChannel] = this;
            timerCount[timerNum]++;
            PWMCount++;
            myFreq = freq;
            isMCPWM = false;
            return pwmChannel;
          }
        }
      }
    }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // fallback to MCPWM ONLY if empty timer (avoid changing other users freq)
    for (int u = 0; u < MCPWM_NUM_UNITS; u++) {
      for (int t = 0; t < MCPWM_NUM_TIMERS_PER_UNIT; t++) {
        if (mcpwmTimers[u][t].genCount == 0) {
          mcpwmGroup = u;
          mcpwmTimerIdx = t;
          mcpwmSlot = 0;

          mcpwmTimers[u][t].users[0] = this;
          mcpwmTimers[u][t].genCount = 1;
          mcpwmTimers[u][t].freq = freqlocal;

          PWMCount++;
          myFreq = freq;
          isMCPWM = true;
          pwmChannel = -1;
          timerNum = -1;
          return 0;
        }
      }
    }
#endif

  } else {
    // Fixed frequency: prefer MCPWM shared freq
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    for (int u = 0; u < MCPWM_NUM_UNITS; u++) {
      for (int t = 0; t < MCPWM_NUM_TIMERS_PER_UNIT; t++) {
        bool freqMatch = (mcpwmTimers[u][t].freq == freqlocal || mcpwmTimers[u][t].freq == -1);
        if (freqMatch && mcpwmTimers[u][t].genCount < MCPWM_NUM_OPERATORS_PER_TIMER) {
          if (mcpwmTimers[u][t].freq == -1) mcpwmTimers[u][t].freq = freqlocal;

          mcpwmGroup = u;
          mcpwmTimerIdx = t;
          mcpwmSlot = mcpwmTimers[u][t].genCount;

          mcpwmTimers[u][t].users[mcpwmSlot] = this;
          mcpwmTimers[u][t].genCount++;

          PWMCount++;
          myFreq = freq;
          isMCPWM = true;
          pwmChannel = -1;
          timerNum = -1;
          return 0;
        }
      }
    }
#endif

    // fallback LEDC shared freq
    for (int i = 0; i < 4; i++) {
      bool freqAllocated = ((timerFreqSet[i] == freqlocal) || (timerFreqSet[i] == -1));
      if (freqAllocated && timerCount[i] < perTimer) {
        if (timerFreqSet[i] == -1) timerFreqSet[i] = freqlocal;
        timerNum = i;

        for (int index = 0; index < perTimer; ++index) {
          int ch = timerAndIndexToChannel(timerNum, index);
          if (ch >= 0 && !ChannelUsed[ch]) {
            pwmChannel = ch;
            ChannelUsed[pwmChannel] = this;
            timerCount[timerNum]++;
            PWMCount++;
            myFreq = freq;
            isMCPWM = false;
            return pwmChannel;
          }
        }
      }
    }
  }

  // fail: no resource
  ESP_LOGE(TAG, "ERROR: All PWM resources allocated; cannot allocate %.3f Hz", freq);
  return -1;
}

void ESP32PWM::deallocate() {
  if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (mcpwmGroup >= 0 && mcpwmTimerIdx >= 0) {
      auto& ti = mcpwmTimers[mcpwmGroup][mcpwmTimerIdx];

      // delete per-pin objects
      if (mcpwmGenerator) {
        mcpwm_del_generator(mcpwmGenerator);
        mcpwmGenerator = nullptr;
      }
      if (mcpwmComparator) {
        mcpwm_del_comparator(mcpwmComparator);
        mcpwmComparator = nullptr;
      }

      if (mcpwmSlot >= 0 && mcpwmSlot < MCPWM_NUM_OPERATORS_PER_TIMER) {
        ti.users[mcpwmSlot] = nullptr;
      }
      if (ti.genCount > 0) ti.genCount--;

      // if no more users, delete shared timer/operator
      if (ti.genCount == 0) {
        if (ti.oper) {
          mcpwm_del_operator(ti.oper);
          ti.oper = nullptr;
        }
        if (ti.timer) {
          mcpwm_timer_start_stop(ti.timer, MCPWM_TIMER_STOP_EMPTY);
          mcpwm_timer_disable(ti.timer);
          mcpwm_del_timer(ti.timer);
          ti.timer = nullptr;
        }
        ti.initialized = false;
        ti.freq = -1;
        ti.period_ticks = 0;
      }
    }
#endif
  } else if (pwmChannel >= 0) {
    timerCount[getTimer()]--;
    if (timerCount[getTimer()] == 0) timerFreqSet[getTimer()] = -1;
    ChannelUsed[pwmChannel] = NULL;
    pwmChannel = -1;
  }

  timerNum = -1;
  attachedState = false;
  isMCPWM = false;

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  mcpwmGroup = -1;
  mcpwmTimerIdx = -1;
  mcpwmSlot = -1;
#endif

  PWMCount--;
}

// ---------- channel/timer info ----------
int ESP32PWM::getChannel() {
  if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    return 100 + (mcpwmGroup * 10 + mcpwmTimerIdx * 2 + mcpwmSlot);
#else
    return -1;
#endif
  }

  if (pwmChannel < 0) ESP_LOGE(TAG, "FAIL! must setup() before using getChannel()");
  return pwmChannel;
}

// ---------- side-effects check (NO ALLOCATION HERE!) ----------
bool ESP32PWM::checkFrequencyForSideEffects(double freq) {
  // Only meaningful for LEDC timers (shared timer -> shared freq impact).
  if (isMCPWM) return true;
  if (timerNum < 0) return true;

  const int perTimer = ledcChannelsPerTimer();

  for (int i = 0; i < perTimer; i++) {
    int pwm = timerAndIndexToChannel(timerNum, i);
    if (pwm < 0) continue;
    if (pwm == pwmChannel) continue;

    if (ChannelUsed[pwm] != NULL && ChannelUsed[pwm]->getTimer() == timerNum) {
      double diff = abs(ChannelUsed[pwm]->myFreq - freq);
      if (diff > 0.1) {
        ESP_LOGW(TAG,
          "\tWARNING: PWM channel %d shares timer %d with channel %d\n"
          "\tChanging frequency to %.3f Hz will ALSO change channel %d (was %.3f Hz)\n",
          pwmChannel, timerNum, pwm, freq, pwm, ChannelUsed[pwm]->myFreq);
      }
    }
  }
  return true;
}

// ---------- setup/attach ----------
double ESP32PWM::setup(double freq, uint8_t resolution_bits) {
  resolutionBits = resolution_bits;

  // Allocate resources if not allocated yet
  if (!attached()) {
    int ret = allocatenext(freq);
    if (ret < 0) return -1;
  }

  if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    auto& ti = mcpwmTimers[mcpwmGroup][mcpwmTimerIdx];

    if (!ti.initialized) {
      ti.resolution_hz = 1000000;
      ti.period_ticks = _mcpwmCalcPeriodTicks(ti.resolution_hz, freq);

      mcpwm_timer_config_t tcfg = {};
      tcfg.group_id = mcpwmGroup;
      tcfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
      tcfg.resolution_hz = ti.resolution_hz;
      tcfg.period_ticks = ti.period_ticks;
      tcfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
      tcfg.flags.update_period_on_empty = 1;
      ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &ti.timer));

      mcpwm_operator_config_t ocfg = {};
      ocfg.group_id = mcpwmGroup;
      ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &ti.oper));
      ESP_ERROR_CHECK(mcpwm_operator_connect_timer(ti.oper, ti.timer));

      ti.initialized = true;
      ti.freq = (long)freq;
    } else {
      uint32_t new_period = _mcpwmCalcPeriodTicks(ti.resolution_hz, freq);
      if (ti.period_ticks != new_period) {
        ti.period_ticks = new_period;
        ti.freq = (long)freq;
        ESP_ERROR_CHECK(mcpwm_timer_set_period(ti.timer, new_period));
      }
    }

    myFreq = freq;
    return freq;
#else
    return -1;
#endif
  }

  // LEDC path: warn about shared timer impact (NOW that timerNum is known)
  checkFrequencyForSideEffects(freq);

#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  myFreq = ledcAttachChannel(getPin(), freq, resolution_bits, getChannel());
#else
  myFreq = ledcSetup(getChannel(), freq, resolution_bits);
#endif
#else
  myFreq = ledcSetup(getChannel(), freq, resolution_bits);
#endif
  return myFreq;
}

void ESP32PWM::attach(int p) {
  pin = p;
  attachedState = true;
}

void ESP32PWM::attachPin(uint8_t p) {
  if (!hasPwm(p)) {
    ESP_LOGE(TAG, "ERROR: PWM unavailable on pin %d", p);
    return;
  }

  attach(p);

  if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_ERROR_CHECK(_mcpwmEnsureTimerStarted());
    ESP_ERROR_CHECK(_mcpwmSetupOutputGPIO(p));
    // apply current duty immediately
    write(myDuty);
#endif
    return;
  }

#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  (void)ledcAttachChannel(p, readFreq(), resolutionBits, getChannel());
#else
  ledcAttachPin(p, getChannel());
#endif
#else
  ledcAttachPin(p, getChannel());
#endif
}

void ESP32PWM::attachPin(uint8_t p, double freq, uint8_t resolution_bits) {
  if (!hasPwm(p)) {
    ESP_LOGE(TAG, "ERROR: Pin %d has no PWM capability", p);
    return;
  }

  this->pin = p;

  double ret = setup(freq, resolution_bits);
  if (ret < 0) {
    ESP_LOGE(TAG, "ERROR: setup() failed for pin %d freq %.3f", p, freq);
    return;
  }

  attachPin(p);
}

void ESP32PWM::detachPin(int p) {
  (void)p;
  if (!isMCPWM) {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcDetach(pin);
#else
    ledcDetachPin(pin);
#endif
#else
    ledcDetachPin(pin);
#endif
  }
  deallocate();
}

// ---------- write/read ----------
double ESP32PWM::getDutyScaled() {
  return mapf((double)myDuty, 0, (double)((1 << resolutionBits) - 1), 0.0, 1.0);
}

void ESP32PWM::writeScaled(double duty) {
  write((uint32_t)mapf(duty, 0.0, 1.0, 0, (double)((1 << resolutionBits) - 1)));
}

void ESP32PWM::write(uint32_t duty) {
  myDuty = duty;

  if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    auto& ti = mcpwmTimers[mcpwmGroup][mcpwmTimerIdx];
    double duty_scaled = (double)duty / (double)((1u << resolutionBits) - 1u);
    if (duty_scaled < 0) duty_scaled = 0;
    if (duty_scaled > 1) duty_scaled = 1;

    uint32_t ticks = (uint32_t)((double)ti.period_ticks * duty_scaled + 0.5);
    ESP_ERROR_CHECK(_mcpwmSetDutyTicks(ticks));
#endif
    return;
  }

#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(getPin(), duty);
#else
  ledcWrite(getChannel(), duty);
#endif
#else
  ledcWrite(getChannel(), duty);
#endif
}

uint32_t ESP32PWM::read() {
  if (isMCPWM) return myDuty;

#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  return ledcRead(getPin());
#else
  return ledcRead(getChannel());
#endif
#else
  return ledcRead(getChannel());
#endif
}

double ESP32PWM::readFreq() { return myFreq; }

// ---------- frequency adjust ----------
void ESP32PWM::adjustFrequencyLocal(double freq, double dutyScaled) {
  timerFreqSet[getTimer()] = (long)freq;
  myFreq = freq;

#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcDetach(pin);
  _ledcSetupTimerFreq(getPin(), freq, resolutionBits, getChannel());
  writeScaled(dutyScaled);
  ledcAttachChannel(getPin(), freq, resolutionBits, getChannel());
#else
  ledcDetachPin(pin);
  _ledcSetupTimerFreq(getPin(), freq, resolutionBits, getChannel());
  writeScaled(dutyScaled);
  ledcAttachPin(pin, getChannel());
#endif
#else
  ledcDetachPin(pin);
  _ledcSetupTimerFreq(getPin(), freq, resolutionBits, getChannel());
  writeScaled(dutyScaled);
  ledcAttachPin(pin, getChannel());
#endif
}

void ESP32PWM::adjustFrequency(double freq, double dutyScaled) {
  if (!useVariableFrequency) {
    ESP_LOGE(TAG, "ERROR: Cannot change frequency on fixed-frequency PWM (pin %d).", pin);
    return;
  }
  if (dutyScaled < 0) dutyScaled = getDutyScaled();

  if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    auto& ti = mcpwmTimers[mcpwmGroup][mcpwmTimerIdx];
    uint32_t new_period = _mcpwmCalcPeriodTicks(ti.resolution_hz, freq);

    ti.period_ticks = new_period;
    ti.freq = (long)freq;
    myFreq = freq;

    ESP_ERROR_CHECK(mcpwm_timer_set_period(ti.timer, new_period));

    // keep duty% consistent for all users on this timer
    for (int i = 0; i < MCPWM_NUM_OPERATORS_PER_TIMER; i++) {
      if (ti.users[i]) {
        ti.users[i]->myFreq = freq;
        ti.users[i]->writeScaled(ti.users[i]->getDutyScaled());
      }
    }
#endif
    return;
  }

  // LEDC shared timer: update all channels on same timer
  const int perTimer = ledcChannelsPerTimer();
  for (int i = 0; i < perTimer; i++) {
    int pwm = timerAndIndexToChannel(getTimer(), i);
    if (pwm < 0) continue;
    if (ChannelUsed[pwm] != NULL) {
      if (abs(ChannelUsed[pwm]->myFreq - freq) > 0.1) {
        ChannelUsed[pwm]->adjustFrequencyLocal(freq, ChannelUsed[pwm]->getDutyScaled());
      }
    }
  }
}

double ESP32PWM::writeTone(double freq) {
  if (isMCPWM) {
    adjustFrequency(freq, 0.5);
  } else {
    adjustFrequency(freq, 0.5);
    write(1 << (resolutionBits - 1));
  }
  return 0;
}

double ESP32PWM::writeNote(note_t note, uint8_t octave) {
  const uint16_t noteFrequencyBase[12] = {
    4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902
  };
  if (octave > 8 || note >= NOTE_MAX) return 0;
  double noteFreq = (double)noteFrequencyBase[note] / (double)(1 << (8 - octave));
  return writeTone(noteFreq);
}

// ---------- factory ----------
ESP32PWM* pwmFactory(int pin) {
  for (int i = 0; i < NUM_PWM; i++) {
    if (ESP32PWM::ChannelUsed[i] != NULL) {
      if (ESP32PWM::ChannelUsed[i]->getPin() == pin) return ESP32PWM::ChannelUsed[i];
    }
  }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  for (int g = 0; g < MCPWM_NUM_UNITS; g++) {
    for (int t = 0; t < MCPWM_NUM_TIMERS_PER_UNIT; t++) {
      for (int s = 0; s < MCPWM_NUM_OPERATORS_PER_TIMER; s++) {
        if (ESP32PWM::mcpwmTimers[g][t].users[s] != NULL) {
          if (ESP32PWM::mcpwmTimers[g][t].users[s]->getPin() == pin)
            return ESP32PWM::mcpwmTimers[g][t].users[s];
        }
      }
    }
  }
#endif
  return NULL;
}
