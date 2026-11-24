// ============================================================================
//  Soldering Iron Controller — STM32F103 + MAX6675 + C245   v0.6
// ============================================================================
#include <Arduino.h>
#include <math.h>
#include <U8g2lib.h>
#include <PID_v1.h>
#include <RotaryEncoder.h>
#include <FlashStorage_STM32.h>
#include <max6675.h>

// ============================================================================
//  PINS
// ============================================================================
#define MAX6675_MISO PA6
#define MAX6675_SCK PA5
#define MAX6675_CS PA4

#define VIN_PIN PA0
#define I_PIN PA3
#define HANDLE_SENSE_PIN PA8
#define ENABLE_PIN PB5 

#define ENC_SW_PIN PB4
#define ENC_DT_PIN PB3
#define ENC_CLK_PIN PA15

#define LED_R_PIN PB12
#define LED_G_PIN PB13
#define LED_B_PIN PB14
#define CTRL_PIN PB0
#define BUZZ_PIN PB15

// ============================================================================
//  CONSTANTS
// ============================================================================
#define VERSION "v0.6"
#define TIPMAX 8
#define TIPNAMELENGTH 7 // 6 chars + terminating NUL  (was 6 -> no NUL, overran)
#define TIPNAME "C245-i"

#define DEG "\xc2\xb0" // UTF-8 degree sign (enableUTF8Print is on)

#define TEMP_MIN 150
#define TEMP_MAX 450
#define TEMP_DEFAULT 275
#define TEMP_SLEEP 150
#define TEMP_BOOST 50
#define TEMP_STEP 5

#define TIME2SLEEP 5   // minutes
#define TIME2OFF 15    // minutes
#define TIMEOFBOOST 40 // seconds
#define TEMP_SETTLE_MS 10   // heater-off settle time before reading the MAX6675

#define SMOOTHIE 0.2f       // EMA smoothing factor
#define SENSOR_INTERVAL 250 // ms (MAX6675 needs >= 220 ms per conversion)

// ---- Debug --------------------------------------------------------------
// While DEBUG is ON: the "tip missing" error and the supply-voltage check are
// disabled (no error screens).  Changeable at runtime: Setup -> Debug.
#define DEBUG_DEFAULT true

// ---- Power / current sensing (0.01 ohm shunt) ---------------------------
#define SHUNT_OHMS 0.01f
// Gain of the shunt amplifier.  *** CALIBRATE FOR YOUR BOARD ***
//   gain = V_adc_pin / (I_measured * SHUNT_OHMS)
// 80 is chosen so ADC full scale (3.3 V) is ~4.1 A, like the old table.
#define I_AMP_GAIN 80.0f
#define ADC_VREF 3.3f
#define ADC_MAX 4095.0f
#define VIN_DIV 0.09f           // input voltage divider ratio
#define I_SAMPLE_US 2000        // average current over 2 PWM periods (1 kHz)
#define CURRENT_LIMIT 4.0f      // A, hard trip (average)
#define VOLTAGE_MAX 26.0f
#define VOLTAGE_MIN 5.0f
#define TIP_R_NOMINAL 2.5f      // ohm, only used for the first power estimate

#define POWER_DEFAULT 65 // W
#define POWER_MIN 10
#define POWER_MAX_ABS 100

#define BEEP_ENABLE true
#define PID_ENABLE true
#define DEFAULT_BUZZ_MS 200

#define LONG_PRESS_TIME 650
#define DEBOUNCE_TIME 30

#define TIP_FAULT_COUNT 3 // consecutive bad reads before "tip missing"

// Logic levels
#define STAND_ACTIVE_LEVEL HIGH  // HANDLE_SENSE level when iron is on stand
#define ENABLE_ACTIVE_LEVEL HIGH // ENABLE_PIN level when pressed (pull-down)

// ============================================================================
//  MODES   0=OFF  1=SLEEP(standby)  2=RUN  3=HOLD  4=BOOST
// ============================================================================
#define MODE_OFF 0
#define MODE_SLEEP 1
#define MODE_RUN 2
#define MODE_HOLD 3
#define MODE_BOOST 4

// ============================================================================
//  DATA STRUCTURES
// ============================================================================
struct Settings
{
  uint32_t signature;
  uint16_t DefaultTemp;
  uint16_t SleepTemp;
  uint8_t BoostTemp;
  uint8_t time2sleep;
  uint8_t time2off;
  uint8_t timeOfBoost;
  uint8_t MainScrType;
  bool PIDenable;
  bool beepEnable;
  uint8_t CurrentTip;
  uint8_t NumberOfTips;
  int16_t TempOffset[TIPMAX]; // per-tip trim
  char TipName[TIPMAX][TIPNAMELENGTH];
  uint8_t MaxPower;
  bool debugMode;
};

// ============================================================================
//  FORWARD DECLARATIONS
// ============================================================================
void rotaryISR();
void setRotary(int rmin, int rmax, int rstep, int rvalue);
int getRotary();
void handleButton();
void processButton();
void handleEnableButton();
void ROTARYCheck();
void SENSORCheck();
void Thermostat();
void PowerCheck();
void SLEEPCheck();
void TipCheck();
void runControlCycle();
void LED_Handler();
void BUZZ(long duration = DEFAULT_BUZZ_MS, int buzzcount = 1);
void loadSettings();
void saveSettings();

void MainScreen();
void SetupScreen();
void GeneralScreen();
void TipScreen();
void TempScreen();
void TimerScreen();
void CalibrationScreen();
void ChangeTipScreen();
void AddTipScreen();
void DeleteTipScreen();
void InfoScreen();
void MessageScreen(const char *Items[], uint8_t numberOfItems);
uint8_t MenuScreen(const char *Items[], uint8_t numberOfItems, uint8_t selected);
uint16_t InputScreen(const char *Items[]);
uint8_t InputNameScreen();

// ============================================================================
//  DISPLAY + ENCODER + SENSOR
// ============================================================================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g(U8G2_R0);
RotaryEncoder encoder(ENC_CLK_PIN, ENC_DT_PIN, RotaryEncoder::LatchMode::FOUR3);
MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_MISO);

// ============================================================================
//  MENU STRINGS
// ============================================================================
const char *SetupItems[] = {"Setup Menu", "Tip Settings", "Temp Settings",
                            "Timers", "General", "Buzzer",
                            "Information", "Debug", "Return"};
const char *TipItems[] = {"Tip:", "Change Tip", "Calibrate Tip",
                          "Rename Tip", "Delete Tip", "Add new Tip", "Return"};
const char *TempItems[] = {"Temp Settings", "Default Temp", "Sleep Temp",
                           "Boost Temp", "Max Power", "Return"};
const char *TimerItems[] = {"Timer Settings", "Sleep Timer", "Off Timer",
                            "Boost Timer", "Return"};
const char *ControlTypeItems[] = {"Control Type", "Direct", "PID"};
const char *SureItems[] = {"Are you sure?", "No", "Yes"};
const char *BuzzerItems[] = {"Buzzer", "Disable", "Enable"};
const char *DebugItems[] = {"Debug Mode", "Off", "On"};
const char *DefaultTempItems[] = {"Default Temp", "deg C"};
const char *SleepTempItems[] = {"Sleep Temp", "deg C"};
const char *BoostTempItems[] = {"Boost Temp", "deg C"};
const char *MaxPowerItems[] = {"Max Power", "Watts"};
const char *SleepTimerItems[] = {"Sleep Timer", "Minutes"};
const char *OffTimerItems[] = {"Off Timer", "Minutes"};
const char *BoostTimerItems[] = {"Boost Timer", "Seconds"};
const char *DeleteMessage[] = {"Warning", "Cannot delete", "your last tip!"};
const char *OverVOLTmessage[] = {"VOLTAGE FAULT", "DISCONNECT"};
const char *OverCURRmessage[] = {"OVER CURRENT", "AUTO CUTOFF"};
const char *TipMissingMessage[] = {"TIP MISSING!", "Check tip &", "connection."};
const char *MaxTipMessage[] = {"Max tips reached"};

// ============================================================================
//  GLOBAL STATE
// ============================================================================
uint8_t mode = MODE_SLEEP; // boots into standby

// Temperature pipeline
float SmoothedTEMP = 0;
double Input = 0, Output = 0, Setpoint = 0;
double CurrentTemp = 0;
uint16_t SetTemp = TEMP_DEFAULT;
uint16_t gap = 0;
bool tempInit = false;
bool tipOK = false; // false until first valid thermocouple read
uint8_t tipFaultCount = 0;

// Power
float VOLTAGE = 0, CURRENT = 0, POWER = 0;
float iOffsetV = 0;       // amplifier zero-current offset (measured at boot)
bool iSaturated = false;
float pFullEst = 0;       // estimated power at 100 % duty (W)
double powerCap = 255;    // max allowed PWM value from power limiter
uint8_t lastDuty = 0;     // PWM value actually applied

// Tips
char TipName[TIPMAX][TIPNAMELENGTH];
int16_t tipTrim[TIPMAX];
int16_t activeTrim = 0;
uint8_t CurrentTip = 0;
uint8_t NumberOfTips = 1;

// User settings
uint16_t DefaultTemp = TEMP_DEFAULT;
uint16_t SleepTemp = TEMP_SLEEP;
uint8_t BoostTemp = TEMP_BOOST;
uint8_t time2sleep = TIME2SLEEP;
uint8_t time2off = TIME2OFF;
uint8_t timeOfBoost = TIMEOFBOOST;
uint8_t MainScrType = 1;
uint8_t MaxPower = POWER_DEFAULT;
bool PIDenable = PID_ENABLE;
bool beepEnable = BEEP_ENABLE;
bool debugMode = DEBUG_DEFAULT;

// Timing
uint32_t sleepmillis = 0;
uint32_t boostmillis = 0;
uint32_t sensorPreviousMillis = 0;
uint32_t buzzerPreviousMillis = 0;
uint32_t goneMinutes = 0;
uint8_t goneSeconds = 0;

volatile bool userActivity = false; // resets the sleep timer, never wakes

// ============================================================================
//  PID
// ============================================================================
double aggKp = 30, aggKi = 0.5, aggKd = 5;
double consKp = 12, consKi = 0.4, consKd = 8;
PID ctrl(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);
bool pidActive = false;

// ============================================================================
//  EEPROM
// ============================================================================
const uint32_t EEPROM_IDENT = 0xBEEFDEB1; // bumped: struct layout changed
Settings settings;

// ============================================================================
//  HEATER OUTPUT  (single place that touches CTRL_PIN)
// ============================================================================
void setHeater(uint8_t duty)
{
  lastDuty = duty;
  analogWrite(CTRL_PIN, duty);
}

// ============================================================================
//  ROTARY ENCODER
// ============================================================================
int16_t rotaryMin = TEMP_MIN;
int16_t rotaryMax = TEMP_MAX;
int16_t rotaryStep = TEMP_STEP;
int16_t rotaryOffset = 0;
int16_t rotaryBaseValue = 0;

void rotaryISR() { encoder.tick(); }

void setRotary(int rmin, int rmax, int rstep, int rvalue)
{
  rotaryMin = rmin;
  rotaryMax = rmax;
  rotaryStep = rstep;
  rotaryBaseValue = rvalue;
  rotaryOffset = encoder.getPosition();
}

int getRotary()
{
  int16_t delta = (int16_t)(encoder.getPosition() - rotaryOffset);
  int16_t value = rotaryBaseValue + delta * rotaryStep;
  if (value < rotaryMin)
  {
    value = rotaryMin;
    rotaryOffset = encoder.getPosition();
    rotaryBaseValue = rotaryMin;
  }
  else if (value > rotaryMax)
  {
    value = rotaryMax;
    rotaryOffset = encoder.getPosition();
    rotaryBaseValue = rotaryMax;
  }
  return value;
}

// ============================================================================
//  BUTTONS
// ============================================================================
enum BtnState
{
  BTN_IDLE,
  BTN_PRESSED,
  BTN_HELD
};
BtnState btnState = BTN_IDLE;
uint32_t btnPressTime = 0;
bool shortPress = false;
bool longPress = false;
bool globalButtonHandling = true;

void handleButton()
{
  static uint32_t lastDebounce = 0;
  bool pressed = !digitalRead(ENC_SW_PIN);
  uint32_t now = millis();

  switch (btnState)
  {
  case BTN_IDLE:
    if (pressed && (now - lastDebounce > DEBOUNCE_TIME))
    {
      btnPressTime = now;
      btnState = BTN_PRESSED;
    }
    break;

  case BTN_PRESSED:
    if (!pressed)
    {
      lastDebounce = now;
      if (now - btnPressTime >= DEBOUNCE_TIME)
        shortPress = true;
      btnState = BTN_IDLE;
    }
    else if (now - btnPressTime >= LONG_PRESS_TIME)
    {
      longPress = true;
      btnState = BTN_HELD;
    }
    break;

  case BTN_HELD:
    if (!pressed)
    {
      lastDebounce = now;
      btnState = BTN_IDLE;
    }
    break;
  }
}

void wakeUp()
{
  mode = MODE_RUN;
  sleepmillis = millis();
  userActivity = true;
}

void processButton()
{
  if (!globalButtonHandling)
    return;

  if (shortPress)
  {
    shortPress = false;
    BUZZ(40, 1);
    userActivity = true;

    if (mode == MODE_OFF || mode == MODE_SLEEP)
    {
      wakeUp();
      return;
    }
    mode = (mode == MODE_BOOST) ? MODE_RUN : MODE_BOOST;
    boostmillis = millis();
  }

  if (longPress)
  {
    longPress = false;
    BUZZ(120, 2);
    SetupScreen();
  }
}

// Dedicated ENABLE button (rising edge). Wakes from standby / off.
void handleEnableButton()
{
  static bool last = (digitalRead(ENABLE_PIN) == ENABLE_ACTIVE_LEVEL);
  static uint32_t lastChange = 0;

  bool now = (digitalRead(ENABLE_PIN) == ENABLE_ACTIVE_LEVEL);
  if (now != last && (millis() - lastChange) > DEBOUNCE_TIME)
  {
    lastChange = millis();
    last = now;
    if (now && (mode == MODE_SLEEP || mode == MODE_OFF))
    {
      BUZZ(100, 1);
      wakeUp();
    }
  }
}

// ============================================================================
//  BUZZER
// ============================================================================
void BUZZ(long duration, int buzzcount)
{
  if (!beepEnable)
    return;
  int i = 0;
  bool run = true;
  buzzerPreviousMillis = millis() - (unsigned long)duration;
  while (run)
  {
    unsigned long now = millis();
    if (now - buzzerPreviousMillis >= (unsigned long)duration)
    {
      digitalWrite(BUZZ_PIN, !digitalRead(BUZZ_PIN));
      buzzerPreviousMillis = now;
      if (digitalRead(BUZZ_PIN) == HIGH)
        i++;
    }
    if (i == buzzcount && digitalRead(BUZZ_PIN) == LOW)
      run = false;
  }
  digitalWrite(BUZZ_PIN, LOW);
}

// ============================================================================
//  SENSOR  (MAX6675 returns NAN when the thermocouple is open / tip missing)
// ============================================================================
void SENSORCheck()
{
  float raw = thermocouple.readCelsius();
  bool valid = !isnan(raw) && raw >= 0.0f && raw <= 1023.75f;

  if (!valid)
  {
    // Never feed NAN into the filter: it would poison SmoothedTEMP and the
    // PID forever (this was the "won't heat" bug).
    if (tipFaultCount < 255)
      tipFaultCount++;
    if (tipFaultCount >= TIP_FAULT_COUNT)
    {
      tipOK = false;
      tempInit = false; // re-seed filter when the tip comes back
    }
    return;
  }

  tipFaultCount = 0;
  tipOK = true;

  if (!tempInit)
  {
    SmoothedTEMP = raw;
    tempInit = true;
  }
  else
  {
    SmoothedTEMP += SMOOTHIE * (raw - SmoothedTEMP);
  }

  CurrentTemp = SmoothedTEMP + activeTrim;
}

// Show "tip missing" once per event (disabled in debug mode)
void TipCheck()
{
  static bool shown = false;

  if (tipOK)
  {
    shown = false;
    return;
  }
  if (debugMode || shown)
    return;

  shown = true;
  setHeater(0);
  mode = MODE_SLEEP; // come back in standby, not full heat
  MessageScreen(TipMissingMessage, 3);
  shortPress = false;
  longPress = false;
  sleepmillis = millis();
}

// ============================================================================
//  THERMOSTAT  (adaptive PID or direct, limited by the power limiter)
// ============================================================================
void Thermostat()
{
  if (mode == MODE_OFF || !tipOK)
  {
    setHeater(0);
    if (pidActive)
    {
      ctrl.SetMode(MANUAL);
      pidActive = false;
    }
    Output = 0;
    return;
  }

  if (!pidActive)
  {
    ctrl.SetMode(AUTOMATIC);
    pidActive = true;
  }

  switch (mode)
  {
  case MODE_SLEEP:
    Setpoint = SleepTemp;
    break;
  case MODE_BOOST:
    Setpoint = SetTemp + BoostTemp;
    break;
  default:
    Setpoint = SetTemp;
    break;
  }
  if (Setpoint > TEMP_MAX)
    Setpoint = TEMP_MAX;

  Input = CurrentTemp;
  gap = (uint16_t)fabsf((float)(Setpoint - Input));

  if (PIDenable)
  {
    bool nearTarget = (gap < 15);
    ctrl.SetTunings(
        nearTarget ? consKp : aggKp,
        nearTarget ? consKi : aggKi,
        nearTarget ? consKd : aggKd);
    ctrl.Compute();
  }
  else
  {
    Output = (Input < Setpoint) ? powerCap : 0; // simple on/off
  }

  if (mode == MODE_RUN && gap <= 3)
    mode = MODE_HOLD;
  else if (mode == MODE_HOLD && gap >= 6)
    mode = MODE_RUN;

  if (isnan(Output))
    Output = 0;
  if (Output > powerCap)
    Output = powerCap;
  if (Output < 0)
    Output = 0;

  setHeater((uint8_t)Output);
}

// ============================================================================
//  POWER MEASUREMENT  (0.01 ohm shunt + amplifier)
// ============================================================================

// Average the shunt amplifier output over ~2 PWM periods so the reading is
// the average heater current regardless of where in the PWM cycle we sample.
float readShuntCurrent()
{
  uint32_t t0 = micros();
  uint32_t sum = 0;
  uint16_t n = 0;
  while ((uint32_t)(micros() - t0) < I_SAMPLE_US)
  {
    sum += analogRead(I_PIN);
    n++;
  }
  float adc = n ? ((float)sum / n) : 0.0f;
  iSaturated = (adc >= 4000.0f);

  float volts = adc * ADC_VREF / ADC_MAX;
  float vShunt = (volts - iOffsetV) / I_AMP_GAIN; // volts across the shunt
  float amps = vShunt / SHUNT_OHMS;               // I = V / R
  return (amps < 0) ? 0 : amps;
}

// Measure the amplifier's zero-current offset. Call with the heater OFF.
void calibrateCurrentOffset()
{
  setHeater(0);
  delay(50);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 64; i++)
  {
    sum += analogRead(I_PIN);
    delay(1);
  }
  iOffsetV = ((float)sum / 64.0f) * ADC_VREF / ADC_MAX;
}

void measurePower()
{
  uint32_t sv = 0;
  for (uint8_t i = 0; i < 8; i++)
    sv += analogRead(VIN_PIN);
  VOLTAGE = (((float)sv / 8.0f) * ADC_VREF / ADC_MAX) / VIN_DIV;

  CURRENT = readShuntCurrent();
  POWER = VOLTAGE * CURRENT;
}

// Max-power limiter.
// avg power = P_full * duty, so P_full is estimated from the measurement and
// the duty cap is MaxPower / P_full.  The cap is also applied as the PID output
// limit, which gives free anti-windup.
void updatePowerLimit()
{
  float dutyFrac = lastDuty / 255.0f;

  if (dutyFrac >= 0.08f && POWER > 2.0f)
  {
    float pf = POWER / dutyFrac;
    pFullEst = (pFullEst <= 0) ? pf : pFullEst + 0.4f * (pf - pFullEst);
  }
  else if (pFullEst <= 0 && VOLTAGE > VOLTAGE_MIN)
  {
    pFullEst = (VOLTAGE * VOLTAGE) / TIP_R_NOMINAL; // first guess
  }

  float cap = (pFullEst > 0) ? (255.0f * MaxPower / pFullEst) : 255.0f;

  // Reactive guard: measured power already above the limit
  if (POWER > MaxPower * 1.1f && lastDuty > 0)
  {
    float c = lastDuty * (float)MaxPower / POWER;
    if (c < cap)
      cap = c;
  }

  if (cap > 255.0f)
    cap = 255.0f;
  if (cap < 4.0f)
    cap = 4.0f;

  powerCap = cap;
  ctrl.SetOutputLimits(0, powerCap);
}

void raiseFault(const char *msg[], uint8_t n)
{
  mode = MODE_OFF;
  setHeater(0);
  Output = 0;
  MessageScreen(msg, n);
  shortPress = false;
  longPress = false;
  sleepmillis = millis();
}

void PowerCheck()
{
  measurePower();

  // Over-current: always active
  static uint8_t ocCount = 0;
  if (CURRENT >= CURRENT_LIMIT || iSaturated)
  {
    if (++ocCount >= 2)
    {
      ocCount = 0;
      raiseFault(OverCURRmessage, 2);
      return;
    }
  }
  else
    ocCount = 0;

  // Supply voltage: disabled in debug mode
  if (!debugMode)
  {
    static uint8_t vCount = 0;
    if (VOLTAGE >= VOLTAGE_MAX || (VOLTAGE > 0.5f && VOLTAGE <= VOLTAGE_MIN))
    {
      if (++vCount >= 3)
      {
        vCount = 0;
        raiseFault(OverVOLTmessage, 2);
        return;
      }
    }
    else
      vCount = 0;
  }

  updatePowerLimit();
}

// One full measure -> limit -> control cycle (order matters)
void runControlCycle()
{
  PowerCheck(); // current/power must be measured while the heater is still ON

  if (lastDuty > 0)
  {
    analogWrite(CTRL_PIN, 0); // cut heater; lastDuty keeps the old value on purpose
    uint32_t t0 = millis();
    while (millis() - t0 < TEMP_SETTLE_MS)
    {
    }
  }

  SENSORCheck();  // heater is OFF here
  Thermostat();   // computes the new duty and re-enables the heater via setHeater()
}

// ============================================================================
//  ROTARY CHECK
// ============================================================================
void ROTARYCheck()
{
  static uint16_t lastSet = 0;
  SetTemp = (uint16_t)getRotary();
  if (SetTemp != lastSet)
  {
    lastSet = SetTemp;
    userActivity = true;
  }

  if (mode == MODE_BOOST && timeOfBoost)
  {
    goneSeconds = (uint8_t)((millis() - boostmillis) / 1000);
    if (goneSeconds >= timeOfBoost)
    {
      mode = MODE_RUN;
      BUZZ(200, 1);
    }
  }
}

// ============================================================================
//  SLEEP CHECK   (SLEEPCheck() was never called in v0.5)
//  Wake-up is ONLY by the enable button / encoder press.
// ============================================================================
void SLEEPCheck()
{
  bool onStand = (digitalRead(HANDLE_SENSE_PIN) == STAND_ACTIVE_LEVEL);

  if (onStand)
  {
    if (mode == MODE_RUN || mode == MODE_BOOST || mode == MODE_HOLD)
    {
      mode = MODE_SLEEP;
      BUZZ(150, 1);
    }
    return;
  }

  if (userActivity)
  {
    userActivity = false;
    sleepmillis = millis();
  }

  goneMinutes = (millis() - sleepmillis) / 60000UL;

  if (mode != MODE_SLEEP && mode != MODE_OFF && time2sleep > 0 && goneMinutes >= time2sleep)
  {
    mode = MODE_SLEEP;
    BUZZ(400, 2);
  }

  if (mode != MODE_OFF && time2off > 0 && goneMinutes >= time2off)
  {
    mode = MODE_OFF;
    setHeater(0);
    BUZZ(600, 1);
  }
}

// ============================================================================
//  LED HANDLER  (common-anode: LOW = on)
// ============================================================================
void LED_Handler()
{
  bool r = HIGH, g = HIGH, b = HIGH;
  switch (mode)
  {
  case MODE_SLEEP:
    b = LOW;
    break; // blue
  case MODE_RUN:
    r = LOW;
    break; // red
  case MODE_HOLD:
    g = LOW;
    break; // green
  case MODE_BOOST:
    r = LOW;
    g = LOW;
    break; // yellow
  default:
    break;
  }
  digitalWrite(LED_R_PIN, r);
  digitalWrite(LED_G_PIN, g);
  digitalWrite(LED_B_PIN, b);
}

// ============================================================================
//  MAIN SCREEN
// ============================================================================
void MainScreen()
{
  uint8_t pwmPct = (uint8_t)((uint16_t)lastDuty * 100 / 255);

  u8g.firstPage();
  do
  {
    u8g.setFont(u8g2_font_9x15_tf);
    u8g.setFontPosTop();

    // ── Top bar ─────────────────────────────────────────────────────────
    u8g.setCursor(0, 0);
    u8g.print(TipName[CurrentTip]);

    u8g.setCursor(90, 0);
    switch (mode)
    {
    case MODE_OFF:
      u8g.print(F("OFF"));
      break;
    case MODE_SLEEP:
      u8g.print(F("STBY"));
      break;
    case MODE_BOOST:
      u8g.setCursor(85, 0);
      u8g.print(F("BOOST"));
      break;
    case MODE_HOLD:
      u8g.print(F("HOLD"));
      break;
    case MODE_RUN:
      u8g.print(F("HEAT"));
      break;
    default:
      u8g.print(F("-----"));
      break;
    }

    u8g.drawHLine(0, 14, 128);

    // ── Centre: large current temperature ───────────────────────────────
    u8g.setFont(u8g2_font_fub25_tf);
    u8g.setFontPosTop();

    if (!tipOK)
    {
      u8g.setCursor(35, 17);
      u8g.print(F("---"));
    }
    else if (CurrentTemp > TEMP_MAX)
    {
      u8g.setCursor(35, 17);
      u8g.print(F("OVT"));
    }
    else
    {
      uint16_t t = (CurrentTemp < 0) ? 0 : (uint16_t)CurrentTemp;
      u8g.setCursor((t < 10) ? 100 : (t < 100) ? 68 : 48, 17);
      u8g.print(t);
    }

    u8g.setFont(u8g2_font_9x15_tf);
    u8g.setFontPosTop();
    u8g.setCursor(106, 30);
    u8g.print(DEG "C");

    // ── Bottom bar ───────────────────────────────────────────────────────
    u8g.drawHLine(0, 48, 128);

    // Max power setting
    u8g.setCursor(0, 51);
    u8g.print((uint16_t)MaxPower);
    u8g.print(F("W"));

    u8g.setCursor(0, 18);
    u8g.print(F("PWR"));
    u8g.setCursor(0, 33);

    if (pwmPct >= 99)
    {
      u8g.drawButtonUTF8(0, 34, U8G2_BTN_INV, 5, 1, 1, "MAX");
    }
    else if (pwmPct == 0)
    {
      u8g.drawButtonUTF8(0, 34, U8G2_BTN_INV, 5, 1, 1, "OFF");
    }
    else
    {
      u8g.print(pwmPct);
      u8g.print(F("%"));
    }

    // SET temp
    u8g.setCursor(43, 51);
    u8g.print(F("SET:"));
    u8g.setCursor((SetTemp < 100) ? 89 : 80, 51);
    u8g.print(SetTemp);
    u8g.print(DEG "C");

    // Boost countdown
    if (mode == MODE_BOOST && timeOfBoost > 0)
    {
      u8g.setCursor(50, 0);
      u8g.print((int)timeOfBoost - (int)goneSeconds);
      u8g.print(F("s"));
    }

  } while (u8g.nextPage());
}

// ============================================================================
//  MENU SCREEN
// ============================================================================
uint8_t MenuScreen(const char *Items[], uint8_t numberOfItems, uint8_t selected)
{
  bool isTipScreen = (strcmp(Items[0], "Tip:") == 0);
  uint8_t lastselected = selected;
  int8_t arrow = selected ? 1 : 0;

  setRotary(0, numberOfItems - 2, 1, selected);
  bool lastbutton = !digitalRead(ENC_SW_PIN);

  do
  {
    selected = (uint8_t)getRotary();
    arrow = (int8_t)constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;

    u8g.firstPage();
    do
    {
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();
      u8g.drawStr(0, 0, Items[0]);
      if (isTipScreen)
        u8g.drawStr(54, 0, TipName[CurrentTip]);
      u8g.drawStr(0, 16 * (arrow + 1), ">");
      for (uint8_t i = 0; i < 3; i++)
      {
        uint8_t idx = selected + i + 1 - arrow;
        if (idx < numberOfItems)
          u8g.drawStr(12, 16 * (i + 1), Items[idx]);
      }
    } while (u8g.nextPage());

    if (lastbutton && digitalRead(ENC_SW_PIN))
    {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(ENC_SW_PIN) || lastbutton);

  BUZZ(60, 1);
  return selected;
}

// ============================================================================
//  MESSAGE SCREEN
// ============================================================================
void MessageScreen(const char *Items[], uint8_t numberOfItems)
{
  bool lastbutton = !digitalRead(ENC_SW_PIN);
  u8g.firstPage();
  do
  {
    u8g.drawHLine(0, 0, 128);
    u8g.drawHLine(0, 63, 128);
    u8g.drawVLine(0, 0, 64);
    u8g.drawVLine(127, 0, 64);
    u8g.setFont(u8g2_font_9x15_tf);
    u8g.setFontPosTop();
    for (uint8_t i = 0; i < numberOfItems; i++)
      u8g.drawStr(4, 4 + i * 15, Items[i]);
  } while (u8g.nextPage());
  do
  {
    if (lastbutton && digitalRead(ENC_SW_PIN))
    {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(ENC_SW_PIN) || lastbutton);
}

// ============================================================================
//  INPUT SCREEN
// ============================================================================
uint16_t InputScreen(const char *Items[])
{
  bool lastbutton = !digitalRead(ENC_SW_PIN);
  uint16_t value = 0;
  do
  {
    value = (uint16_t)getRotary();
    u8g.firstPage();
    do
    {
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();
      u8g.drawStr(0, 0, Items[0]);
      u8g.setCursor(0, 32);
      u8g.print(F("> "));
      if (value == 0)
        u8g.print(F("Disabled"));
      else
      {
        u8g.print(value);
        u8g.print(F(" "));
        u8g.print(Items[1]);
      }
    } while (u8g.nextPage());
    if (lastbutton && digitalRead(ENC_SW_PIN))
    {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(ENC_SW_PIN) || lastbutton);
  return value;
}

// ============================================================================
//  INPUT NAME SCREEN
// ============================================================================
uint8_t InputNameScreen()
{
  uint8_t value = 65;
  for (uint8_t digit = 0; digit < TIPNAMELENGTH - 1; digit++)
  {
    bool lastbutton = !digitalRead(ENC_SW_PIN);
    setRotary(32, 95, 1, 65);
    do
    {
      value = (uint8_t)getRotary();
      u8g.firstPage();
      do
      {
        u8g.setFont(u8g2_font_9x15_tf);
        u8g.setFontPosTop();
        u8g.drawStr(0, 0, "Rename Tip");
        u8g.setCursor(0, 32);
        for (uint8_t i = 0; i < digit; i++)
          u8g.print(TipName[CurrentTip][i]);
        u8g.print((char)value);
        u8g.setCursor(9 * digit, 48);
        u8g.print(F("^"));
      } while (u8g.nextPage());
      if (lastbutton && digitalRead(ENC_SW_PIN))
      {
        delay(10);
        lastbutton = false;
      }
    } while (digitalRead(ENC_SW_PIN) || lastbutton);
    TipName[CurrentTip][digit] = (char)value;
    delay(10);
  }
  TipName[CurrentTip][TIPNAMELENGTH - 1] = 0;
  return value;
}

// ============================================================================
//  INFO SCREEN  (heater OFF while shown; live sensor + power readings)
// ============================================================================
void InfoScreen()
{
  bool exiting = false;
  uint32_t lastRead = 0;
  shortPress = false;
  longPress = false;
  globalButtonHandling = false;
  setHeater(0);

  while (!exiting)
  {
    handleButton();

    if (millis() - lastRead >= SENSOR_INTERVAL)
    {
      lastRead = millis();
      SENSORCheck();
      measurePower();
    }

    u8g.firstPage();
    do
    {
      handleButton();
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();

      u8g.setCursor(0, 0);
      u8g.print(F("FW: " VERSION));
      if (debugMode)
        u8g.print(F(" DBG"));

      u8g.setCursor(0, 13);
      u8g.print(F("RAW: "));
      u8g.print((int)SmoothedTEMP);
      u8g.print(DEG "C");

      u8g.setCursor(0, 26);
      if (tipOK)
        u8g.print((int)CurrentTemp);
      else
        u8g.print(F("NO TIP"));
      u8g.print(DEG "C T:");
      u8g.print((int)activeTrim);

      u8g.setCursor(0, 39);
      u8g.print(VOLTAGE, 1);
      u8g.print(F("V "));
      u8g.print(CURRENT, 2);
      u8g.print(F("A "));
      u8g.print((int)POWER);
      u8g.print(F("W"));

      u8g.setCursor(0, 52);
      u8g.print(F("CAP:"));
      u8g.print((int)(powerCap * 100.0 / 255.0));
      u8g.print(F("% "));
      switch (mode)
      {
      case MODE_OFF:
        u8g.print(F("OFF"));
        break;
      case MODE_SLEEP:
        u8g.print(F("STBY"));
        break;
      case MODE_RUN:
        u8g.print(F("RUN"));
        break;
      case MODE_HOLD:
        u8g.print(F("HOLD"));
        break;
      case MODE_BOOST:
        u8g.print(F("BOOST"));
        break;
      }
    } while (u8g.nextPage());

    if (shortPress || longPress)
      exiting = true;
  }

  globalButtonHandling = true;
  shortPress = false;
  longPress = false;
}

// ============================================================================
//  CHANGE TIP SCREEN
// ============================================================================
void ChangeTipScreen()
{
  uint8_t selected = CurrentTip;
  uint8_t lastselected = selected;
  int8_t arrow = selected ? 1 : 0;
  setRotary(0, NumberOfTips - 1, 1, selected);
  bool lastbutton = !digitalRead(ENC_SW_PIN);

  do
  {
    selected = (uint8_t)getRotary();
    arrow = (int8_t)constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g.firstPage();
    do
    {
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();
      u8g.drawStr(0, 0, "Select Tip");
      u8g.drawStr(0, 16 * (arrow + 1), ">");
      for (uint8_t i = 0; i < 3; i++)
      {
        uint8_t idx = selected + i - arrow;
        if (idx < NumberOfTips)
          u8g.drawStr(12, 16 * (i + 1), TipName[idx]);
      }
    } while (u8g.nextPage());
    if (lastbutton && digitalRead(ENC_SW_PIN))
    {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(ENC_SW_PIN) || lastbutton);

  BUZZ(60, 1);
  CurrentTip = selected;
  activeTrim = tipTrim[CurrentTip]; // v0.5 forgot to load the new tip's trim
}

// ============================================================================
//  DELETE TIP SCREEN
// ============================================================================
void DeleteTipScreen()
{
  if (NumberOfTips == 1)
  {
    MessageScreen(DeleteMessage, 3);
    return;
  }
  if (!MenuScreen(SureItems, 3, 0))
    return;

  if (CurrentTip == NumberOfTips - 1)
  {
    CurrentTip--;
  }
  else
  {
    for (uint8_t i = CurrentTip; i < NumberOfTips - 1; i++)
    {
      memcpy(TipName[i], TipName[i + 1], TIPNAMELENGTH);
      tipTrim[i] = tipTrim[i + 1]; // trim must shift with the tip
    }
  }
  NumberOfTips--;
  activeTrim = tipTrim[CurrentTip];
}

// ============================================================================
//  ADD TIP SCREEN
// ============================================================================
void AddTipScreen()
{
  if (NumberOfTips >= TIPMAX)
  {
    MessageScreen(MaxTipMessage, 1);
    return;
  }
  CurrentTip = NumberOfTips++;
  tipTrim[CurrentTip] = 0;
  activeTrim = 0;
  InputNameScreen();
}

// ============================================================================
//  TIP SCREEN
// ============================================================================
void TipScreen()
{
  uint8_t sel = 0;
  bool repeat = true;
  while (repeat)
  {
    sel = MenuScreen(TipItems, 7, sel);
    switch (sel)
    {
    case 0:
      ChangeTipScreen();
      break;
    case 1:
      CalibrationScreen();
      break;
    case 2:
      InputNameScreen();
      break;
    case 3:
      DeleteTipScreen();
      break;
    case 4:
      AddTipScreen();
      break;
    default:
      repeat = false;
      break;
    }
  }
}

// ============================================================================
//  TEMP SCREEN  (includes Max Power)
// ============================================================================
void TempScreen()
{
  uint8_t sel = 0;
  bool repeat = true;
  while (repeat)
  {
    sel = MenuScreen(TempItems, 6, sel);
    switch (sel)
    {
    case 0:
      setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
      DefaultTemp = InputScreen(DefaultTempItems);
      break;
    case 1:
      setRotary(20, 200, TEMP_STEP, SleepTemp);
      SleepTemp = InputScreen(SleepTempItems);
      break;
    case 2:
      setRotary(10, 100, 5, BoostTemp);
      BoostTemp = (uint8_t)InputScreen(BoostTempItems);
      break;
    case 3:
      setRotary(POWER_MIN, POWER_MAX_ABS, 5, MaxPower);
      MaxPower = (uint8_t)InputScreen(MaxPowerItems);
      break;
    default:
      repeat = false;
      break;
    }
  }
}

// ============================================================================
//  TIMER SCREEN
// ============================================================================
void TimerScreen()
{
  uint8_t sel = 0;
  bool repeat = true;
  while (repeat)
  {
    sel = MenuScreen(TimerItems, 5, sel);
    switch (sel)
    {
    case 0:
      setRotary(0, 30, 1, time2sleep);
      time2sleep = (uint8_t)InputScreen(SleepTimerItems);
      break;
    case 1:
      setRotary(0, 60, 5, time2off);
      time2off = (uint8_t)InputScreen(OffTimerItems);
      break;
    case 2:
      setRotary(0, 180, 10, timeOfBoost);
      timeOfBoost = (uint8_t)InputScreen(BoostTimerItems);
      break;
    default:
      repeat = false;
      break;
    }
  }
}

// ============================================================================
//  GENERAL SCREEN  (PID / Direct control toggle — now actually used)
// ============================================================================
void GeneralScreen()
{
  uint8_t sel = PIDenable ? 1 : 0;
  sel = MenuScreen(ControlTypeItems, 3, sel);
  PIDenable = (sel == 1);
}

// ============================================================================
//  SETUP SCREEN
// ============================================================================
void SetupScreen()
{
  setHeater(0);
  uint16_t savedTemp = SetTemp;
  uint8_t sel = 0;
  bool repeat = true;

  while (repeat)
  {
    sel = MenuScreen(SetupItems, 9, sel);
    switch (sel)
    {
    case 0:
      TipScreen();
      break;
    case 1:
      TempScreen();
      break;
    case 2:
      TimerScreen();
      break;
    case 3:
      GeneralScreen();
      break;
    case 4:
      beepEnable = (bool)MenuScreen(BuzzerItems, 3, (uint8_t)beepEnable);
      break;
    case 5:
      InfoScreen();
      break;
    case 6:
      debugMode = (bool)MenuScreen(DebugItems, 3, (uint8_t)debugMode);
      break;
    default:
      repeat = false;
      break;
    }
  }

  saveSettings();
  userActivity = true; // reset sleep timer only; does NOT wake from standby
  SetTemp = savedTemp;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
  pFullEst = 0; // re-learn after possible changes
}

// ============================================================================
//  CALIBRATION SCREEN
//  Heats to the default temp so you can compare against a reference
//  thermometer, then adjust the trim.  Press = save, long press = cancel.
// ============================================================================
void CalibrationScreen()
{
  globalButtonHandling = false;
  shortPress = false;
  longPress = false;

  uint8_t prevMode = mode;
  uint16_t prevSet = SetTemp;
  mode = MODE_RUN;
  SetTemp = DefaultTemp;

  setRotary(-200, 200, 1, (int)tipTrim[CurrentTip]);

  bool exiting = false;
  uint32_t lastCycle = 0;

  while (!exiting)
  {
    handleButton();

    if (millis() - lastCycle >= SENSOR_INTERVAL)
    {
      lastCycle = millis();
      runControlCycle();
    }

    int16_t trim = (int16_t)getRotary();
    activeTrim = trim; // live preview
    int16_t dispTemp = (int16_t)(SmoothedTEMP + trim);
    if (dispTemp < 0)
      dispTemp = 0;

    u8g.firstPage();
    do
    {
      handleButton();
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();

      u8g.setCursor(0, 0);
      u8g.print(TipName[CurrentTip]);
      u8g.setCursor(62, 0);
      u8g.print(F("CALIBRATE"));
      u8g.drawHLine(0, 14, 128);

      u8g.setFont(u8g2_font_fub25_tf);
      u8g.setFontPosTop();
      if (!tipOK)
      {
        u8g.setCursor(0, 17);
        u8g.print(F("---"));
      }
      else
      {
        if (dispTemp < 10)
          u8g.setCursor(36, 17);
        else if (dispTemp < 100)
          u8g.setCursor(18, 17);
        else
          u8g.setCursor(0, 17);
        u8g.print(dispTemp);
      }

      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();
      u8g.setCursor(82, 17);
      u8g.print(DEG "C now");

      u8g.drawHLine(0, 44, 128);
      u8g.setCursor(0, 48);
      u8g.print(F("Trim:"));
      if (trim >= 0)
        u8g.print(F("+"));
      u8g.print((int)trim);

      u8g.setCursor(80, 48);
      u8g.print(F("Prs=save"));

    } while (u8g.nextPage());

    if (shortPress)
    {
      shortPress = false;
      tipTrim[CurrentTip] = trim;
      activeTrim = trim;
      saveSettings();
      BUZZ(80, 2);
      exiting = true;
    }

    if (longPress)
    {
      longPress = false;
      activeTrim = tipTrim[CurrentTip]; // cancel
      exiting = true;
    }
  }

  setHeater(0);
  mode = prevMode;
  SetTemp = prevSet;
  shortPress = false;
  longPress = false;
  globalButtonHandling = true;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
}

// ============================================================================
//  LOAD / SAVE SETTINGS
// ============================================================================
void loadSettings()
{
  EEPROM.get(0, settings);

  if (settings.signature != EEPROM_IDENT)
  {
    memset(&settings, 0, sizeof(settings));
    settings.signature = EEPROM_IDENT;
    settings.DefaultTemp = TEMP_DEFAULT;
    settings.SleepTemp = TEMP_SLEEP;
    settings.BoostTemp = TEMP_BOOST;
    settings.time2sleep = TIME2SLEEP;
    settings.time2off = TIME2OFF;
    settings.timeOfBoost = TIMEOFBOOST;
    settings.MainScrType = 1;
    settings.PIDenable = PID_ENABLE;
    settings.beepEnable = BEEP_ENABLE;
    settings.CurrentTip = 0;
    settings.NumberOfTips = 1;
    settings.MaxPower = POWER_DEFAULT;
    settings.debugMode = DEBUG_DEFAULT;
    for (uint8_t i = 0; i < TIPMAX; i++)
    {
      settings.TempOffset[i] = 0;
      strncpy(settings.TipName[i], TIPNAME, TIPNAMELENGTH);
      settings.TipName[i][TIPNAMELENGTH - 1] = 0;
    }
    EEPROM.put(0, settings);
    EEPROM.commit();
  }

  DefaultTemp = settings.DefaultTemp;
  SleepTemp = settings.SleepTemp;
  BoostTemp = settings.BoostTemp;
  time2sleep = settings.time2sleep;
  time2off = settings.time2off;
  timeOfBoost = settings.timeOfBoost;
  MainScrType = settings.MainScrType;
  PIDenable = settings.PIDenable;
  beepEnable = settings.beepEnable;
  CurrentTip = settings.CurrentTip;
  NumberOfTips = settings.NumberOfTips;
  MaxPower = settings.MaxPower;
  debugMode = settings.debugMode;

  memcpy(tipTrim, settings.TempOffset, sizeof(tipTrim));
  memcpy(TipName, settings.TipName, sizeof(TipName));

  // Sanity checks
  if (NumberOfTips < 1 || NumberOfTips > TIPMAX)
    NumberOfTips = 1;
  if (CurrentTip >= NumberOfTips)
    CurrentTip = 0;
  if (DefaultTemp < TEMP_MIN || DefaultTemp > TEMP_MAX)
    DefaultTemp = TEMP_DEFAULT;
  if (MaxPower < POWER_MIN || MaxPower > POWER_MAX_ABS)
    MaxPower = POWER_DEFAULT;
  for (uint8_t i = 0; i < TIPMAX; i++)
    TipName[i][TIPNAMELENGTH - 1] = 0;

  activeTrim = tipTrim[CurrentTip];
}

void saveSettings()
{
  settings.signature = EEPROM_IDENT;
  settings.DefaultTemp = DefaultTemp;
  settings.SleepTemp = SleepTemp;
  settings.BoostTemp = BoostTemp;
  settings.time2sleep = time2sleep;
  settings.time2off = time2off;
  settings.timeOfBoost = timeOfBoost;
  settings.MainScrType = MainScrType;
  settings.PIDenable = PIDenable;
  settings.beepEnable = beepEnable;
  settings.CurrentTip = CurrentTip;
  settings.NumberOfTips = NumberOfTips;
  settings.MaxPower = MaxPower;
  settings.debugMode = debugMode;

  memcpy(settings.TempOffset, tipTrim, sizeof(tipTrim));
  memcpy(settings.TipName, TipName, sizeof(TipName));

  EEPROM.put(0, settings);
  EEPROM.commit();
}

// ============================================================================
//  SETUP
// ============================================================================
void setup()
{
  pinMode(VIN_PIN, INPUT_ANALOG);
  pinMode(I_PIN, INPUT_ANALOG);
  pinMode(HANDLE_SENSE_PIN, INPUT_PULLDOWN);
  pinMode(ENABLE_PIN, INPUT_PULLDOWN);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(CTRL_PIN, OUTPUT);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(PB2, OUTPUT);
  pinMode(PC13, OUTPUT);

  digitalWrite(BUZZ_PIN, LOW);
  digitalWrite(PC13, HIGH);

  analogReadResolution(12);
  analogWriteFrequency(1000);
  setHeater(0); // heater OFF on boot

  u8g.begin();
  u8g.enableUTF8Print();

  loadSettings();

  ctrl.SetOutputLimits(0, 255);
  ctrl.SetMode(MANUAL); // Thermostat() enables it when needed
  pidActive = false;

  calibrateCurrentOffset(); // heater is off here

  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), rotaryISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), rotaryISR, CHANGE);

  SetTemp = DefaultTemp;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);

  // Let the MAX6675 finish its first conversion, then prime the filter
  delay(300);
  SENSORCheck();
  SENSORCheck();

  mode = MODE_SLEEP; // straight to standby; press enable to heat
  sleepmillis = millis();

  BUZZ(300, 2);
}

// ============================================================================
//  LOOP
// ============================================================================
void loop()
{
  uint32_t now = millis();

  if (now - sensorPreviousMillis >= SENSOR_INTERVAL)
  {
    sensorPreviousMillis = now;
    runControlCycle(); // sensor -> power/limit -> thermostat
  }

  handleButton();
  processButton();
  handleEnableButton();
  ROTARYCheck();
  SLEEPCheck();
  TipCheck();
  MainScreen();
  LED_Handler();

  // heartbeat on the on-board LED (blue pill: LOW = on)
  digitalWrite(PC13, (now / 500) & 1);
}