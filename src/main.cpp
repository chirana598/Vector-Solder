// ============================================================================
//  Soldering Iron Controller — STM32  v0.5
// ============================================================================
#include <Arduino.h>
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
#define VERSION "v0.5"
#define TIPMAX 8
#define TIPNAMELENGTH 6
#define TIPNAME "C245-i"
#define POWER_SELECTED 65

#define TEMP_MIN 150
#define TEMP_MAX 450
#define TEMP_DEFAULT 275
#define TEMP_SLEEP 150
#define TEMP_BOOST 50
#define TEMP_STEP 5

#define TIME2SLEEP 5   // minutes
#define TIME2OFF 15    // minutes
#define TIMEOFBOOST 40 // seconds

#define TIME2SETTLE 5000    // µs — OpAmp settle time
#define SMOOTHIE 0.2f       // EMA smoothing factor
#define SENSOR_INTERVAL 250 // ms between sensor reads

#define CURRENT_LIMIT 4.2f
#define VOLTAGE_MAX 26.0f
#define VOLTAGE_MIN 5.0f

#define PID_ENABLE true
#define BEEP_ENABLE true
#define DEFAULT_BUZZ_MS 200

#define LONG_PRESS_TIME 650
#define DEBOUNCE_TIME 30

#define STABLE_TOLERANCE 5   // ADC counts
#define STABLE_DURATION 2000 // ms
#define FLASH_INTERVAL 300   // ms

// ============================================================================
//  MODES   0=OFF  1=SLEEP  2=RUN  3=HOLD  4=BOOST
// ============================================================================
#define MODE_OFF 0
#define MODE_SLEEP 1
#define MODE_RUN 2
#define MODE_HOLD 3
#define MODE_BOOST 4

// ============================================================================
//  DATA STRUCTURES
// ============================================================================
struct CalPoint
{
  uint16_t adc;
  uint16_t temp;
};

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
  int16_t TempOffset[TIPMAX]; // per-tip trim, ±200°C range
  char TipName[TIPMAX][TIPNAMELENGTH];
};

// ============================================================================
//  FORWARD DECLARATIONS
// ============================================================================
void rotaryISR();
void setRotary(int rmin, int rmax, int rstep, int rvalue);
int getRotary();
void handleButton();
void processButton();
void ROTARYCheck();
void SENSORCheck();
void Thermostat();
void PowerCheck();
void SLEEPCheck();
void stabilityReset();
void stabilityUpdate();
void LED_Handler();
void BUZZ(long duration = DEFAULT_BUZZ_MS, int buzzcount = 1);
void loadSettings();
void saveSettings();
float calculateTemp(int adc);

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
//  DISPLAY + ENCODER
// ============================================================================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g(U8G2_R0);
RotaryEncoder encoder(ENC_CLK_PIN, ENC_DT_PIN, RotaryEncoder::LatchMode::FOUR3);
MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_MISO);

// ============================================================================
//  MENU STRINGS
// ============================================================================
const char *SetupItems[] = {"Setup Menu", "Tip Settings", "Temp Settings",
                            "Timers", "General", "Buzzer",
                            "Information", "Return"};
const char *TipItems[] = {"Tip:", "Change Tip", "Calibrate Tip",
                          "Rename Tip", "Delete Tip", "Add new Tip", "Return"};
const char *TempItems[] = {"Temp Settings", "Default Temp", "Sleep Temp",
                           "Boost Temp", "Return"};
const char *TimerItems[] = {"Timer Settings", "Sleep Timer", "Off Timer",
                            "Boost Timer", "Return"};
const char *ControlTypeItems[] = {"Control Type", "Direct", "PID"};
const char *StoreItems[] = {"Store Settings?", "No", "Yes"};
const char *SureItems[] = {"Are you sure?", "No", "Yes"};
const char *BuzzerItems[] = {"Buzzer", "Disable", "Enable"};
const char *DefaultTempItems[] = {"Default Temp", "deg C"};
const char *SleepTempItems[] = {"Sleep Temp", "deg C"};
const char *BoostTempItems[] = {"Boost Temp", "deg C"};
const char *SleepTimerItems[] = {"Sleep Timer", "Minutes"};
const char *OffTimerItems[] = {"Off Timer", "Minutes"};
const char *BoostTimerItems[] = {"Boost Timer", "Seconds"};
const char *DeleteMessage[] = {"Warning", "Cannot delete", "your last tip!"};
const char *OverVOLTmessage[] = {"VOLTAGE FAULT", "DISCONNECT"};
const char *OverCURRmessage[] = {"OVER CURRENT", "AUTO CUTOFF"};
const char *MaxTipMessage[] = {"Max tips reached"};
const char *BadCalMessage[] = {"Cal invalid!", "Need 2+ pts,", "ADC & temp", "must increase."};

// ============================================================================
//  GLOBAL STATE
// ============================================================================
uint8_t mode = MODE_RUN;

// Temperature pipeline
float SmoothedTEMP = 0;
double Input = 0, Output = 0, Setpoint = 0;
double RawTemp = 0, CurrentTemp = 0;
uint16_t SetTemp = TEMP_DEFAULT;
float ShowTemp = 0;
uint16_t gap = 0;
bool tempInit = false;

// Power
float VOLTAGE = 0, CURRENT = 0, POWER = 0;

// Tips
CalPoint calTable[TIPMAX][4];
uint8_t numCalPoints[TIPMAX];
char TipName[TIPMAX][TIPNAMELENGTH];
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
bool PIDenable = PID_ENABLE;
bool beepEnable = BEEP_ENABLE;

// Timing
uint32_t sleepmillis = 0;
uint32_t boostmillis = 0;
uint32_t sensorPreviousMillis = 0;
uint32_t buzzerPreviousMillis = 0;
uint8_t goneMinutes = 0;
uint8_t goneSeconds = 0;

volatile bool handleMoved = false;

// ============================================================================
//  PID
// ============================================================================
double aggKp = 30, aggKi = 0.5, aggKd = 5;
double consKp = 12, consKi = 0.4, consKd = 8;
PID ctrl(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

// ============================================================================
//  EEPROM
// ============================================================================
const uint32_t EEPROM_IDENT = 0xBEEFDEB0;
Settings settings;

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

int16_t tipTrim[TIPMAX]; // stored per tip in EEPROM
int16_t activeTrim = 0;  // loaded from tipTrim[CurrentTip], used by calculateTemp
int16_t TempOffset[TIPMAX];

// ============================================================================
//  BUTTON
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

void processButton()
{
  if (!globalButtonHandling)
    return;

  if (shortPress)
  {
    shortPress = false;
    if (beepEnable)
      BUZZ(40, 1);

    if (mode == MODE_OFF || mode == MODE_SLEEP)
    {
      mode = MODE_RUN;
      sleepmillis = millis();
      return;
    }
    mode = (mode == MODE_BOOST) ? MODE_RUN : MODE_BOOST;
    boostmillis = millis();
    handleMoved = true;
  }

  if (longPress)
  {
    longPress = false;
    if (beepEnable)
      BUZZ(120, 2); // beep here, not inside handleButton
    SetupScreen();
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

void SENSORCheck()
{
  float rawTemp = thermocouple.readCelsius();

  if (!tempInit)
  {
    SmoothedTEMP = rawTemp;
    tempInit = true;
  }
  else
  {
    SmoothedTEMP += SMOOTHIE * (rawTemp - SmoothedTEMP);
  }

  CurrentTemp = SmoothedTEMP;
}

// ============================================================================
//  THERMOSTAT  (adaptive PID)
// ============================================================================

bool pidActive = false;

void Thermostat()
{
  if (mode == MODE_OFF)
  {
    analogWrite(CTRL_PIN, 0);
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

  Input = CurrentTemp;
  gap = (uint16_t)fabsf(Setpoint - Input);

  bool nearTarget = (gap < 15);
  ctrl.SetTunings(
      nearTarget ? consKp : aggKp,
      nearTarget ? consKi : aggKi,
      nearTarget ? consKd : aggKd);
  ctrl.Compute();

  if (mode == MODE_RUN && gap <= 3)
    mode = MODE_HOLD;
  else if (mode == MODE_HOLD && gap >= 6)
    mode = MODE_RUN;

  analogWrite(CTRL_PIN, (uint8_t)Output);
}

// ============================================================================
//  ROTARY CHECK
// ============================================================================
void ROTARYCheck()
{
  SetTemp = (uint16_t)getRotary();

  if (mode == MODE_BOOST && timeOfBoost)
  {
    goneSeconds = (uint8_t)((millis() - boostmillis) / 1000);
    if (goneSeconds >= timeOfBoost)
    {
      mode = MODE_RUN;
      if (beepEnable)
        BUZZ(200, 1);
    }
  }
}

// ============================================================================
//  SLEEP CHECK
// ============================================================================
void SLEEPCheck()
{
  bool onStand = digitalRead(HANDLE_SENSE_PIN); // active LOW = on stand

  if (onStand)
  {
    if (mode == MODE_RUN || mode == MODE_BOOST || mode == MODE_HOLD)
    {
      mode = MODE_SLEEP;
      analogWrite(CTRL_PIN, 0);
      if (beepEnable)
        BUZZ(150, 1);
    }
    return;
  }

  if (handleMoved)
  {
    if (mode == MODE_SLEEP || mode == MODE_OFF)
    {
      mode = MODE_RUN;
      if (beepEnable)
        BUZZ(100, 1);
    }
    handleMoved = false;
    sleepmillis = millis();
  }

  goneMinutes = (uint8_t)((millis() - sleepmillis) / 60000UL);

  if (mode != MODE_SLEEP && mode != MODE_OFF && time2sleep > 0 && goneMinutes >= time2sleep)
  {
    mode = MODE_SLEEP;
    analogWrite(CTRL_PIN, 0);
    if (beepEnable)
      BUZZ(400, 2);
  }

  if (mode != MODE_OFF && time2off > 0 && goneMinutes >= time2off)
  {
    mode = MODE_OFF;
    analogWrite(CTRL_PIN, 0);
    if (beepEnable)
      BUZZ(600, 1);
  }
}

// ============================================================================
//  POWER CHECK
// ============================================================================
void PowerCheck()
{
  long rawV = analogRead(VIN_PIN);
  long rawI = analogRead(I_PIN);

  VOLTAGE = (rawV * 3.3f / 4095.0f) / 0.09f;

  if (rawI < 1365)
    CURRENT = map(rawI, 0, 1365, 0, 800) / 1000.0f;
  else if (rawI < 2730)
    CURRENT = map(rawI, 1365, 2730, 800, 2500) / 1000.0f;
  else
    CURRENT = map(rawI, 2730, 4095, 2500, 4200) / 1000.0f;

  POWER = VOLTAGE * CURRENT;

  if (CURRENT >= CURRENT_LIMIT)
  {
    mode = MODE_OFF;
    analogWrite(CTRL_PIN, 0);
    MessageScreen(OverCURRmessage, 2);
  }
  /*
   if (VOLTAGE >= VOLTAGE_MAX || (VOLTAGE > 0.5f && VOLTAGE <= VOLTAGE_MIN))
   {
     mode = MODE_OFF;
     analogWrite(CTRL_PIN, 0);
     MessageScreen(OverVOLTmessage, 2);
   }
     */
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
  SetTemp = (uint16_t)getRotary();
  uint8_t pwmPct = (uint8_t)map((long)Output, 0, 255, 0, 100);

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
    // clang-format off
 
    u8g.drawHLine(0, 14, 128);

    // ── Centre: large current temperature ───────────────────────────────
    u8g.setFont(u8g2_font_fub25_tf);
    u8g.setFontPosTop();

    if (CurrentTemp > TEMP_MAX)
    {
      u8g.setCursor(35, 17);
      u8g.print(F("OVT"));
    }
    else
    {
      u8g.setCursor((CurrentTemp < 10) ? 100 : (CurrentTemp < 100) ? 68: 48,17);
      u8g.print((uint16_t)CurrentTemp);
    }

    // °C — small font, top-right of temp area
    u8g.setFont(u8g2_font_9x15_tf);
    u8g.setFontPosTop();
    u8g.setCursor(106, 30);
    u8g.print(F("\xb0""C"));

    // ── Bottom bar ───────────────────────────────────────────────────────
    u8g.drawHLine(0, 48, 128);

    // PWM%
    u8g.setCursor(0, 51);
    u8g.print(POWER_SELECTED);
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
    if (SetTemp < 100)
    {
      u8g.setCursor(89, 51);
    }
    else
    {
      u8g.setCursor(80, 51);
    }
    u8g.print(SetTemp);
    u8g.print(F("\xb0""C"));

    // Boost countdown
    if (mode == MODE_BOOST && timeOfBoost > 0)
    {
      u8g.setCursor(50, 0);
      u8g.print(timeOfBoost - goneSeconds);
      u8g.print(F("s"));
    }

    // clang-format on

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

  if (beepEnable)
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
//  INFO SCREEN
// ============================================================================
void InfoScreen()
{
  bool exiting = false;
  shortPress = false;
  longPress = false;
  globalButtonHandling = false;

  while (!exiting)
  {
    handleButton();

    // Take a fresh sensor read every iteration for live ADC display
    analogWrite(CTRL_PIN, 0);
    delayMicroseconds(TIME2SETTLE);
    float liveADC = SmoothedTEMP;
    analogWrite(CTRL_PIN, (mode == MODE_OFF) ? 0 : (uint8_t)Output);

    u8g.firstPage();
    do
    {
      handleButton();
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();

      // Row 0 — firmware version
      u8g.setCursor(0, 0);
      u8g.print(F("FW: "));
      u8g.print(F(VERSION));

      // Row 1 — raw ADC (live)
      u8g.setCursor(0, 13);
      u8g.print(F("ADC: "));
      u8g.print((uint16_t)liveADC);
      u8g.print(F("  /4095"));

      // Row 2 — calculated temperature
      u8g.setCursor(0, 26);
      u8g.print(F("TMP: "));
      u8g.print((uint16_t)CurrentTemp);
      u8g.print(F("\xb0"
                  "C  trim:"));

      // Row 3 — voltage and current
      u8g.setCursor(0, 39);
      u8g.print(VOLTAGE, 1);
      u8g.print(F("V "));
      u8g.print(CURRENT, 2);
      u8g.print(F("A "));
      u8g.print(POWER, 1);
      u8g.print(F("W"));

      // Row 4 — PID output and mode
      u8g.setCursor(0, 52);
      u8g.print(F("OUT:"));
      u8g.print((uint8_t)map((long)Output, 0, 255, 0, 100));
      u8g.print(F("% "));
      switch (mode)
      {
      case MODE_OFF:
        u8g.print(F("OFF"));
        break;
      case MODE_SLEEP:
        u8g.print(F("SLEEP"));
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
    {
      shortPress = false;
      longPress = false;
      exiting = true;
    }
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

  if (beepEnable)
    BUZZ(60, 1);
  CurrentTip = selected;
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
      memcpy(calTable[i], calTable[i + 1], sizeof(CalPoint) * 4);
      numCalPoints[i] = numCalPoints[i + 1];
    }
  }
  NumberOfTips--;
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
  InputNameScreen();
  calTable[CurrentTip][0] = {800, 150};
  calTable[CurrentTip][1] = {1600, 220};
  calTable[CurrentTip][2] = {2800, 310};
  calTable[CurrentTip][3] = {3800, 390};
  numCalPoints[CurrentTip] = 4;
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
//  TEMP SCREEN
// ============================================================================
void TempScreen()
{
  uint8_t sel = 0;
  bool repeat = true;
  while (repeat)
  {
    sel = MenuScreen(TempItems, 5, sel);
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
//  GENERAL SCREEN  (PID / Direct control toggle)
// ============================================================================
void GeneralScreen()
{
  uint8_t sel = PIDenable ? 1 : 0;
  sel = MenuScreen(ControlTypeItems, 3, sel);
  if (sel == 0)
    PIDenable = false;
  if (sel == 1)
    PIDenable = true;
}

// ============================================================================
//  SETUP SCREEN
// ============================================================================
void SetupScreen()
{
  analogWrite(CTRL_PIN, 0);
  uint16_t savedTemp = SetTemp;
  uint8_t sel = 0;
  bool repeat = true;

  while (repeat)
  {
    sel = MenuScreen(SetupItems, 8, sel);
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
    default:
      repeat = false;
      break;
    }
  }

  saveSettings();
  handleMoved = true;
  SetTemp = savedTemp;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
}

// ============================================================================
//  CALIBRATION SCREEN
// ============================================================================

void CalibrationScreen()
{
  globalButtonHandling = false;
  shortPress = false;
  longPress = false;

  // Start encoder at current tip's trim value
  setRotary(-200, 200, 1, (int)tipTrim[CurrentTip]);

  bool exiting = false;

  while (!exiting)
  {
    handleButton();

    int16_t trim = (int16_t)getRotary();
    activeTrim = trim; // live preview while adjusting
    uint16_t dispTemp = (uint16_t)CurrentTemp;

    u8g.firstPage();
    do
    {
      handleButton();
      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();

      // ── Top bar ───────────────────────────────────────────────────────
      u8g.setCursor(0, 0);
      u8g.print(TipName[CurrentTip]);
      u8g.setCursor(62, 0);
      u8g.print(F("CALIBRATE"));
      u8g.drawHLine(0, 14, 128);

      // ── Live temperature (large) ──────────────────────────────────────
      u8g.setFont(u8g2_font_fub25_tf);
      u8g.setFontPosTop();
      if (dispTemp < 10)
        u8g.setCursor(36, 17);
      else if (dispTemp < 100)
        u8g.setCursor(18, 17);
      else
        u8g.setCursor(0, 17);
      u8g.print(dispTemp);

      u8g.setFont(u8g2_font_9x15_tf);
      u8g.setFontPosTop();
      u8g.setCursor(82, 17);
      u8g.print(F("\xb0"
                  "C now"));

      // ── Bottom bar ────────────────────────────────────────────────────
      u8g.drawHLine(0, 44, 128);
      u8g.setCursor(0, 48);
      u8g.print(F("Trim:"));
      if (trim >= 0)
        u8g.print(F("+"));
      u8g.print((int)trim);
      u8g.print(F("\xb0"
                  "C"));

      u8g.setCursor(80, 48);
      u8g.print(F("Prs=save"));

    } while (u8g.nextPage());

    if (shortPress)
    {
      shortPress = false;
      tipTrim[CurrentTip] = trim;
      activeTrim = trim;
      saveSettings();
      if (beepEnable)
        BUZZ(80, 2);
      exiting = true;
    }

    if (longPress)
    {
      longPress = false;
      // Cancel — restore old trim
      activeTrim = tipTrim[CurrentTip];
      exiting = true;
    }
  }

  shortPress = false;
  longPress = false;
  globalButtonHandling = true;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
}

// ============================================================================
//  LOAD SETTINGS
// ============================================================================
void loadSettings()
{
  EEPROM.get(0, settings);

  if (settings.signature != EEPROM_IDENT)
  {
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
    for (uint8_t i = 0; i < TIPMAX; i++)
    {
      settings.TempOffset[i] = 0;
      strncpy(settings.TipName[i], TIPNAME, TIPNAMELENGTH);
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

  memcpy(tipTrim, settings.TempOffset, sizeof(tipTrim));
  memcpy(TipName, settings.TipName, sizeof(TipName));

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

  digitalWrite(BUZZ_PIN, LOW);
  analogWrite(CTRL_PIN, 0); // heater OFF on boot

  u8g.begin();
  u8g.enableUTF8Print();

  analogReadResolution(12);
  analogWriteFrequency(1000);

  // PID: output 0=full heat, 255=off  → limits reversed
  ctrl.SetOutputLimits(0, 255);
  ctrl.SetMode(AUTOMATIC);

  loadSettings();

  activeTrim = tipTrim[CurrentTip];

  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), rotaryISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), rotaryISR, CHANGE);

  SetTemp = DefaultTemp;
  mode = MODE_RUN; // start in RUN, heater will activate immediately
  pidActive = false;
  sleepmillis = millis();

  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);

  digitalWrite(PC13, HIGH);
  delay(3000);
  digitalWrite(PC13, LOW);

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
    SENSORCheck();
    Thermostat();
    PowerCheck();
  }

  handleButton();
  processButton();
  ROTARYCheck();
  MainScreen();
  LED_Handler();

  digitalWrite(PC13, HIGH);
}