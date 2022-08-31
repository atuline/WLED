#pragma once

#include "wled.h"

//
// Inspired by the v1 usermods
// * rotary_encoder_change_brightness
// * rotary_encoder_change_effect
//
// v2 usermod that provides a rotary encoder-based UI.
//
// This usermod allows you to control:
//
// * Brightness
// * Selected Effect
// * Effect Speed
// * Effect Intensity
// * Palette
//
// Change between modes by pressing a button.
//
// Dependencies
// * This usermod REQURES the ModeSortUsermod
// * This Usermod works best coupled with
//   FourLineDisplayUsermod.
//

#ifndef ENCODER_DT_PIN
#define ENCODER_DT_PIN 12 //12 needed for bootstrap... (no current at boot) overwrite in env for other values
#endif

#ifndef ENCODER_CLK_PIN
#define ENCODER_CLK_PIN 14
#endif

#ifndef ENCODER_SW_PIN
#define ENCODER_SW_PIN 13
#endif

#ifndef USERMOD_FOUR_LINE_DISPLAY
// These constants won't be defined if we aren't using FourLineDisplay.
#define FLD_LINE_BRIGHTNESS       0
#define FLD_LINE_MODE             0
#define FLD_LINE_EFFECT_SPEED     0
#define FLD_LINE_EFFECT_INTENSITY 0
#define FLD_LINE_EFFECT_CUSTOM1      0 //WLEDSR
#define FLD_LINE_EFFECT_CUSTOM2      0 //WLEDSR
#define FLD_LINE_EFFECT_CUSTOM3      0 //WLEDSR
#define FLD_LINE_PALETTE          0
#define FLD_LINE_SQUELCH          0 //WLEDSR
#define FLD_LINE_GAIN             0 //WLEDSR

char sliderNames[5][LINE_BUFFER_SIZE*2] = {"FX Speed", "FX Intens.", "FX Custom1", "FX Custom2", "FX Custom3"}; //WLEDSR

#endif


// The last UI state
#define LAST_UI_STATE 9 //brightness until sample gain


class RotaryEncoderUIUsermod : public Usermod {
private:
  int fadeAmount = 10;             // Amount to change every step (brightness)
  unsigned long currentTime;
  unsigned long loopTime;
  //  -D ENCODER_DT_PIN=26 -D ENCODER_CLK_PIN=27 -D ENCODER_SW_PIN=33
  int8_t pinA = ENCODER_DT_PIN;       // DT from encoder
  int8_t pinB = ENCODER_CLK_PIN;      // CLK from encoder
  int8_t pinC = ENCODER_SW_PIN;       // SW from encoder
  unsigned char select_state = 0;     // 0: brightness, 1: effect, 2: effect speed, ...
  unsigned char button_state = HIGH;
  unsigned char prev_button_state = HIGH;

#ifdef USERMOD_FOUR_LINE_DISPLAY
  FourLineDisplayUsermod *display;
#else
  void* display = nullptr;
#endif

  char **modes_qstrings = nullptr;        //WLEDSR (HarryB-210729): Add invalid effect index skip
  byte *modes_alpha_indexes = nullptr;
  byte *palettes_alpha_indexes = nullptr;

  unsigned char Enc_A;
  unsigned char Enc_B;
  unsigned char Enc_A_prev = 0;

  bool currentEffectAndPaletteInitialized = false;
  uint8_t effectCurrentIndex = 0;
  uint8_t effectPaletteIndex = 0;

  bool initDone = false;
  bool enabled = true;

  // strings to reduce flash memory usage (used more than twice)
  static const char _name[];
  static const char _enabled[];
  static const char _DT_pin[];
  static const char _CLK_pin[];
  static const char _SW_pin[];

public:
  /*
     * setup() is called once at boot. WiFi is not yet connected at this point.
     * You can use it to initialize variables, sensors or similar.
     */
  void setup()
  {
    PinManagerPinType pins[3] = { { pinA, false }, { pinB, false }, { pinC, false } };
    if (!pinManager.allocateMultiplePins(pins, 3, PinOwner::UM_RotaryEncoderUI)) {
      // BUG: configuring this usermod with conflicting pins
      //      will cause it to de-allocate pins it does not own
      //      (at second config)
      //      This is the exact type of bug solved by pinManager
      //      tracking the owner tags....
      pinA = pinB = pinC = -1;
      enabled = false;
      return;
    }

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinC, INPUT_PULLUP);
    currentTime = millis();
    loopTime = currentTime;

    ModeSortUsermod *modeSortUsermod = (ModeSortUsermod*) usermods.lookup(USERMOD_ID_MODE_SORT);
    modes_qstrings = modeSortUsermod->getModesQStrings(); //WLEDSR (HarryB-210729): Add invalid effect index skip
    modes_alpha_indexes = modeSortUsermod->getModesAlphaIndexes();
    palettes_alpha_indexes = modeSortUsermod->getPalettesAlphaIndexes();

#ifdef USERMOD_FOUR_LINE_DISPLAY
    // This Usermod uses FourLineDisplayUsermod for the best experience.
    // But it's optional. But you want it.
    display = (FourLineDisplayUsermod*) usermods.lookup(USERMOD_ID_FOUR_LINE_DISP);
    if (display != nullptr) {
      display->setLineType(FLD_LINE_BRIGHTNESS);
      display->setMarkLine(3);
    }
#endif

    initDone = true;
  }

  /*
     * connected() is called every time the WiFi is (re)connected
     * Use it to initialize network interfaces
     */
  void connected()
  {
    //Serial.println("Connected to WiFi!");
  }

  /*
     * loop() is called continuously. Here you can check for events, read sensors, etc.
     *
     * Tips:
     * 1. You can use "if (WLED_CONNECTED)" to check for a successful network connection.
     *    Additionally, "if (WLED_MQTT_CONNECTED)" is available to check for a connection to an MQTT broker.
     *
     * 2. Try to avoid using the delay() function. NEVER use delays longer than 10 milliseconds.
     *    Instead, use a timer check as shown here.
     */
  void loop()
  {
    if (!enabled) return;

    currentTime = millis(); // get the current elapsed time

    // Initialize effectCurrentIndex and effectPaletteIndex to
    // current state. We do it here as (at least) effectCurrent
    // is not yet initialized when setup is called.
    if (!currentEffectAndPaletteInitialized) {
      findCurrentEffectAndPalette();
    }

    if (currentTime >= (loopTime + 2)) // 2ms since last check of encoder = 500Hz
    {
      button_state = digitalRead(pinC);
      if (prev_button_state != button_state)
      {
        if (button_state == LOW)
        {
          prev_button_state = button_state;

          char newState = select_state + 1;

          //WLEDSR: Skip slider if not used
          if (newState == 2 && strlen_P(sliderNames[0]) == 0)
            newState++;
          if (newState == 3 && strlen_P(sliderNames[1]) == 0)
            newState++;
          if (newState == 4 && strlen_P(sliderNames[2]) == 0)
            newState++;
          if (newState == 5 && strlen_P(sliderNames[3]) == 0)
            newState++;
          if (newState == 6 && strlen_P(sliderNames[4]) == 0)
            newState++;

          if (newState > LAST_UI_STATE) newState = 0;

          bool changedState = true;
          if (display != nullptr) {
            switch(newState) {
              case 0:
                changedState = changeState("Brightness", FLD_LINE_BRIGHTNESS, 3);
                break;
              case 1:
                changedState = changeState("Select FX", FLD_LINE_MODE, 2);
                break;
              case 2:
                changedState = changeState(sliderNames[0], FLD_LINE_EFFECT_SPEED, 3);
                break;
              case 3:
                changedState = changeState(sliderNames[1], FLD_LINE_EFFECT_INTENSITY, 3);
                break;
              case 4:
                changedState = changeState(sliderNames[2], FLD_LINE_EFFECT_CUSTOM1, 3); //WLEDSR
                break;
              case 5:
                changedState = changeState(sliderNames[3], FLD_LINE_EFFECT_CUSTOM2, 3); //WLEDSR
                break;
              case 6:
                changedState = changeState(sliderNames[4], FLD_LINE_EFFECT_CUSTOM3, 3); //WLEDSR
                break;
              case 7:
                changedState = changeState("Palette", FLD_LINE_PALETTE, 3);
                break;
              case 8:
                changedState = changeState("Squelch", FLD_LINE_SQUELCH, 3);
                break;
              case 9:
                changedState = changeState("Gain", FLD_LINE_GAIN, 3);
                break;
            }
          }
          if (changedState) {
            select_state = newState;
          }
        }
        else
        {
          prev_button_state = button_state;
        }
      }
      int Enc_A = digitalRead(pinA); // Read encoder pins
      int Enc_B = digitalRead(pinB);
      if ((!Enc_A) && (Enc_A_prev))
      { // A has gone from high to low
        if (Enc_B == HIGH)
        { // B is high so clockwise
          switch(select_state) {
            case 0:
              changeBrightness(true);
              break;
            case 1:
              changeEffect(true);
              break;
            case 2:
              changeEffectSpeed(true);
              break;
            case 3:
              changeEffectIntensity(true);
              break;
            case 4:
              changeEffectCustom1(true); //WLEDSR
              break;
            case 5:
              changeEffectCustom2(true); //WLEDSR
              break;
            case 6:
              changeEffectCustom3(true); //WLEDSR
              break;
            case 7:
              changePalette(true);
              break;
            case 8:
              changeSquelch(true);
              break;
            case 9:
              changeGain(true);
              break;
          }
        }
        else if (Enc_B == LOW)
        { // B is low so counter-clockwise
          switch(select_state) {
            case 0:
              changeBrightness(false);
              break;
            case 1:
              changeEffect(false);
              break;
            case 2:
              changeEffectSpeed(false);
              break;
            case 3:
              changeEffectIntensity(false);
              break;
            case 4:
              changeEffectCustom1(false); //WLEDSR
              break;
            case 5:
              changeEffectCustom2(false); //WLEDSR
              break;
            case 6:
              changeEffectCustom3(false); //WLEDSR
              break;
            case 7:
              changePalette(false);
              break;
            case 8:
              changeSquelch(false);
              break;
            case 9:
              changeGain(false);
              break;
          }
        }
      }
      Enc_A_prev = Enc_A;     // Store value of A for next time
      loopTime = currentTime; // Updates loopTime
    }
  }

  void findCurrentEffectAndPalette() {
    currentEffectAndPaletteInitialized = true;
    for (uint8_t i = 0; i < strip.getModeCount(); i++) {
      //byte value = modes_alpha_indexes[i];
      if (modes_alpha_indexes[i] == effectCurrent) {
        effectCurrentIndex = i;
        break;
      }
    }

    for (uint8_t i = 0; i < strip.getPaletteCount(); i++) {
      //byte value = palettes_alpha_indexes[i];
      if (palettes_alpha_indexes[i] == strip.getSegment(0).palette) {
        effectPaletteIndex = i;
        break;
      }
    }
  }

  boolean changeState(const char *stateName, byte lineThreeMode, byte markedLine) {
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display != nullptr) {
      if (display->wakeDisplay()) {
        // Throw away wake up input
        return false;
      }
      display->overlay("Mode change", stateName, 1500);
      display->setLineType(lineThreeMode);
      display->setMarkLine(markedLine);
    }
  #endif
    return true;
  }

  void lampUdated() {
    colorUpdated(CALL_MODE_BUTTON);
    updateInterfaces(CALL_MODE_BUTTON);
  }

  void changeBrightness(bool increase) {
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    if (increase) {
      bri = (bri + fadeAmount <= 255) ? (bri + fadeAmount) : 255;
    }
    else {
      bri = (bri - fadeAmount >= 0) ? (bri - fadeAmount) : 0;
    }
    lampUdated();
  }

  void changeEffect(bool increase) {
    char reservedEffectsName[] = "Reserved";  //WLEDSR(HarryB-210729) Add invalid effect index skip
    bool isReservedEffectIndex = true;  //WLEDSR(HarryB-210729) Add invalid effect index skip
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    do {    //WLEDSR(HarryB-210729) Add invalid effect index skip  ** repeats increase/decrease until valid effectindex reached
      if (increase) {
        effectCurrentIndex = (effectCurrentIndex + 1 >= strip.getModeCount()) ? 0 : (effectCurrentIndex + 1);
      }
      else {
        effectCurrentIndex = (effectCurrentIndex - 1 < 0) ? (strip.getModeCount() - 1) : (effectCurrentIndex - 1);
      }
      for (int j=0; (j<8 && isReservedEffectIndex); j++) {    //WLEDSR(HarryB-210729) Add invalid effect index skip  *** checks single characters (max 8) of new effectname until different from "Reserved"
        isReservedEffectIndex = reservedEffectsName[j] == pgm_read_byte_near(modes_qstrings[modes_alpha_indexes[effectCurrentIndex]]+j);    //WLEDSR(HarryB-210729) Add invalid effect index skip
      }
    } while (isReservedEffectIndex);   //WLEDSR(HarryB-210729) Add invalid effect index skip
    effectCurrent = modes_alpha_indexes[effectCurrentIndex];
    lampUdated();
  }

  void changeEffectSpeed(bool increase) {
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    if (increase) {
      effectSpeed = (effectSpeed + fadeAmount <= 255) ? (effectSpeed + fadeAmount) : 255;
    }
    else {
      effectSpeed = (effectSpeed - fadeAmount >= 0) ? (effectSpeed - fadeAmount) : 0;
    }
    lampUdated();
  }

  void changeEffectIntensity(bool increase) {
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    if (increase) {
      effectIntensity = (effectIntensity + fadeAmount <= 255) ? (effectIntensity + fadeAmount) : 255;
    }
    else {
      effectIntensity = (effectIntensity - fadeAmount >= 0) ? (effectIntensity - fadeAmount) : 0;
    }
    lampUdated();
  }

  void changeEffectCustom1(bool increase) { //WLEDSR
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    if (increase) {
      effectCustom1 = (effectCustom1 + fadeAmount <= 255) ? (effectCustom1 + fadeAmount) : 255;
    }
    else {
      effectCustom1 = (effectCustom1 - fadeAmount >= 0) ? (effectCustom1 - fadeAmount) : 0;
    }
    lampUdated();
  }

  void changeEffectCustom2(bool increase) { //WLEDSR
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    if (increase) {
      effectCustom2 = (effectCustom2 + fadeAmount <= 255) ? (effectCustom2 + fadeAmount) : 255;
    }
    else {
      effectCustom2 = (effectCustom2 - fadeAmount >= 0) ? (effectCustom2 - fadeAmount) : 0;
    }
    lampUdated();
  }

  void changeEffectCustom3(bool increase) { //WLEDSR
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    if (increase) {
      effectCustom3 = (effectCustom3 + fadeAmount <= 255) ? (effectCustom3 + fadeAmount) : 255;
    }
    else {
      effectCustom3 = (effectCustom3 - fadeAmount >= 0) ? (effectCustom3 - fadeAmount) : 0;
    }
    lampUdated();
  }

  void changePalette(bool increase) {
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    if (increase) {
      effectPaletteIndex = (effectPaletteIndex + 1 >= strip.getPaletteCount()) ? 0 : (effectPaletteIndex + 1);
    }
    else {
      effectPaletteIndex = (effectPaletteIndex - 1 < 0) ? (strip.getPaletteCount() - 1) : (effectPaletteIndex - 1);
    }
    effectPalette = palettes_alpha_indexes[effectPaletteIndex];
    lampUdated();
  }

  void changeSquelch(bool increase) {
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    if (increase) {
      soundSquelch = (soundSquelch + 1 <= 255) ? (soundSquelch + 1) : 255;
    }
    else {
      soundSquelch = (soundSquelch - 1 >= 0) ? (soundSquelch - 1) : 0;
    }
    lampUdated();
  }

  void changeGain(bool increase) {
#ifdef USERMOD_FOUR_LINE_DISPLAY
    if (display && display->wakeDisplay()) {
      // Throw away wake up input
      return;
    }
#endif
    if (increase) {
      sampleGain = (sampleGain + 1 <= 255) ? (sampleGain + 1) : 255;
    }
    else {
      sampleGain = (sampleGain - 1 >= 0) ? (sampleGain - 1) : 0;
    }
    lampUdated();
  }

  /*
     * addToJsonInfo() can be used to add custom entries to the /json/info part of the JSON API.
     * Creating an "u" object allows you to add custom key/value pairs to the Info section of the WLED web UI.
     * Below it is shown how this could be used for e.g. a light sensor
     */
  /*
    void addToJsonInfo(JsonObject& root)
    {
      int reading = 20;
      //this code adds "u":{"Light":[20," lux"]} to the info object
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      JsonArray lightArr = user.createNestedArray("Light"); //name
      lightArr.add(reading); //value
      lightArr.add(" lux"); //unit
    }
    */

  /*
     * addToJsonState() can be used to add custom entries to the /json/state part of the JSON API (state object).
     * Values in the state object may be modified by connected clients
     */
  void addToJsonState(JsonObject &root)
  {
    //root["user0"] = userVar0;
  }

  /*
     * readFromJsonState() can be used to receive data clients send to the /json/state part of the JSON API (state object).
     * Values in the state object may be modified by connected clients
     */
  void readFromJsonState(JsonObject &root)
  {
    //userVar0 = root["user0"] | userVar0; //if "user0" key exists in JSON, update, else keep old value
    //if (root["bri"] == 255) Serial.println(F("Don't burn down your garage!"));
  }

  /**
   * addToConfig() (called from set.cpp) stores persistent properties to cfg.json
   */
  void addToConfig(JsonObject &root) {
    // we add JSON object: {"Rotary-Encoder":{"DT-pin":12,"CLK-pin":14,"SW-pin":13}}
    JsonObject top = root.createNestedObject(FPSTR(_name)); // usermodname
    top[FPSTR(_enabled)] = enabled;
    top[FPSTR(_DT_pin)]  = pinA;
    top[FPSTR(_CLK_pin)] = pinB;
    top[FPSTR(_SW_pin)]  = pinC;
    DEBUG_PRINTLN(F("Rotary Encoder config saved."));
  }

  /**
   * readFromConfig() is called before setup() to populate properties from values stored in cfg.json
   *
   * The function should return true if configuration was successfully loaded or false if there was no configuration.
   */
  bool readFromConfig(JsonObject &root) {
    // we look for JSON object: {"Rotary-Encoder":{"DT-pin":12,"CLK-pin":14,"SW-pin":13}}
    JsonObject top = root[FPSTR(_name)];
    if (top.isNull()) {
      DEBUG_PRINT(FPSTR(_name));
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }
    int8_t newDTpin  = pinA;
    int8_t newCLKpin = pinB;
    int8_t newSWpin  = pinC;

    enabled   = top[FPSTR(_enabled)] | enabled;
    newDTpin  = top[FPSTR(_DT_pin)]  | newDTpin;
    newCLKpin = top[FPSTR(_CLK_pin)] | newCLKpin;
    newSWpin  = top[FPSTR(_SW_pin)]  | newSWpin;

    DEBUG_PRINT(FPSTR(_name));
    if (!initDone) {
      // first run: reading from cfg.json
      pinA = newDTpin;
      pinB = newCLKpin;
      pinC = newSWpin;
      DEBUG_PRINTLN(F(" config loaded."));
    } else {
      DEBUG_PRINTLN(F(" config (re)loaded."));
      // changing parameters from settings page
      if (pinA!=newDTpin || pinB!=newCLKpin || pinC!=newSWpin) {
        pinManager.deallocatePin(pinA, PinOwner::UM_RotaryEncoderUI);
        pinManager.deallocatePin(pinB, PinOwner::UM_RotaryEncoderUI);
        pinManager.deallocatePin(pinC, PinOwner::UM_RotaryEncoderUI);
        pinA = newDTpin;
        pinB = newCLKpin;
        pinC = newSWpin;
        if (pinA<0 || pinB<0 || pinC<0) {
          enabled = false;
          return true;
        }
        setup();
      }
    }
    // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
    return !top[FPSTR(_enabled)].isNull();
  }

  /*
     * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
     * This could be used in the future for the system to determine whether your usermod is installed.
     */
  uint16_t getId()
  {
    return USERMOD_ID_ROTARY_ENC_UI;
  }
};

// strings to reduce flash memory usage (used more than twice)
const char RotaryEncoderUIUsermod::_name[]     PROGMEM = "Rotary-Encoder";
const char RotaryEncoderUIUsermod::_enabled[]  PROGMEM = "enabled";
const char RotaryEncoderUIUsermod::_DT_pin[]   PROGMEM = "DT-pin";
const char RotaryEncoderUIUsermod::_CLK_pin[]  PROGMEM = "CLK-pin";
const char RotaryEncoderUIUsermod::_SW_pin[]   PROGMEM = "SW-pin";
