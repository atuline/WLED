#include "wled.h"
#include "audio_reactive.h"
/*
 * This v1 usermod file allows you to add own functionality to WLED more easily
 * See: https://github.com/Aircoookie/WLED/wiki/Add-own-functionality
 * EEPROM bytes 2750+ are reserved for your custom use case. (if you extend #define EEPSIZE in const.h)
 * If you just need 8 bytes, use 2551-2559 (you do not need to increase EEPSIZE)
 *
 * Consider the v2 usermod API if you need a more advanced feature set!
 */

/*
 * Functions and variable delarations moved to audio_reactive.h
 * Not 100% sure this was done right. There is probably a better way to handle this...
 */

// These variables are feeding the "Info" Page
static unsigned long last_UDPTime = 0;    // time of last valid UDP sound sync datapacket
static float maxSample5sec = 0.0f;        // max sample (after AGC) in last 5 seconds 
static unsigned long sampleMaxTimer = 0;  // last time maxSample5sec was reset
static int receivedFormat = 0;            // last received UDP sound sync format - 0=none, 1=v1 (0.13.x), 2=v2 (0.14.x)
#define CYCLE_SAMPLEMAX 2500              // time window for merasuring

// This gets called once at boot. Do all initialization that doesn't depend on network here
void userSetup() {
  disableSoundProcessing = true; // just to be safe
  // Reset I2S peripheral for good measure
  i2s_driver_uninstall(I2S_NUM_0);
  delay(100); // Give this peripheral time to disable to avoid an indeterminate state.
  periph_module_reset(PERIPH_I2S0_MODULE);

  delay(100);         // Give that poor microphone some time to setup.
  switch (dmType) {
    case 1:
      Serial.print("AS: Generic I2S Microphone - "); Serial.println(I2S_MIC_CHANNEL_TEXT);
      audioSource = new I2SSource(SAMPLE_RATE, BLOCK_SIZE, 0, 0xFFFFFFFF);
      break;
    case 2:
      Serial.println("AS: ES7243 Microphone (right channel only).");
      audioSource = new ES7243(SAMPLE_RATE, BLOCK_SIZE, 0, 0xFFFFFFFF);
      break;
    case 3:
      Serial.print("AS: SPH0645 Microphone - "); Serial.println(I2S_MIC_CHANNEL_TEXT);
      audioSource = new SPH0654(SAMPLE_RATE, BLOCK_SIZE, 0, 0xFFFFFFFF);
      break;
    case 4:
      Serial.print("AS: Generic I2S Microphone with Master Clock - "); Serial.println(I2S_MIC_CHANNEL_TEXT);
      audioSource = new I2SSourceWithMasterClock(SAMPLE_RATE, BLOCK_SIZE, 0, 0xFFFFFFFF);
      break;
    case 5:
      Serial.print("AS: I2S PDM Microphone - "); Serial.println(I2S_MIC_CHANNEL_TEXT);
      audioSource = new I2SPdmSource(SAMPLE_RATE, BLOCK_SIZE, 0, 0xFFFFFFFF);
      break;
    case 0:
    default:
      Serial.println("AS: Analog Microphone (left channel only).");
      audioSource = new I2SAdcSource(SAMPLE_RATE, BLOCK_SIZE, 0, 0x0FFF);
      break;
  }

  delay(100);

  audioSource->initialize();
  delay(250);

  if(!audioSource->isInitialized())
    Serial.println("AS: Failed to initialize sound input driver. Please check input PIN settings.");

  sampling_period_us = round(1000000*(1.0/SAMPLE_RATE));

  // Define the FFT Task and lock it to core 0
  xTaskCreatePinnedToCore(
        FFTcode,                          // Function to implement the task
        "FFT",                            // Name of the task
        5000,                            // Stack size in words
        NULL,                             // Task input parameter
        1,                                // Priority of the task
        &FFT_Task,                        // Task handle
        0);                               // Core where the task should run

  if(audioSource->isInitialized())
    disableSoundProcessing = false; // let it run
}

// This gets called every time WiFi is (re-)connected. Initialize own network interfaces here
void userConnected() {
}

// userLoop. You can use "if (WLED_CONNECTED)" to check for successful connection
void userLoop() {
  static unsigned long lastUMRun = millis();          // time of last filter run

  // suspend local sound processing when "real time mode" is active (E131, UDP, ADALIGHT, ARTNET)
  if (  (realtimeOverride == REALTIME_OVERRIDE_NONE)  // user override
      &&(useMainSegmentOnly == false)                 // cannot suspend when "main segment only" is set - other segments may still need sound data.
      &&( (realtimeMode == REALTIME_MODE_GENERIC)     // these realtime modes take complete control of all LEDs, so it's safe to disable sound processing
        ||(realtimeMode == REALTIME_MODE_E131)
        ||(realtimeMode == REALTIME_MODE_UDP)
        ||(realtimeMode == REALTIME_MODE_ADALIGHT)
        ||(realtimeMode == REALTIME_MODE_TPM2NET)
        ||(realtimeMode == REALTIME_MODE_ARTNET) ) ) 
  {
    #ifdef WLED_DEBUG
    if ((disableSoundProcessing == false) && (audioSyncEnabled == 0)) {  // we just switched to "disabled"
      DEBUG_PRINTLN("[AS userLoop] realtime mode active - audio processing suspended.");
      DEBUG_PRINTF( "              RealtimeMode = %d; RealtimeOverride = %d useMainSegmentOnly=%d\n", int(realtimeMode), int(realtimeOverride), int(useMainSegmentOnly));
    }
    #endif
    disableSoundProcessing = true;
  } else {
    if(audioSource->isInitialized()) { // only enable if sound input driver was initialized successfully
      #ifdef WLED_DEBUG
      if ((disableSoundProcessing == true) && (audioSyncEnabled == 0)) {    // we just switched to "enabled"
        DEBUG_PRINTLN("[AS userLoop] realtime mode ended - audio processing resumed.");
        DEBUG_PRINTF( "              RealtimeMode = %d; RealtimeOverride = %d useMainSegmentOnly=%d\n", int(realtimeMode), int(realtimeOverride), int(useMainSegmentOnly));
      }
      #endif
      if ((disableSoundProcessing == true) && (audioSyncEnabled == 0)) lastUMRun = millis();  // just left "realtime mode" - update timekeeping
      disableSoundProcessing = false;
    }
  }

  if (audioSyncEnabled & (1 << 1)) 
    disableSoundProcessing = true;   // make sure everything is disabled IF in audio Receive mode
  if (audioSyncEnabled & (1 << 0)  && audioSource->isInitialized()) 
    disableSoundProcessing = false;  // keep running audio IF we're in audio Transmit mode

  int userloopDelay = int(millis() - lastUMRun);
  if (lastUMRun == 0) userloopDelay=0; // startup - don't have valid data from last run.

  if ((!disableSoundProcessing) && (!(audioSyncEnabled & (1 << 1)))) { // Only run the sampling code IF we're not in realtime mode and not in audio Receive mode
    #ifdef WLED_DEBUG
    // compain when audio userloop has been delayed for long. Currently we need userloop running between 500 and 1500 times per second. 
    if ((userloopDelay > 50) || ((dmType != 0) && userloopDelay > 23)) {    // should not happen. Expect lagging in SR effects if you see this mesage !!!
      DEBUG_PRINTF("[AS userLoop] hickup detected -> was inactive for last %d millis!\n", int(millis() - lastUMRun));
    }
    #endif

    unsigned long t_now = millis();
    lastUMRun = t_now;
    if (soundAgc > AGC_NUM_PRESETS) soundAgc = 0; // make sure that AGC preset is valid (to avoid array bounds violation)

    if (userloopDelay <2) userloopDelay = 0;      // minor glitch, no problem
    if (userloopDelay >150) userloopDelay = 150;  // limit number of filter re-runs  
    do {
      getSample();                        // Sample the microphone
      agcAvg(t_now - userloopDelay);      // Calculated the PI adjusted value as sampleAvg
      userloopDelay -= 2;                 // advance "simulated time" by 2ms
    } while (userloopDelay > 0);

    myVals[millis()%32] = sampleAgc;

    static uint8_t lastMode = 0;
    static bool agcEffect = false;
    uint8_t knownMode = strip.getMainSegment().mode;

    if (lastMode != knownMode) { // only execute if mode changes
      char lineBuffer[3];
      /* uint8_t printedChars = */ extractModeName(knownMode, JSON_mode_names, lineBuffer, 3); //is this 'the' way to get mode name here?

      //used the following code to reverse engineer this
      // Serial.println(lineBuffer);
      // for (uint8_t i = 0; i<printedChars; i++) {
      //   Serial.print(i);
      //   Serial.print( ": ");
      //   Serial.println(uint8_t(lineBuffer[i]));
      // }
      agcEffect = (lineBuffer[1] == 226 && lineBuffer[2] == 153); // && (lineBuffer[3] == 170 || lineBuffer[3] == 171 ) encoding of ♪ or ♫
      // agcEffect = (lineBuffer[4] == 240 && lineBuffer[5] == 159 && lineBuffer[6] == 142 && lineBuffer[7] == 154 ); //encoding of 🎚 No clue why as not found here https://www.iemoji.com/view/emoji/918/objects/level-slider

      // if (agcEffect)
      //   Serial.println("found ♪ or ♫");
    }

    // update inputLevel Slider based on current AGC gain
    if ((soundAgc>0) && agcEffect) {
      static unsigned long last_update_time = 0;
      static unsigned long last_kick_time = 0;
      static int last_user_inputLevel = 0;
      unsigned long now_time = millis();    

      // "user kick" feature - if user has moved the slider by at least 32 units, we "kick" AGC gain by 30% (up or down)
      // only once in 3.5 seconds
      if (   (lastMode == knownMode)
          && (abs(last_user_inputLevel - inputLevel) > 31) 
          && (now_time - last_kick_time > 3500)) {
        if (last_user_inputLevel > inputLevel) multAgc *= 0.60; // down -> reduce gain
        if (last_user_inputLevel < inputLevel) multAgc *= 1.50; // up -> increase gain
        last_kick_time = now_time;
      }

      int new_user_inputLevel = 128.0 * multAgc;                                       // scale AGC multiplier so that "1" is at 128
      if (multAgc > 1.0) new_user_inputLevel = 128.0 * (((multAgc - 1.0) / 4.0) +1.0); // compress range so we can show values up to 4
      new_user_inputLevel = MIN(MAX(new_user_inputLevel, 0),255);

	    // update user interfaces - restrict frequency to avoid flooding UI's with small changes
      if ( ( ((now_time - last_update_time > 3500) && (abs(new_user_inputLevel - inputLevel) > 2))     // small change - every 3.5 sec (max) 
           ||((now_time - last_update_time > 2200) && (abs(new_user_inputLevel - inputLevel) > 15))    // medium change
           ||((now_time - last_update_time > 1200) && (abs(new_user_inputLevel - inputLevel) > 31))    // BIG change - every second
           ) && (now_time - lastInterfaceUpdate > INTERFACE_UPDATE_COOLDOWN))		                     // respect UI cooldown time
      {
        inputLevel = new_user_inputLevel;           // change of least 3 units -> update user variable
        updateInterfaces(CALL_MODE_WS_SEND);        // is this the correct way to notify UIs ? Yes says blazoncek
        last_update_time = now_time;
        last_user_inputLevel = new_user_inputLevel;
      }
    }
    lastMode = knownMode;

#if defined(MIC_LOGGER) || defined(FFT_SAMPLING_LOG)
    static unsigned long last_miclogger_time = 0;
    if (millis() - last_miclogger_time > 24) {
      last_miclogger_time = millis();
      logAudio();
    }
#endif

  }

  // Info Page: keep max sample from last 5 seconds
  if ((millis() -  sampleMaxTimer) > CYCLE_SAMPLEMAX) {
    sampleMaxTimer = millis();
    maxSample5sec = (0.15 * maxSample5sec) + 0.85 *((soundAgc) ? sampleAgc : sampleAvg); // reset, and start with some smoothing
    if (sampleAvg < 1) maxSample5sec = 0; // noise gate 
  } else {
      if ((sampleAvg >= 1)) maxSample5sec = fmaxf(maxSample5sec, (soundAgc) ? rawSampleAgc : sampleRaw); // follow maximum volume
  }

  // limit dynamics (experimental)
  limitSampleDynamics();

  // Begin UDP Microphone Sync
  if (audioSyncEnabled & (1 << 0)) {    // Only run the transmit code IF we're in Transmit mode
    static unsigned long last_transmit_time = 0;
    if (millis() - last_transmit_time > 18) {
        //Serial.println("Transmitting UDP Mic Packet");
        last_transmit_time = millis();
        transmitAudioData();
      }
  }

  if (audioSyncEnabled & (1 << 1)) {    // Only run the audio listener code if we're in Receive mode
    if (millis()-lastTime > delayMs) {
      lastTime = millis();
      if (udpSyncConnected) {
        //Serial.println("Checking for UDP Microphone Packet");
        int packetSize = fftUdp.parsePacket();
        if (packetSize > 6) {  // packet is big enough to contain a t least the header
          // Serial.println("Received UDP Sync Packet");
          uint8_t fftBuff[packetSize];
          fftUdp.read(fftBuff, packetSize);
          static audioSyncPacket receivedPacket;                                      // softhack007: added "static"
          memcpy(&receivedPacket, fftBuff, MIN(sizeof(receivedPacket), packetSize));  // don't copy more that what fits into audioSyncPacket
          receivedPacket.header[5] = '\0';                                            // ensure string termination

          // VERIFY THAT THIS IS A COMPATIBLE PACKET
          if (isValidUdpSyncVersion2(receivedPacket.header)) {
            // decode "V2" packet
            last_UDPTime = millis();                                                  // tell Info page that we are "receiving"
            receivedFormat = 2;
            extract_v2_packet(packetSize, fftBuff);
          } else {

            if (isValidUdpSyncVersion(receivedPacket.header)) {
              // decode "V1" packet
              last_UDPTime = millis();                                                  // tell Info page that we are "receiving"
              receivedFormat = 1;
              for (int i = 0; i < 32; i++ ) myVals[i] = receivedPacket.myVals[i];
              sampleAgc = receivedPacket.sampleAgc;
              rawSampleAgc = receivedPacket.sampleAgc;
              sampleRaw = receivedPacket.sampleRaw;
              sampleAvg = receivedPacket.sampleAvg;

              // auto-reset sample peak. Need to do it here, because getSample() is not running
              uint16_t MinShowDelay = strip.getMinShowDelay();
              if (millis() - timeOfPeak > MinShowDelay) {   // Auto-reset of samplePeak after a complete frame has passed.
                samplePeak = 0;
                udpSamplePeak = 0;
              }
              if (userVar1 == 0) samplePeak = 0;

              // Only change samplePeak IF it's currently false.
              // If it's true already, then the animation still needs to respond.
              if (!samplePeak) {
                samplePeak = receivedPacket.samplePeak;
                if (samplePeak) timeOfPeak = millis();
                udpSamplePeak = samplePeak;
                userVar1 = samplePeak;
              }
              //These values are only available on the ESP32
              for (int i = 0; i < 16; i++) fftResult[i] = receivedPacket.fftResult[i];
              FFT_Magnitude = fabsf(receivedPacket.FFT_Magnitude);
              FFT_MajorPeak = constrain(receivedPacket.FFT_MajorPeak, 1.0f, 5120.0f); // restrict value to range expected by effects
              //Serial.println("Finished parsing UDP Sync Packet");
            }
          }
        }
      }
    }
  }
} // userLoop()


// Provide Info for Web UI Info page
char audioStatusInfo[7][24] = {{'\0'}, {'\0'}, {'\0'}, {'\0'}, {'\0'}, {'\0'}, {'\0'}};
void usermod_updateInfo(void) {

  // Audio Source
  strcpy(audioStatusInfo[0], "- none");
  strcpy(audioStatusInfo[1], " -");
  if (audioSyncEnabled & 0x02) {                    // UDP sound sync - receive mode
    strcpy(audioStatusInfo[0], "UDP sound sync");
    if (udpSyncConnected) {
      if ((millis() - last_UDPTime) < 2500)
        strcpy(audioStatusInfo[1], " - receiving");
      else
        strcpy(audioStatusInfo[1], " - idle");
    } else {
        strcpy(audioStatusInfo[1], " - no connection");
    }
  } else {                                          // Analog or I2S digital input
    if (audioSource && (audioSource->isInitialized())) { 
      // audio source sucessfully configured
      if ((dmType == 0) && (audioPin > 0)) strcpy(audioStatusInfo[0], "ADC analog");
      if ((dmType > 0) && (i2ssdPin > 0)) strcpy(audioStatusInfo[0], "I2S digital");
      if (maxSample5sec > 1.0) {
        float my_usage = 100.0f * (maxSample5sec / 255.0f);
        snprintf(audioStatusInfo[1], 23, " - peak %3d%%", int(my_usage));
      } else {
        strcpy(audioStatusInfo[1], " - quiet");
      }
    } else {                                        // error during audio source setup
      strcpy(audioStatusInfo[0], "not initialized");
      strcpy(audioStatusInfo[1], " - check GPIO config");
    }
  }
  
  // AGC or manual Gain
  if (audioSource && audioSource->isInitialized() && (disableSoundProcessing == false) && !(audioSyncEnabled & 0x02)) {
    if (soundAgc==0) {
      float myGain = ((float)sampleGain/40.0f * (float)inputLevel/128.0f) + 1.0f/16.0f;     // non-AGC gain from presets
      snprintf(audioStatusInfo[2], 23, "%5.2f", roundf(myGain*100.0f) / 100.0f);
    } else {
      snprintf(audioStatusInfo[2], 23, "%5.2f", roundf(multAgc*100.0f) / 100.0f);           // AGC gain
    }
  } else strcpy(audioStatusInfo[2], "");                                                    // nothing

  // UDP Sound Sync status
  strcpy(audioStatusInfo[3], "");
  strcpy(audioStatusInfo[4], "");
  if (audioSyncEnabled > 0) {
    if (audioSyncEnabled & 0x01){ 
      strcpy(audioStatusInfo[3], "send mode");
      if ((udpSyncConnected) && (disableSoundProcessing == false)) strcpy(audioStatusInfo[4], " (v1)");
    } else { 
      if (audioSyncEnabled & 0x02) {
        strcpy(audioStatusInfo[3], "receive mode");
        if ((receivedFormat == 1) && udpSyncConnected && ((millis() - last_UDPTime) < 2500)) strcpy(audioStatusInfo[4], " (v1)");
        if ((receivedFormat == 2) && udpSyncConnected && ((millis() - last_UDPTime) < 2500)) strcpy(audioStatusInfo[4], " (v2)");
      } else strcpy(audioStatusInfo[3], "");
    }
  } else strcpy(audioStatusInfo[3], "off");
  if (audioSyncEnabled && !udpSyncConnected) strcpy(audioStatusInfo[4], " <i>(unconnected)</i>");

  // Sound processing (FFT and input filters)
  if (audioSource && (disableSoundProcessing == false)) {
    strcpy(audioStatusInfo[5], "running");
  } else {
    strcpy(audioStatusInfo[5], "suspended");
  }

  bool foundPot = false;
  if ((dmType == 0) && (audioPin > 0)) { // ADC analog input - warn if Potentimeter is configured
    for (int b=0; b<WLED_MAX_BUTTONS; b++) {
      if ((btnPin[b] >= 0) 
          && (buttonType[b] == BTN_TYPE_ANALOG || buttonType[b] == BTN_TYPE_ANALOG_INVERTED) 
          && (digitalPinToAnalogChannel(btnPin[b]) >= 0) && (digitalPinToAnalogChannel(btnPin[b]) < 9)) // found ADC1(channel 0...8) analog input
        foundPot = true;
    }
  }
  if (foundPot) strcpy(audioStatusInfo[6], "disable analog button");
  else  strcpy(audioStatusInfo[6], "");

  // make sure all strings are terminated properly
  audioStatusInfo[0][23] = '\0'; audioStatusInfo[1][23] = '\0';
  audioStatusInfo[2][23] = '\0'; 
  audioStatusInfo[3][23] = '\0'; audioStatusInfo[4][23] = '\0';
  audioStatusInfo[5][23] = '\0'; 
  audioStatusInfo[6][23] = '\0'; 
}
