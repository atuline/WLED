/*
 * This file allows you to add own functionality to WLED more easily
 * See: https://github.com/Aircoookie/WLED/wiki/Add-own-functionality
 * EEPROM bytes 2750+ are reserved for your custom use case. (if you extend #define EEPSIZE in const.h)
 * bytes 2400+ are currently ununsed, but might be used for future wled features
 */

#include "wled.h"

#ifdef ESP8266_I2S
#include <i2s.h>
#endif // ESP8266_I2S

// WARNING Sound reactive variables that are used by the animations or other asynchronous routines must NOT
// have interim values, but only updated in a single calculation. These are:
//
// sample     sampleAvg     sampleAgc       samplePeak    myVals[]
//
// fftBin[]   fftResult[]   FFT_MajorPeak   FFT_Magnitude
//
// Otherwise, the animations may asynchronously read interim values of these variables.
//

// Comment/Uncomment to toggle usb serial debugging
// #define SR_DEBUG

#ifdef SR_DEBUG
  #define DEBUGSR_PRINT(x) Serial.print(x)
  #define DEBUGSR_PRINTLN(x) Serial.println(x)
  #define DEBUGSR_PRINTF(x...) Serial.printf(x)
#else
  #define DEBUGSR_PRINT(x)
  #define DEBUGSR_PRINTLN(x)
  #define DEBUGSR_PRINTF(x...)
#endif

// #define MIC_LOGGER
// #define MIC_SAMPLING_LOG

const int SAMPLE_RATE = 10240;

//Use userVar0 and userVar1 (API calls &U0=,&U1=, uint16_t)
#ifndef MIC_PIN
  #define MIC_PIN   A0
#endif

#ifndef LED_BUILTIN       // Set LED_BUILTIN if it is not defined by Arduino framework
  #define LED_BUILTIN 3
#endif

#define UDP_SYNC_HEADER "00001"

uint8_t maxVol = 10;                            // Reasonable value for constant volume for 'peak detector', as it won't always trigger
uint8_t targetAgc = 60;                         // This is our setPoint at 20% of max for the adjusted output
uint8_t myVals[32];                             // Used to store a pile of samples because WLED frame rate and WLED sample rate are not synchronized. Frame rate is too low.
bool samplePeak = 0;                            // Boolean flag for peak. Responding routine must reset this flag
bool udpSamplePeak = 0;                         // Boolean flag for peak. Set at the same tiem as samplePeak, but reset by transmitAudioData
constexpr unsigned delayMs = 10;                               // I don't want to sample too often and overload WLED
int micIn;                                      // Current sample starts with negative values and large values, which is why it's 16 bit signed
int sample;                                     // Current sample. Must only be updated ONCE!!!
int tmpSample;                                  // An interim sample variable used for calculatioins.
int sampleAdj;                                  // Gain adjusted sample value
int sampleAgc;                                  // Our AGC sample
long timeOfPeak = 0;
long lastTime = 0;
float micLev = 0;                               // Used to convert returned value to have '0' as minimum. A leveller
float multAgc;                                  // sample * multAgc = sampleAgc. Our multiplier
float sampleAvg = 0;                            // Smoothed Average

float expAdjF;                                  // Used for exponential filter.
float weighting = 0.2;                          // Exponential filter weighting. Will be adjustable in a future release.

#ifdef ESP8266_I2S
int16_t micIn_L;
int16_t micIn_R;
#endif // ESP8266_I2S

struct audioSyncPacket {
  char header[6] = UDP_SYNC_HEADER;
  uint8_t myVals[32];     //  32 Bytes
  int sampleAgc;          //  04 Bytes
  int sample;             //  04 Bytes
  float sampleAvg;        //  04 Bytes
  bool samplePeak;        //  01 Bytes
  uint8_t fftResult[16];  //  16 Bytes
  double FFT_Magnitude;   //  08 Bytes
  double FFT_MajorPeak;   //  08 Bytes
};

bool isValidUdpSyncVersion(char header[6]) {
  if (strncmp(header, UDP_SYNC_HEADER, 6) == 0) {
    return true;
  } else {
    return false;
  }
}

void getSample() {
  static long peakTime;

  #ifdef WLED_DISABLE_SOUND
    micIn = inoise8(millis(), millis());          // Simulated analog read
  #else
    #ifdef ESP8266_I2S // test sketch: https://pastebin.com/SVqWGxrH
      i2s_read_sample(&micIn_L, &micIn_R, true);
      micIn = micIn_L; // 16 bit read, could have >> 6 to match analogRead's 10 bit, but in my testing, the range is pretty close to the ADC even with 16 bit.
    #else
      micIn = analogRead(MIC_PIN);                  // Poor man's analog read
    #endif
  #endif

//////
    DEBUGSR_PRINT("micIn:\tmicIn>>2:\tmic_In_abs:\tsample:\tsampleAdj:\tsampleAvg:\n");
    DEBUGSR_PRINT(micIn);
//////
    DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(micIn);
  micLev = ((micLev * 31) + micIn) / 32;          // Smooth it out over the last 32 samples for automatic centering
  micIn -= micLev;                                // Let's center it to 0 now
  micIn = abs(micIn);                             // And get the absolute value of each sample
//////
  DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(micIn);

// Using an exponential filter to smooth out the signal. We'll add controls for this in a future release.
  expAdjF = (weighting * micIn + (1.0-weighting) * expAdjF);
  expAdjF = (expAdjF <= soundSquelch) ? 0: expAdjF;

  tmpSample = (int)expAdjF;

//////
  DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(sample);

  sampleAdj = tmpSample * sampleGain / 40 + tmpSample / 16; // Adjust the gain.
  sampleAdj = min(sampleAdj, 255);
  sample = sampleAdj;                             // ONLY update sample ONCE!!!!

  sampleAvg = ((sampleAvg * 15) + sample) / 16;   // Smooth it out over the last 16 samples.
//////

  DEBUGSR_PRINT("\t"); DEBUGSR_PRINT(sample);
  DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(sampleAvg); DEBUGSR_PRINT("\n\n");

  if (millis() - timeOfPeak > MIN_SHOW_DELAY) {   // Auto-reset of samplePeak after a complete frame has passed.
    samplePeak = 0;
    udpSamplePeak = 0;
    }

  if (userVar1 == 0) samplePeak = 0;
  // Poor man's beat detection by seeing if sample > Average + some value.
  if ((sample > (sampleAvg + maxVol)) && ((millis() - peakTime) > 100)) {
  // Then we got a peak, else we don't. Display routines need to reset the samplepeak value in case they miss the trigger.
    samplePeak = 1;
    timeOfPeak = millis();
    userVar1 = samplePeak;
    peakTime=millis();
  }
} // getSample()

/*
 * A simple averaging multiplier to automatically adjust sound sensitivity.
 */
void agcAvg() {

  multAgc = (sampleAvg < 1) ? targetAgc : targetAgc / sampleAvg;  // Make the multiplier so that sampleAvg * multiplier = setpoint
  int tmpAgc = sample * multAgc;
  if (tmpAgc > 255) tmpAgc = 0;
  sampleAgc = tmpAgc;                             // ONLY update sampleAgc ONCE because it's used elsewhere asynchronously!!!!
  userVar0 = sampleAvg * 4;
  if (userVar0 > 255) userVar0 = 255;
} // agcAvg()



void logAudio() {
#ifdef MIC_LOGGER


//  Serial.print(micIn);      Serial.print(" ");
//  Serial.print(sample); Serial.print(" ");
//  Serial.print(sampleAvg); Serial.print(" ");
//  Serial.print(sampleAgc);  Serial.print(" ");
  Serial.println(" ");

#endif

#ifdef MIC_SAMPLING_LOG
  //------------ Oscilloscope output ---------------------------
  Serial.print(targetAgc); Serial.print(" ");
  Serial.print(multAgc); Serial.print(" ");
  Serial.print(sampleAgc); Serial.print(" ");

  Serial.print(sample); Serial.print(" ");
  Serial.print(sampleAvg); Serial.print(" ");
  Serial.print(micLev); Serial.print(" ");
  Serial.print(samplePeak); Serial.print(" ");    //samplePeak = 0;
  Serial.print(micIn); Serial.print(" ");
  Serial.print(100); Serial.print(" ");
  Serial.print(0); Serial.print(" ");
  Serial.println(" ");
#endif
}  // logAudio()
