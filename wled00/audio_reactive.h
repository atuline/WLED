/*
 * This file allows you to add own functionality to WLED more easily
 * See: https://github.com/Aircoookie/WLED/wiki/Add-own-functionality
 * EEPROM bytes 2750+ are reserved for your custom use case. (if you extend #define EEPSIZE in const.h)
 * bytes 2400+ are currently ununsed, but might be used for future wled features
 */

#include "wled.h"

//#define MIC_SAMPLING_LOG

const int SAMPLE_RATE = 16000;

//Use userVar0 and userVar1 (API calls &U0=,&U1=, uint16_t)
#ifndef MIC_PIN
  #define MIC_PIN   A0
#endif

#ifndef LED_BUILTIN       // Set LED_BUILTIN if it is not defined by Arduino framework
  #define LED_BUILTIN 3
#endif

#define UDP_SYNC_HEADER "00001"

// As defined in wled.h
// byte soundSquelch = 10;                          // default squelch value for volume reactive routines
// byte sampleGain = 0;                             // Define a 'gain' for different types of ADC input devices.

int micIn;                                          // Current sample starts with negative values and large values, which is why it's 16 bit signed
int sample;                                         // Current sample
float sampleAvg = 0;                                // Smoothed Average
float micLev = 0;                                   // Used to convert returned value to have '0' as minimum. A leveller
uint8_t maxVol = 5;                                 // Reasonable value for constant volume for 'peak detector', as it won't always trigger
bool samplePeak = 0;                                // Boolean flag for peak. Responding routine must reset this flag
int sampleAdj;                                      // Gain adjusted sample value.
#ifdef ESP32                                        // Transmitting doesn't work on ESP8266, don't bother allocating memory
bool udpSamplePeak = 0;                             // Boolean flag for peak. Set at the same tiem as samplePeak, but reset by transmitAudioData
#endif
int sampleAgc;                                      // Our AGC sample
float multAgc;                                      // sample * multAgc = sampleAgc. Our multiplier
uint8_t targetAgc = 60;                             // This is our setPoint at 20% of max for the adjusted output

long lastTime = 0;
int delayMs = 10;                                   // I don't want to sample too often and overload WLED.
double beat = 0;                                    // beat Detection

uint16_t micData;                                   // Analog input for FFT
uint16_t lastSample;                                // last audio noise sample

uint8_t myVals[32];                                 // Used to store a pile of samples as WLED frame rate and WLED sample rate are not synchronized

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
  return (header == UDP_SYNC_HEADER);
}

void getSample() {
  static long peakTime;

  #ifdef WLED_DISABLE_SOUND
    micIn = inoise8(millis(), millis());            // Simulated analog read
  #else
    micIn = analogRead(MIC_PIN);                    // Poor man's analog read
  #endif

  micLev = ((micLev * 31) + micIn) / 32;            // Smooth it out over the last 32 samples for automatic centering
  micIn -= micLev;                                  // Let's center it to 0 now
  micIn = abs(micIn);                               // And get the absolute value of each sample

  lastSample = micIn;

  // Using a ternary operator, the resultant sample is either 0 or it's a bit smoothed out with the last sample.
  sample = (micIn <= soundSquelch) ? 0 : (sample * 3 + micIn) / 4;

  sampleAdj = sample * sampleGain / 40 + sample / 16; // Adjust the gain.
  sampleAdj = min(sampleAdj, 255);
  sample = sampleAdj;                                 // We'll now make our rebase our sample to be adjusted.

  sampleAvg = ((sampleAvg * 15) + sample) / 16;       // Smooth it out over the last 16 samples.

  if (userVar1 == 0)
    samplePeak = 0;
  // Poor man's beat detection by seeing if sample > Average + some value.
  if (sample > (sampleAvg + maxVol) && millis() > (peakTime + 100)) {
  // Then we got a peak, else we don't. Display routines need to reset the samplepeak value in case they miss the trigger.
    samplePeak = 1;
    #ifdef ESP32
      udpSamplePeak = 1;
    #endif
    userVar1 = samplePeak;
    peakTime=millis();
  }

}  // getSample()



void agcAvg() {                                                     // A simple averaging multiplier to automatically adjust sound sensitivity.

  multAgc = (sampleAvg < 1) ? targetAgc : targetAgc / sampleAvg;    // Make the multiplier so that sampleAvg * multiplier = setpoint
  sampleAgc = sample * multAgc;
  if (sampleAgc > 255) sampleAgc = 0;

  userVar0 = sampleAvg * 4;
  if (userVar0 > 255) userVar0 = 255;

} // agcAvg()


  double FFT_MajorPeak = 0;
  double FFT_Magnitude = 0;
  double fftResult[16];


void logAudio() {

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
  #ifdef ESP32                                   // if we are on a ESP32
    Serial.print("running on core ");               // identify core
    Serial.println(xPortGetCoreID());
  #endif
#endif
}  // logAudio()
