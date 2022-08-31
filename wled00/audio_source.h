#pragma once

#include <Wire.h>
#include "wled.h"
#include <driver/i2s.h>
#include <driver/adc.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include <driver/adc_deprecated.h>
#include <driver/adc_types_deprecated.h>
#endif

//#include <driver/i2s_std.h>
//#include <driver/i2s_pdm.h>
//#include <driver/gpio.h>

/* ToDo: remove. ES7243 is controlled via compiler defines
   Until this configuration is moved to the webinterface
*/


// data type requested from the I2S driver - currently we always use 32bit
//#define I2S_USE_16BIT_SAMPLES    // (experimental) define this to request 16bit - more efficient but possibly less compatible

// if you have problems to get your microphone work on the left channel, uncomment the following line
//#define I2S_USE_RIGHT_CHANNEL    // (experimental) define this to use right channel (digital mics only)

// Uncomment the line below to utilize ADC1 _exclusively_ for I2S sound input.
// benefit: analog mic inputs will be sampled contiously -> better response times and less "glitches"
// WARNING: this option WILL lock-up your device in case that any other analogRead() operation is performed; 
//          for example if you want to read "analog buttons"
//#define I2S_GRAB_ADC1_COMPLETELY // (experimental) continously sample analog ADC microphone. WARNING will cause analogRead() lock-up 


#ifdef I2S_USE_16BIT_SAMPLES
#define I2S_SAMPLE_RESOLUTION I2S_BITS_PER_SAMPLE_16BIT
#define I2S_datatype int16_t
#define I2S_unsigned_datatype uint16_t
#undef  I2S_SAMPLE_DOWNSCALE_TO_16BIT
#else
#define I2S_SAMPLE_RESOLUTION I2S_BITS_PER_SAMPLE_32BIT
#define I2S_datatype int32_t
#define I2S_unsigned_datatype uint32_t
#define I2S_SAMPLE_DOWNSCALE_TO_16BIT
#endif

#ifdef I2S_USE_RIGHT_CHANNEL
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_RIGHT
#define I2S_MIC_CHANNEL_TEXT "right channel only."
#else
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
#define I2S_MIC_CHANNEL_TEXT "left channel only."
#endif

#ifndef MCLK_PIN
    int mclkPin = 0;
#else
    int mclkPin = MLCK_PIN;
#endif

#ifndef ES7243_ADDR
    int addr_ES7243 = 0x13;
#else
    int addr_ES7243 =  ES7243_ADDR;
#endif

#ifndef ES7243_SDAPIN
    int pin_ES7243_SDA = 18;
#else
    int pin_ES7243_SDA =  ES7243_SDAPIN;
#endif

#ifndef ES7243_SDAPIN
    int pin_ES7243_SCL = 23;
#else
    int pin_ES7243_SCL =  ES7243_SCLPIN;
#endif

/* Interface class
   AudioSource serves as base class for all microphone types
   This enables accessing all microphones with one single interface
   which simplifies the caller code
*/
class AudioSource {
public:
    /* All public methods are virtual, so they can be overridden
       Everything but the destructor is also removed, to make sure each mic
       Implementation provides its version of this function
    */
    virtual ~AudioSource() {};

    /* Initialize
       This function needs to take care of anything that needs to be done
       before samples can be obtained from the microphone.
    */
    virtual void initialize() = 0;

    /* Deinitialize
       Release all resources and deactivate any functionality that is used
       by this microphone
    */
    virtual void deinitialize() = 0;

    /* getSamples
       Read num_samples from the microphone, and store them in the provided
       buffer
    */
    virtual void getSamples(double *buffer, uint16_t num_samples) = 0;

    /* Get an up-to-date sample without DC offset */
    virtual int getSampleWithoutDCOffset() = 0;

    /* check if the audio source driver was initialized successfully */
    virtual bool isInitialized(void) {return(_initialized);}

protected:
    // Private constructor, to make sure it is not callable except from derived classes
    AudioSource(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) : _sampleRate(sampleRate), _blockSize(blockSize), _sampleNoDCOffset(0), _dcOffset(0.0f), _shift(lshift), _mask(mask), 
                _initialized(false), _myADCchannel(0x0F), _lastADCsample(0), _broken_samples_counter(0) {};

    int _sampleRate;                /* Microphone sampling rate (from uint16_t to int to suppress warning)*/ 
    int _blockSize;                 /* I2S block size */
    volatile int _sampleNoDCOffset; /* Up-to-date sample without DCOffset */
    float _dcOffset;                /* Rolling average DC offset */
    int16_t _shift;                /* Shift obtained samples to the right (positive) or left(negative) by this amount */
    uint32_t _mask;                 /* Bitmask for sample data after shifting. Bitmask 0X0FFF means that we need to convert 12bit ADC samples from unsigned to signed*/
    bool _initialized;              /* Gets set to true if initialization is successful */
    int8_t _myADCchannel;           /* current ADC channel, in case of analog input. 0x0F if undefined */
    I2S_datatype _lastADCsample;    /* last sample from ADC */
    I2S_datatype decodeADCsample(I2S_unsigned_datatype rawData); /* function to handle ADC samples */
    unsigned int _broken_samples_counter; /* counts number of broken (and fixed) ADC samples */
};

/* Basic I2S microphone source
   All functions are marked virtual, so derived classes can replace them
*/
class I2SSource : public AudioSource {
public:
    I2SSource(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
        AudioSource(sampleRate, blockSize, lshift, mask) {
        _config = {
            .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
            .sample_rate = _sampleRate,                       // "narrowing conversion" warning can be ignored here - our _sampleRate is never bigger that INT32_MAX
            .bits_per_sample = I2S_SAMPLE_RESOLUTION,
            .channel_format = I2S_MIC_CHANNEL,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
            .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
#else
            .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
#endif
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = 8,
            .dma_buf_len = _blockSize
        };

        _pinConfig = {
            .bck_io_num = i2sckPin,
            .ws_io_num = i2swsPin,
            .data_out_num = I2S_PIN_NO_CHANGE,
            .data_in_num = i2ssdPin
        };
    };




    virtual void initialize() {

        if (!pinManager.allocatePin(i2swsPin, true, PinOwner::DigitalMic) ||
            !pinManager.allocatePin(i2ssdPin, false, PinOwner::DigitalMic)) {
                return;
        }

        // i2ssckPin needs special treatment, since it might be unused on PDM mics
        if (i2sckPin != -1) {
            if (!pinManager.allocatePin(i2sckPin, true, PinOwner::DigitalMic))
                return;
        }

        esp_err_t err = i2s_driver_install(I2S_NUM_0, &_config, 0, nullptr);
        if (err != ESP_OK) {
            Serial.printf("Failed to install i2s driver: %d\n", err);
            return;
        }

        err = i2s_set_pin(I2S_NUM_0, &_pinConfig);
        if (err != ESP_OK) {
            Serial.printf("Failed to set i2s pin config: %d\n", err);
            return;
        }

        _initialized = true;
    }

    virtual void deinitialize() {
        if (_initialized) {
            _initialized = false;
            esp_err_t err = i2s_driver_uninstall(I2S_NUM_0);
            if (err != ESP_OK) {
                Serial.printf("Failed to uninstall i2s driver: %d\n", err);
                return;
            }
        }
        pinManager.deallocatePin(i2swsPin, PinOwner::DigitalMic);
        pinManager.deallocatePin(i2ssdPin, PinOwner::DigitalMic);
        // i2ssckPin needs special treatment, since it might be unused on PDM mics
        if (i2sckPin != -1) {
            pinManager.deallocatePin(i2sckPin, PinOwner::DigitalMic);
        }
    }

    virtual void getSamples(double *buffer, uint16_t num_samples) {
        if(_initialized) {
            esp_err_t err;
            size_t bytes_read = 0;        /* Counter variable to check if we actually got enough data */
            I2S_datatype newSamples[num_samples]; /* Intermediary sample storage */
           
            _dcOffset = 0.0f;              // Reset dc offset
            _broken_samples_counter = 0;   // Reset ADC broken samples counter

            // get fresh samples
            err = i2s_read(I2S_NUM_0, (void *)newSamples, sizeof(newSamples), &bytes_read, portMAX_DELAY);
            if ((err != ESP_OK)){
                Serial.printf("Failed to get samples: %d\n", err);
                return;
            }

            // For correct operation, we need to read exactly sizeof(samples) bytes from i2s
            if(bytes_read != sizeof(newSamples)) {
                Serial.printf("Failed to get enough samples: wanted: %d read: %d\n", sizeof(newSamples), bytes_read);
                return;
            }

            // Store samples in sample buffer and update DC offset
            for (int i = 0; i < num_samples; i++) {

                if (_mask == 0x0FFF) {  // mask = 0x0FFF means we are in I2SAdcSource
                    I2S_unsigned_datatype rawData = * reinterpret_cast<I2S_unsigned_datatype *> (newSamples + i); // C++ acrobatics to get sample as "unsigned"
                    I2S_datatype sampleNoFilter = decodeADCsample(rawData);
                    if (_broken_samples_counter >= num_samples-1) {             // kill-switch: ADC sample correction off when all samples in a batch were "broken"
                        _myADCchannel = 0x0F;
                        Serial.println("AS: too many broken audio samples from ADC - sample correction switched off.");
                    }

                    newSamples[i] = (3 * sampleNoFilter + _lastADCsample) / 4;  // apply low-pass filter (2-tap FIR)
                    //newSamples[i] = (sampleNoFilter + lastADCsample) / 2;      // apply stronger low-pass filter (2-tap FIR)
                    _lastADCsample = sampleNoFilter;                            // update ADC last sample
                }

                // pre-shift samples down to 16bit
#ifdef I2S_SAMPLE_DOWNSCALE_TO_16BIT
                if (_shift != 0)
                    newSamples[i] >>= 16;
#endif
                double currSample = 0.0;
                if(_shift > 0)
                  currSample = (double) (newSamples[i] >> _shift);
                else {
                  if(_shift < 0)
                    currSample = (double) (newSamples[i] << (- _shift)); // need to "pump up" 12bit ADC to full 16bit as delivered by other digital mics
                  else
#ifdef I2S_SAMPLE_DOWNSCALE_TO_16BIT
                    currSample = (double) newSamples[i] / 65536.0;        // _shift == 0 -> use the chance to keep lower 16bits
#else
                    currSample = (double) newSamples[i];
#endif
                }
                buffer[i] = currSample;
                _dcOffset = ((_dcOffset * 31) + currSample) / 32;
            }

            // Update no-DC sample
            _sampleNoDCOffset = buffer[num_samples - 1] - _dcOffset;
        }
    }

    // function to handle ADC samples
    I2S_datatype decodeADCsample(I2S_unsigned_datatype rawData) {
#ifndef I2S_USE_16BIT_SAMPLES
        rawData = (rawData >> 16) & 0xFFFF;                        // scale input down from 32bit -> 16bit
        I2S_datatype lastGoodSample = _lastADCsample / 16384 ;     // 26bit-> 12bit with correct sign handling
#else
        rawData = rawData & 0xFFFF;                                // input is already in 16bit, just mask off possible junk
        I2S_datatype lastGoodSample = _lastADCsample * 4;          // 10bit-> 12bit
#endif
        // decode ADC sample
        uint16_t the_channel = (rawData >> 12) & 0x000F;           // upper 4 bit = ADC channel
        uint16_t the_sample  =  rawData & 0x0FFF;                  // lower 12bit -> ADC sample (unsigned)
        I2S_datatype finalSample = (int(the_sample) - 2048);       // convert to signed (centered at 0);

        // fix bad samples
        if ((the_channel != _myADCchannel) && (_myADCchannel != 0x0F)) { // 0x0F means "don't know what my channel is" 
            finalSample = lastGoodSample;                         // replace with the last good ADC sample
            _broken_samples_counter ++;
            //Serial.print("\n!ADC rogue sample 0x"); Serial.print(rawData, HEX); Serial.print("\tchannel:");Serial.println(the_channel);
        }
#ifndef I2S_USE_16BIT_SAMPLES
        finalSample = finalSample << 16;    // scale up from 16bit -> 32bit;
#endif
        finalSample = finalSample / 4;      // mimic old analog driver behaviour (12bit -> 10bit)
        return(finalSample);
    }

    virtual int getSampleWithoutDCOffset() {
        return _sampleNoDCOffset;
    }

protected:
    i2s_config_t _config;
    i2s_pin_config_t _pinConfig;
};

/* I2S microphone with master clock
   Our version of the IDF does not support setting master clock
   routing via the provided API, so we have to do it by hand
*/
class I2SSourceWithMasterClock : public I2SSource {
public:
    I2SSourceWithMasterClock(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
        I2SSource(sampleRate, blockSize, lshift, mask) {
    };

    virtual void initialize() {
        // Reserve the master clock pin
        if(!pinManager.allocatePin(mclkPin, true, PinOwner::DigitalMic)) {
            return;
        }
        _routeMclk();
        I2SSource::initialize();

    }

    virtual void deinitialize() {
        // Release the master clock pin
        pinManager.deallocatePin(mclkPin, PinOwner::DigitalMic);
        I2SSource::deinitialize();
    }
protected:
    void _routeMclk() {
        /* Enable the mclk routing depending on the selected mclk pin
           Only I2S_NUM_0 is supported
        */
        if (mclkPin == GPIO_NUM_0) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
            WRITE_PERI_REG(PIN_CTRL,0xFFF0);
        } else if (mclkPin == GPIO_NUM_1) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
            WRITE_PERI_REG(PIN_CTRL, 0xF0F0);
        } else {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
            WRITE_PERI_REG(PIN_CTRL, 0xFF00);
        }
    }
};

/* ES7243 Microphone
   This is an I2S microphone that requires ininitialization over
   I2C before I2S data can be received
*/
class ES7243 : public I2SSourceWithMasterClock {

private:
    // I2C initialization functions for ES7243
    void _es7243I2cBegin() {
        Wire.begin(pin_ES7243_SDA, pin_ES7243_SCL, 100000U);
    }

    void _es7243I2cWrite(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(addr_ES7243);
        Wire.write((uint8_t)reg);
        Wire.write((uint8_t)val);
        Wire.endTransmission();
    }

    void _es7243InitAdc() {
        _es7243I2cBegin();
        _es7243I2cWrite(0x00, 0x01);
        _es7243I2cWrite(0x06, 0x00);
        _es7243I2cWrite(0x05, 0x1B);
        _es7243I2cWrite(0x01, 0x00); // 0x00 for 24 bit to match INMP441 - not sure if this needs adjustment to get 16bit samples from I2S
        _es7243I2cWrite(0x08, 0x43);
        _es7243I2cWrite(0x05, 0x13);
    }

public:

    ES7243(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
        I2SSourceWithMasterClock(sampleRate, blockSize, lshift, mask) {
        _config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
    };
    void initialize() {
        // Reserve SDA and SCL pins of the I2C interface
        if (!pinManager.allocatePin(pin_ES7243_SDA, true, PinOwner::DigitalMic) ||
            !pinManager.allocatePin(pin_ES7243_SCL, true, PinOwner::DigitalMic)) {
                return;
            }

        // First route mclk, then configure ADC over I2C, then configure I2S
        _es7243InitAdc();
        I2SSourceWithMasterClock::initialize();
    }

    void deinitialize() {
        // Release SDA and SCL pins of the I2C interface
        pinManager.deallocatePin(pin_ES7243_SDA, PinOwner::DigitalMic);
        pinManager.deallocatePin(pin_ES7243_SCL, PinOwner::DigitalMic);
        I2SSourceWithMasterClock::deinitialize();
    }
};

/* ADC over I2S Microphone
   This microphone is an ADC pin sampled via the I2S interval
   This allows to use the I2S API to obtain ADC samples with high sample rates
   without the need of manual timing of the samples
*/
class I2SAdcSource : public I2SSource {
public:
    I2SAdcSource(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
        I2SSource(sampleRate, blockSize, lshift, mask){
        _config = {
            .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
            .sample_rate = _sampleRate,                       // "narrowing conversion" warning can be ignored here - our _sampleRate is never bigger that INT32_MAX
            .bits_per_sample = I2S_SAMPLE_RESOLUTION,
            .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
            .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
#else
            .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
#endif
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = 8,
            .dma_buf_len = _blockSize,
            .use_apll    = false, // must be disabled for analog microphone
            .tx_desc_auto_clear = false,
            .fixed_mclk = 0
        };
    }

    void initialize() {

        // check if "analog buttons" are configured for ADC1, and issue warning
        for (int b=0; b<WLED_MAX_BUTTONS; b++) {
            //if ((btnPin[b] >= 0) && (buttonType[b] == BTN_TYPE_ANALOG || buttonType[b] == BTN_TYPE_ANALOG_INVERTED) && (digitalPinToAnalogChannel(btnPin[b]) < 10)) {
            if ((btnPin[b] >= 0) && (buttonType[b] == BTN_TYPE_ANALOG || buttonType[b] == BTN_TYPE_ANALOG_INVERTED) && (digitalPinToAnalogChannel(btnPin[b]) >= 0)) {
                Serial.println("AS: Analog Microphone does not work reliably when analog buttons are configured on ADC1.");
                Serial.printf( "    Button %d GPIO %d\n", b, btnPin[b]);
                Serial.println("    To recover, please disable any analog button in LED preferences, then restart your device.");
                Serial.flush();
#ifdef I2S_GRAB_ADC1_COMPLETELY
                Serial.println("AS: Analog Microphone initialization aborted. Cannot use ADC1 exclusively");
                return;
#endif
            }
        }

        if(!pinManager.allocatePin(audioPin, false, PinOwner::AnalogMic)) {
            return;
        }
        // Determine Analog channel. Only Channels on ADC1 are supported
        int8_t channel = digitalPinToAnalogChannel(audioPin);
        if ((channel < 0) || (channel > 9)) {  // channel == -1 means "not an ADC pin"
            Serial.printf("Incompatible GPIO used for audio in: %d\n", audioPin);
            return;
        } else {
            adc_gpio_init(ADC_UNIT_1, adc_channel_t(channel));
        }
        _myADCchannel = channel;
        _lastADCsample = 0;

        // Install Driver
        esp_err_t err = i2s_driver_install(I2S_NUM_0, &_config, 0, nullptr);
        if (err != ESP_OK) {
            Serial.printf("Failed to install i2s driver: %d\n", err);
            return;
        }

        // Enable I2S mode of ADC
        err = i2s_set_adc_mode(ADC_UNIT_1, adc1_channel_t(channel));
        if (err != ESP_OK) {
            Serial.printf("Failed to set i2s adc mode: %d\n", err);
            return;

        }
#if defined(I2S_GRAB_ADC1_COMPLETELY)
        // according to docs from espressif, the ADC needs to be started explicitly
        // fingers crossed
        err = i2s_adc_enable(I2S_NUM_0);
        if (err != ESP_OK) {
            Serial.printf("Failed to enable i2s adc: %d\n", err);
            //return;
        }
#else
        err = i2s_adc_disable(I2S_NUM_0);
		//err = i2s_stop(I2S_NUM_0);
        if (err != ESP_OK) {
            Serial.printf("Failed to initially disable i2s adc: %d\n", err);
        }
#endif
        _initialized = true;
    }

    void getSamples(double *buffer, uint16_t num_samples) {

    /* Enable ADC. This has to be enabled and disabled directly before and
    after sampling, otherwise Wifi dies and analogRead() hangs
    */
        if (_initialized) {
#if !defined(I2S_GRAB_ADC1_COMPLETELY)
			//esp_err_t err = i2s_start(I2S_NUM_0);
            esp_err_t err = i2s_adc_enable(I2S_NUM_0);
            if (err != ESP_OK) {
                Serial.printf("Failed to enable i2s adc: %d\n", err);
                return;
            }
#endif
            I2SSource::getSamples(buffer, num_samples);

#if !defined(I2S_GRAB_ADC1_COMPLETELY)
            err = i2s_adc_disable(I2S_NUM_0);
			//err = i2s_stop(I2S_NUM_0);
            if (err != ESP_OK) {
                Serial.printf("Failed to disable i2s adc: %d\n", err);
                return;
            }
#endif
        }
    }

    void deinitialize() {
        pinManager.deallocatePin(audioPin, PinOwner::AnalogMic);
        esp_err_t err;
        if (_initialized) {
            _initialized = false;
#if defined(I2S_GRAB_ADC1_COMPLETELY)
            // according to docs from espressif, the ADC needs to be stopped explicitly
            // fingers crossed
            err = i2s_adc_disable(I2S_NUM_0);
            if (err != ESP_OK) {
                Serial.printf("Failed to disable i2s adc: %d\n", err);
                //return;
            }
#endif
            i2s_adc_disable(I2S_NUM_0);     // just to be sure
            i2s_stop(I2S_NUM_0);
            err = i2s_driver_uninstall(I2S_NUM_0);
            if (err != ESP_OK) {
                Serial.printf("Failed to uninstall i2s driver: %d\n", err);
                return;
            }
        }
        _myADCchannel = 0x0F;
        _lastADCsample = 0;
    }
};

/* SPH0645 Microphone
   This is an I2S microphone with some timing quirks that need
   special consideration.
*/
class SPH0654 : public I2SSource {

public:
    SPH0654(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
        I2SSource(sampleRate, blockSize, lshift, mask){}

    void initialize() {
        I2SSource::initialize();
        REG_SET_BIT(I2S_TIMING_REG(I2S_NUM_0), BIT(9));
        REG_SET_BIT(I2S_CONF_REG(I2S_NUM_0), I2S_RX_MSB_SHIFT);
    }
};

/* I2S PDM Microphone
   This is an I2S PDM microphone, these microphones only use a clock and
   data line, to make it simpler to debug, use the WS pin as CLK and SD
   pin as DATA
*/

class I2SPdmSource : public I2SSource {

public:
    I2SPdmSource(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
        I2SSource(sampleRate, blockSize, lshift, mask) {

        _config.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM); // Change mode to pdm

        _pinConfig = {
            .bck_io_num = I2S_PIN_NO_CHANGE, // bck is unused in PDM mics
            .ws_io_num = i2swsPin, // clk pin for PDM mic
            .data_out_num = I2S_PIN_NO_CHANGE,
            .data_in_num = i2ssdPin
        };
    }
};
