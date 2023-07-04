/**
*********************************************************************************
* @file:	esp32_pwm_read.cpp
* @author:	R.W.
* @date:	2023.03.27
* @brief:	test program for use lib esp32_pwm_reader
*
* @history:	* 06.05.2023   rw v.0.1
*********************************************************************************
**/

#include <Arduino.h>
#include <Ticker.h>    // for wait in loop()
#include <inttypes.h>  // for printf

#include <esp32-rmt-pwm-reader.h>

Ticker ticker;

// init channels and pins
uint8_t pins[] = {13, 14, 33, 27, 32};  // Input pin numbers (pin 34 & 35 only usable as inputs!)
int numberOfChannels = sizeof(pins) / sizeof(uint8_t);

// helper for table print
#define CLEAR_SCREEN printf("\e[1;1H\e[2J")

void gotoRowCol(const int row, const int col) {
    // Position the cursor at the desired position (row, col)
    Serial.print("\033[");  // Begin of escape sequence
    Serial.print(row);      // row number (begins with 1)
    Serial.print(";");
    Serial.print(col);      // column (begins with 1)
    Serial.print("H");
}


//
// =======================================================================================================
// READ PWM RC SIGNALS AND PRINT TABLE
// =======================================================================================================
//
void readPwmSignals() {
    // auto channels = pwm_get_whole_channels_object(); // test read the whole array of channels readonly

    gotoRowCol(1, 1);
    
    // print table
    /*
    The function uses tab escape secence to format the table.
    It assumes that the terminal window has a tab width of 8. 
    If this is not the case, the tab width of the terminal shell should be set to 8 outside the application.
    Depending on the shell used, there are individual methods.
    Maybe this link can be helpful:
    https://stackoverflow.com/questions/10782699/how-to-set-4-space-tab-in-bash
    */
    // table header
    printf("Channel\toffset\t    raw\t scaled\t period\tfreq Hz\t  p_min\t p-ntrl\t  p_max\t   dc %%\t  state\n");
    for (uint8_t channel = 0; channel < numberOfChannels; channel++) {
        auto data = pwm_get_channel_data(channel);      // the whole date struct for the channel
        auto config = pwm_get_channel_config(channel);  // the whole config struct for the channel

       
        /* // If you want only the channels with status=STABLE
        if (data->state != PwmState::STABLE)
            continue;
        */

        auto rawPwm = pwm_get_rawPwm(channel);  // if you use pwm_get_channel_data() you can use date->higus
        auto scaledPwm = pwm_get_scaledPwm(channel);
    
        Serial.printf("%7d\t", channel);
        Serial.printf("%7" PRId32 "\t", pwm_get_offset(channel) );
        Serial.printf("%7lu\t", rawPwm);
        Serial.printf("%7lu\t", scaledPwm);
        Serial.printf("%7lu\t", data->totalus);
        Serial.printf("%7lu\t", data->frq);
        Serial.printf("%7u\t", config->pulse_min);
        Serial.printf("%7u\t", config->pulse_neutral);
        Serial.printf("%7u\t", config->pulse_max);
        Serial.printf("%7.2f\t", pwm_get_rc_duty_cycle(config->pulse_min,
                                                       config->pulse_max,
                                                       (uint16_t)rawPwm));
        Serial.printf("%7lu\t", data->state);
        Serial.printf(" %s\n", pwm_get_state_name(channel));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("V0.03 280623 0101");

    // begin prepare pwm reading -------------------------------------------------
    // init channels
    pwm_reader_init(pins, numberOfChannels);

    // here you can change channel defaults values before reading (if needed)
    // e.g. pwm_set_channel_pulse_min() /max/neutral
    // e.g. set auto_zero/auto_min_max for channel 0-2
    for (int ch = 0; ch < 3; ch++) {
        pwm_set_auto_zero(ch, true);     // set channel to auto zero
        pwm_set_auto_min_max(ch, true);  // set channel to auto min/max calibration
    }

    // begin reading 
    esp_err_t err = pwm_reader_begin();
    if (err != ESP_OK) {
        Serial.printf("begin() err: %i", err);
    }
    // end prepare pwm reading ----------------------------------------------------

    // DEBUG:
    // if you want to read the current configuration, you can use the function pwm_get_channel_config(x)
    auto cfg = pwm_get_channel_config(0);  // read conbfig for channel 0
    ESP_LOGV(TAG, "CH: 0 min:%u max:%u neutral:%u periodMax:%u autoZero:%d",
             cfg->pulse_min,
             cfg->pulse_max,
             cfg->pulse_neutral,
             cfg->period_max,
             cfg->auto_zero);

    CLEAR_SCREEN;  // clear screen before table print
    
    ticker.attach_ms(100, readPwmSignals);  // print table every 100ms
}

void loop() {
}
