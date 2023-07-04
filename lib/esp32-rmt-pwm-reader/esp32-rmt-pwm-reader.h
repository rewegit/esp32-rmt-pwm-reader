/**
*********************************************************************************
* @file:	esp32-rmt-pwm-reader.h
* @author:	R.W.
* @date:	28.04.23
* @brief:
* @history:
*********************************************************************************
**/

#ifndef esp32_rmt_pwm_reader_h
#define esp32_rmt_pwm_reader_h

#include <Arduino.h>
#include <esp_timer.h>       // for meassure time

#include "driver/rmt.h"      // for PWM signal detection
#include "soc/rmt_reg.h"     // for rmt channel status
#include "soc/rmt_struct.h"  // for rmt ir

// DEBUG:
#include "driver/gpio.h"  // for direct gpio access
// #define PIN_TST GPIO_NUM_4


#include "esp32-hal-log.h"  // loglevel ist set as build_flags = -DCORE_DEBUG_LEVEL in platformio.ini
static const char *TAG = "rmt_pwm_reader";

// defines for pulse calculations
#define STABLE_COUNTER 10  // periods that must be in limit before status changes from UNSTABLE to STABLE

#define PULSE_MAX_VALID 2300
#define IDLE_TRESHOLD (PULSE_MAX_VALID + 500)  // Receiving is considered finished when IDLE_TRESHOLD us no edge change (ir) occurs.

#define PERIOD_MIN_VALID 25000                 // -> 40Hz min pwm frequency
#define PERIOD_MAX_VALID 12500                 // -> 80Hz max pwm frequency

#define AUTO_ZERO_CALIBRATION true             // default for auto zero calibration
#define ZERO_CALC_LOWER_LIMIT 20               // lower limit in %. current pulse in us - ZERO_CALC_LOWER_LIMIT
#define ZERO_CALC_UPPER_LIMIT 20               // upper limit in %. current pulse in us + ZERO_CALC_UPPER_LIMIT

#define SCALE_PULSE_MIN 1000                   // scaled pulse min (default value)
#define SCALE_PULSE_NEUTRAL 1500               // scaled pulse neutral (default value)
#define SCALE_PULSE_MAX 2000                   // scaled pulse max (default value)

#define AUTO_MIN_MAX_CALIBRATION true          // default for auto min/max calibration (default value)
#define MIN_MAX_CALIBRATE_LOWER_LIMIT 20       // lower limit in % based on calculated pulse_min
#define MIN_MAX_CALIBRATE_UPPER_LIMIT 20       // upper limit in % based on calculated pulse_max
/*
This means the distance between neutral and min/max.
In the standard, this is calculated from (max - min)/2.
The respective validity range is then calculated with:
Neutral +/- (VALID_MIN_MAX_MID +/- calibrate limits (%)).

Example No AutoZero and default values
minLowerLimit = (1500 - 500) - (1500 - 500) * 20%) = 800
maxLowerLimit = (1500 - 500) + (1500 - 500) * 20%) = 1200
minupperLimit = (1500 + 500) - (1500 + 500) * 20%) = 1600
maxLowerLimit = (1500 + 500) + (1500 + 500) * 20%) = 2400

Example AutoZero neutral=1300 and default values
minLowerLimit = (1300 - 500) - (1300 - 500) * 20%) = 640
maxLowerLimit = (1300 - 500) + (1300 - 500) * 20%) = 960
minupperLimit = (1300 + 500) - (1300 + 500) * 20%) = 1440
maxLowerLimit = (1300 + 500) + (1300 + 500) * 20%) = 2160
*/
#define VALID_MIN_MAX_MID ((SCALE_PULSE_MAX - SCALE_PULSE_MIN) / 2)

#define AUTO_MIN_MAX_CALIBRATE_TIME_SEC 5   // 5 sec for calculate min max
#define GET_FAILESAFE true                  // returns pulse_neutral if state not STABLE (default value)
#define GET_FAILESAFE_DURING_MIN_MAX false  // send failsafe value during auto_min_max_calibrate

// helpers
#define MIN(A, B) ((A) <= (B) ? (A) : (B))
#define MAX(A, B) ((A) >= (B) ? (A) : (B))

/*!
 * @brief states for SM
 */
enum class PwmState { NOT_CONNECTED,
                      UNSTABLE,
                      STABLE };

/*!
 * @brief Configuration of a channel
 */
struct pwm_config {
    uint16_t pulse_min;      ///< Minimum valid pulse width
    uint16_t pulse_max;      ///< Maximum valid pulse width
    uint16_t pulse_neutral;  ///< Neutral pulse width
    uint16_t period_max;     ///< Maximum period (fmin = 20Hz)(needed for FSM to detect NO_SIGNAL)
    boolean auto_zero;       ///< if true, auto calculate pulse_neutral if state change from NOT_CONNECTED to STABLE default is AUTO_ZERO_CALIBRATION
    boolean auto_min_max;    ///< if true, auto calculate pulse_min/max if state change from NOT_CONNECTED to STABLE default is AUTO_MIN_MAX_CALIBRATION
    boolean failsafe;        ///< if true and state not STABLE get impulse returns pulse_neutral default is GET_FAILESAFE
};

/*!
 * @brief Data for a PWM channel
 */
struct pwm_data {
    volatile int64_t lastIr = 0;               ///< Timestamp of last interrupt (IDLE_TRESHOLD after falling edge)
    volatile uint32_t highus = 0;              ///< Pulse width of the current pulse
    volatile uint32_t lastTotalus = 0;         ///< width of the last period in us (needed for FSM to detect jitter)
    volatile uint32_t totalus = 0;             ///< width of the current period in us
    volatile uint16_t frq = 0;                 ///< Frequency of the current pulse in Hz
    uint8_t stableCounter = 0;                 ///< Counter for detecting stability
    uint16_t auto_pulse_min = 0;               ///< detectet min pulse width in the first 5 sec after first STABLE
    uint16_t auto_pulse_max = 0;               ///< detectet max pulse width in the first 5 sec after first STABLE
    int64_t stable_time = 0;                   ///< Timestamp of last STABLE state
    PwmState state = PwmState::NOT_CONNECTED;  ///< State of the channel (NOT_CONNECTED, UNSTABLE, or STABLE)
};

/*!
 * @brief Object representing a PWM channel
 */
typedef struct pwm_channel {
    uint8_t pin;               ///< GPIO pin for the channel
    uint8_t channel;           ///< RMT channel for the channel
    struct pwm_config config;  ///< Configuration for the channel
    struct pwm_data data;      ///< Data for the channel
} pwm_channel_t;

/*!
 * @brief Initializes the PWM reader with the given channel pins
 * @param channelPins An array of GPIO pins for each channel
 * @param numberOfPins The number of channels to initialize
 */
void pwm_reader_init(const uint8_t channelPins[], const int numberOfPins);

/*!
 * @brief configure rmt_channels and start reading
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Function pointer error.
 *     - ESP_FAIL System driver installed, can not register ISR handler for RMT
 */
esp_err_t pwm_reader_begin();

/*!
 * @brief Returns the object representing all PWM channels for debug
 * @return A pointer to an array of pwm_channel_t objects
 */
const pwm_channel_t *pwm_get_whole_channels_object();

/*!
 * @brief Returns the data for a specific PWM channel
 * @param channel The channel number
 * @return A pointer to the pwm_data object for the channel
 */
const pwm_data *pwm_get_channel_data(const uint8_t channel);

/*!
 * @brief Returns the configuration for a specific PWM channel
 * @param channel The channel number
 * @return A pointer to the pwm_config object for the channel
 */
const pwm_config *pwm_get_channel_config(const uint8_t channel);

/*!
 * @brief configure channel RMT_Channel
 * @param channel to configure
 * @param rmtChannel new RMT_Channel
 */
void pwm_set_channel_rmtChannel(const uint8_t channel, const uint8_t rmtChannel);


/*!
 * @brief configure channel pulse_min
 * @param channel to configure
 * @param pulse_min new config.pulse_min
 */
void pwm_set_channel_pulse_min(const uint8_t channel, const uint16_t pulse_min);

/*!
 * @brief configure channel pulse_max
 * @param channel to configure
 * @param pulse_max new config.pulse_min
 */
void pwm_set_channel_pulse_max(const uint8_t channel, const uint16_t pulse_max);

/*!
 * @brief configure channel pulse_neutral
 * @param channel to configure
 * @param pulse_neutral new config.pulse_min
 */
void pwm_set_channel_pulse_neutral(const uint8_t channel, const uint16_t pulse_neutral);

/*!
 * @brief configure channel period_max
 * @param channel to configure
 * @param pulse_min new config.period_max
 */
void pwm_set_channel_period_max(const uint8_t channel, const uint16_t period_max);

/*!
 * @brief configure channel auto_zero
 * @param channel to configure
 * @param pulse_min new config.auto_zero
 */
void pwm_set_auto_zero(const uint8_t channel, const boolean auto_zero);

/*!
 * @brief configure channel auto_min_max
 * @param channel to configure
 * @param pulse_min new config.auto_min_max
 */
void pwm_set_auto_min_max(const uint8_t channel, const boolean auto_min_max);

/*!
 * @brief configure channel failsafe
 * @param channel to configure
 * @param failsafe new config.failsafe
 */
void pwm_set_failsafe(const uint8_t channel, const boolean failsafe);

/*!
 * @brief set state e.g. for reset channel to new calibrate
 * @param channel to configure
 * @param state new state
 */
void pwm_set_state(const uint8_t channel, const PwmState state);

/*!
 * @brief Returns the last impuls width
 * @param channel The channel number
 * @return last impuls width for the channel in us
 *         if data->state != STABLE and config.failsafe==true -> pulse_neutral
 *         if data->state != STABLE and config.failsafe==false -> -1
 */
const int32_t pwm_get_rawPwm(const uint8_t channel);

/*!
 * @brief Returns the last scaled impuls width.  Linear scaling is performed.
 *        If the pulse duration <= pulse_neutral, scaling is performed between pulse_min and pulse_neutral.
 *        If the pulse duration > pulse_neutral, scaling is performed between pulse_neutral and pulse_max.
 * @param channel The channel number
 * @return last impuls width for the channel in us
 *         if data->state != STABLE and config.failsafe==true -> pulse_neutral
 *         if data->state != STABLE and config.failsafe==false -> -1
 */
const int32_t pwm_get_scaledPwm(const uint8_t channel);

/*!
 * @brief Get the offset of the channel to channel0
 *        If state of Channel or Channel0 != STABLE returns -1 or freq of Channel0 != freq of Channel
 *        If Channel = 0 returns 0
 * @param channel The channel number
 * @return offset in us between channel and channel0 or -1 if state != STABLE or 0 if channel == 0
 */
const int32_t pwm_get_offset(const uint8_t channel);

/*!
 * @brief Returns the actual state for a PWM channel
 * @param channel The channel number
 * @return The state of the channel (NOT_CONNECTED, UNSTABLE, or STABLE)
 */
const PwmState pwm_get_state(const uint8_t channel);

/*!
 * @brief Returns the actual state name for a PWM channel
 * @param channel The channel number
 * @return The state name of the channel (NOT_CONNECTED, UNSTABLE, or STABLE)
 */
const char *pwm_get_state_name(const uint8_t channel);

/*!
 * @brief Returns the actual frequency for a PWM channel
 * @param channel The channel number
 * @return The frequency of the channel
 */
const uint16_t pwm_get_frequency(const uint8_t channel);

/*!
 * @brief calculate duty cycle baset on pulse_min, pulse_max, pulse_current
 * @param pulse_min  Minimum valid pulse width
 * @param pulse_max  Maximum valid pulse width
 * @param pulse_current pulse width to calculate duty cycle
 * @return The calculated duty cycle
 */
const float_t pwm_get_rc_duty_cycle(const uint16_t pulse_min, const uint16_t pulse_max, const uint16_t pulse_current);

/*!
 * @brief deregister isr, delete pwm_channels, etc
 * @return some error or ESP_OK
 */
esp_err_t pwm_cleanup();

#endif /* esp32_rmt_pwm_reader_h */
