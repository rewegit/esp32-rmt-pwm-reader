/**
*********************************************************************************
* @file:	esp32_rmt_pwm_reader.cpp
* @author:	R.W.
* @date:	28.04.23
* @brief:
* @history:
*********************************************************************************
**/
#include "esp32-rmt-pwm-reader.h"

// globals in esp32_rmt_pwm_reader.cpp
int _numberOfChannels;
static pwm_channel_t *pwm_channels;

static const char *PwmStateStrings[] = {
    "NOT_CONNECTED",
    "UNSTABLE",
    "STABLE"};

// handles are needed for cleanup
xQueueHandle interruptQueue;
TaskHandle_t controlTask;
rmt_isr_handle_t rmt_isr_handle;

// data to sent from isr to control task
struct isr_data {
    uint8_t channel;
    int64_t currentPwmInterruptTime;
    uint32_t highus;
};

// declare internal functions
static void pwm_control_task(void *parms);
PwmState getNewState(const uint8_t channel);
uint16_t calculateChannelZero(const uint8_t channel);
void calculateChannelMinMax(const uint8_t channel);

/**
 * Interrupt service routine handler for the RMT peripheral.
 *
 * @param arg pointer to the argument passed to the ISR
 *
 * @return void
 *
 * @throws None
 */
// Reference https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html#application-examples
static void IRAM_ATTR rmt_isr_handler(void *arg) {
    // the isr takes about 6us (measured with gpio_set_level())

    uint32_t intr_st = RMT.int_st.val;  // see rmt_struct.h  RMT Register for set ir

    for (uint8_t channel = 0; channel < _numberOfChannels; channel++) {
        // see rmt_struct.h
        // chx_rx_end on pos ch * 3 + 1 -> 1, 4, 7, ...
        uint32_t channel_mask = BIT(channel * 3 + 1);

        // only ch(n)_rx_end ir for channel i status-bit is set
        if (!(intr_st & channel_mask))
            continue;

        RMT.conf_ch[channel].conf1.rx_en = 0;                     // disable receive data
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;  // Release memory for possible transmitters

        // pointer to start of channel mem_block
        // but since idle_threshold < periode, only the item->duration0 (high-time) can be set.
        volatile rmt_item32_t *item = RMTMEM.chan[channel].data32;

        isr_data isrData = {};
        if (item) {
            // fill Queue data if item
            isrData = {.channel = channel,
                       .currentPwmInterruptTime = esp_timer_get_time(),  // current isr time in us fir this channel
                       .highus = item->duration0};                       // impulse high time in us

            xQueueSendFromISR(interruptQueue, &isrData, NULL);
        }

        RMT.conf_ch[channel].conf1.mem_wr_rst = 1;                // Set this bit to reset write ram address for channel0-7 by receiver access
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;  // lock memory for receiver
        RMT.conf_ch[channel].conf1.rx_en = 1;                     // enable receive data

        RMT.int_clr.val = channel_mask;                           // clear RMT interrupt status.
    }
}

/*!
 * @brief calculate channel data and state
 * @param parms (not needed)
 */
static void pwm_control_task(void *parms) {
    isr_data isrData = {};  // buffer for received struct
    while (true) {
        if (xQueueReceive(interruptQueue, &isrData, portMAX_DELAY)) {
            uint8_t channel = isrData.channel;
            auto &data = pwm_channels[channel].data;

            // calculate channels data
            uint64_t totalus = isrData.currentPwmInterruptTime - data.lastIr;  // period time
            data.lastTotalus = data.totalus;                                   // needed for FSM
            data.totalus = totalus;

            // Calculate signal frequency in Hz  (1000 000 us / Period duration in us)
            data.frq = (totalus > 0) ? 1000000 / totalus : 0;  // 1/(period time)

            data.highus = isrData.highus;                      // pwm pulse length in us
            data.lastIr = isrData.currentPwmInterruptTime;     // save current time for calculate next period time

            getNewState(channel);                              // FSM calculate state
        }
        vTaskDelay(1);                                         // add a small delay to prevent busy-waiting
    }
}

/**
 * Initializes the PWM reader with the given channel pins and number of channels.
 *
 * @param channelPins The array of channel pins.
 * @param numberOfChannels The number of PWM channels.
 *
 * @throws None
 */
void pwm_reader_init(const uint8_t channelPins[], const int numberOfChannels) {
    _numberOfChannels = numberOfChannels;

    // create array of channels
    pwm_channels = new pwm_channel_t[_numberOfChannels * sizeof(pwm_channel_t)];

    // init with defaults
    struct pwm_config default_config = {.pulse_min = SCALE_PULSE_MIN,
                                        .pulse_max = SCALE_PULSE_MAX,
                                        .pulse_neutral = SCALE_PULSE_NEUTRAL,
                                        .period_max = 2 * PERIOD_MIN_VALID,
                                        .auto_zero = AUTO_ZERO_CALIBRATION,
                                        .auto_min_max = AUTO_MIN_MAX_CALIBRATION,
                                        .failsafe = GET_FAILESAFE};

    struct pwm_data initial_data;
    for (int channel = 0; channel < _numberOfChannels; channel++) {
        pwm_channels[channel].pin = channelPins[channel];
        pwm_channels[channel].channel = channel;
        pwm_channels[channel].config = default_config;
        pwm_channels[channel].data = initial_data;

        // set pinMode for pwm pins
        pinMode(pwm_channels[channel].pin, INPUT_PULLDOWN);
    }
}

esp_err_t pwm_reader_begin() {
    interruptQueue = xQueueCreate(8, sizeof(isr_data));  // buffer for incoming pwm data
    if (interruptQueue == NULL) {
        return ESP_FAIL;
    }

    if (xTaskCreate(pwm_control_task, "pwm_control_task", 2048, NULL, 1, &controlTask) != pdPASS) {  // task for handle isr data and status
        return ESP_FAIL;
    }

    for (uint8_t channel = 0; channel < _numberOfChannels; channel++) {
        rmt_config_t rxConfig = {};

        rxConfig.rmt_mode = RMT_MODE_RX;
        rxConfig.channel = (rmt_channel_t)pwm_channels[channel].channel;
        rxConfig.clk_div = 80;
        rxConfig.gpio_num = (gpio_num_t)pwm_channels[channel].pin;
        rxConfig.mem_block_num = 1;
        rxConfig.rx_config.filter_en = true;
        rxConfig.rx_config.filter_ticks_thresh = 100;
        rxConfig.rx_config.idle_threshold = IDLE_TRESHOLD;

        if (rxConfig.gpio_num == GPIO_NUM_NC) {  // error check if pin connected to s/w
            return ESP_FAIL;
        }

        if (rmt_config(&rxConfig) != ESP_OK) {              // configure RMT channel
            return ESP_FAIL;
        }

        if (rmt_set_rx_intr_en(rxConfig.channel, true) != ESP_OK) {  // enable ir
            return ESP_FAIL;
        }

        if (rmt_rx_start(rxConfig.channel, 1) != ESP_OK) {  // start reading
            return ESP_FAIL;
        }
    }

    if (rmt_isr_register(rmt_isr_handler, NULL, 0, &rmt_isr_handle) != ESP_OK) {  // register isr handler
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t pwm_cleanup() {
    for (uint8_t channel = 0; channel < _numberOfChannels; channel++) {  // cleanup channels
        RMT.conf_ch[channel].conf1.mem_wr_rst = 1;                       // Set this bit to reset write ram address for channel0-7 by receiver access
        RMT.conf_ch[channel].conf1.rx_en = 0;                            // disable receive data
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;         // Release memory for possible transmitters
        if (rmt_rx_stop((rmt_channel_t)channel) != ESP_OK) {             // stop receiving data
            return ESP_FAIL;
        }
        pinMode(pwm_channels[channel].pin, INPUT);  // all pins standard input
    }

    if (rmt_isr_deregister(rmt_isr_handle) != ESP_OK) {  // deregister isr handler
        return ESP_FAIL;
    }

    vQueueDelete(interruptQueue);
    vTaskDelete(controlTask);
    delete[] pwm_channels;

    return ESP_OK;
}

const pwm_channel_t *pwm_get_whole_channels_object() {
    return pwm_channels;
}

const pwm_data *pwm_get_channel_data(const uint8_t channel) {
    return &pwm_channels[channel].data;
}

const pwm_config *pwm_get_channel_config(const uint8_t channel) {
    return &pwm_channels[channel].config;
}

void pwm_set_channel_rmtChannel(const uint8_t channel, const uint8_t rmtChannel) {
    pwm_channels[channel].channel = rmtChannel;
}

void pwm_set_channel_pulse_min(const uint8_t channel, const uint16_t pulse_min) {
    pwm_channels[channel].config.pulse_min = pulse_min;
}

void pwm_set_channel_pulse_max(const uint8_t channel, const uint16_t pulse_max) {
    pwm_channels[channel].config.pulse_max = pulse_max;
}

void pwm_set_channel_pulse_neutral(const uint8_t channel, const uint16_t pulse_neutral) {
    pwm_channels[channel].config.pulse_neutral = pulse_neutral;
}

void pwm_set_channel_period_max(const uint8_t channel, const uint16_t period_max) {
    pwm_channels[channel].config.period_max = period_max;
}

void pwm_set_auto_zero(const uint8_t channel, const boolean auto_zero) {
    pwm_channels[channel].config.auto_zero = auto_zero;
}

void pwm_set_auto_min_max(const uint8_t channel, const boolean auto_min_max) {
    pwm_channels[channel].config.auto_min_max = auto_min_max;
}

void pwm_set_failsafe(const uint8_t channel, const boolean failsafe) {
    pwm_channels[channel].config.failsafe = failsafe;
}

const PwmState pwm_get_state(const uint8_t channel) {
    return pwm_channels[channel].data.state;
}

void pwm_set_state(const uint8_t channel, const PwmState state) {
    pwm_channels[channel].data.state = state;
}

const char *pwm_get_state_name(const uint8_t channel) {
    return PwmStateStrings[static_cast<size_t>(pwm_channels[channel].data.state)];
}

const uint16_t pwm_get_frequency(const uint8_t channel) {
    return pwm_channels[channel].data.frq;
}

const int32_t pwm_get_rawPwm(const uint8_t channel) {
    const auto &data = pwm_channels[channel].data;
    const auto &config = pwm_channels[channel].config;

    if (data.state == PwmState::STABLE) {
        return static_cast<int32_t>(GET_FAILESAFE_DURING_MIN_MAX && config.auto_min_max)
                   ? config.pulse_neutral
                   : data.highus;
    }
    return static_cast<int32_t>(config.failsafe)
               ? config.pulse_neutral
               : -1;
}

/*!
 * @brief Calculates and returns offset based on CH0
 * @param channel The channel number
 * @return offset based on CH0
 */
int32_t calculateOffset(const uint8_t channel) {
    const auto &data = pwm_channels[channel].data;

    if ((data.state != PwmState::STABLE) ||
        (pwm_channels[0].data.state != PwmState::STABLE)) {
        return 0;
    }

    int frq_diff = static_cast<int>((data.frq) -
                                    (pwm_channels[0].data.frq));
    // offset only make sense if frequency of channels is equal (< 2Hz is Jitter)
    if (channel == 0 || (abs(frq_diff) > 2)) {
        return 0;
    }

    int64_t time_diff = (data.lastIr - pwm_channels[0].data.lastIr);

    int64_t impulse_diff = static_cast<int64_t>(pwm_channels[0].data.highus) -
                           static_cast<int64_t>(data.highus);

    // If this query is already made at a moment when a new ir has already occurred at ch0,
    // the distance between ch0 and the measured channel is too large by exactly one period.
    // This is taken into account here,
    if ((abs(time_diff) << 1) > static_cast<int64_t>(data.totalus)) {
        time_diff = (time_diff < 0)
                        ? static_cast<int64_t>(data.totalus) % time_diff
                        : -(static_cast<int64_t>(data.totalus) % time_diff);
    }

    return time_diff + impulse_diff;
}

const int32_t pwm_get_offset(const uint8_t channel) {
    return calculateOffset(channel);
}

const int32_t pwm_get_scaledPwm(const uint8_t channel) {
    const auto &data = pwm_channels[channel].data;
    const auto &config = pwm_channels[channel].config;

    if (data.state == PwmState::STABLE) {
        if (GET_FAILESAFE_DURING_MIN_MAX && config.auto_min_max) {
            return config.pulse_neutral;
        }

        if (data.highus <= config.pulse_neutral) {
            return map(data.highus,
                       config.pulse_min,
                       config.pulse_neutral,
                       SCALE_PULSE_MIN,
                       SCALE_PULSE_NEUTRAL);
        }

        return map(data.highus,
                   config.pulse_neutral,
                   config.pulse_max,
                   SCALE_PULSE_NEUTRAL,
                   SCALE_PULSE_MAX);
    }

    return (config.failsafe) ? config.pulse_neutral : -1;
}

const float_t pwm_get_rc_duty_cycle(const uint16_t pulse_min, const uint16_t pulse_max, const uint16_t pulse_current) {
    if ((pulse_min + pulse_max) == 0) {
        return 0;
    }
    return 100.0 * pulse_current / (pulse_min + pulse_max);
}

/*!
 * @brief Calculates pulse_min, pulse_max after first STABLE based on logged min/max values.
 *        The control unit must be set to minimum and maximum position within AUTO_MIN_MAX_CALIBRATE_TIME_SEC.
 * @param channel The channel number
 * @return The new pulse_neutral of the channel
 */
void calculateChannelMinMax(const uint8_t channel) {
    auto &data = pwm_channels[channel].data;
    auto &config = pwm_channels[channel].config;

    uint16_t lowerMid = (config.pulse_neutral - VALID_MIN_MAX_MID);
    uint16_t upperMid = (config.pulse_neutral + VALID_MIN_MAX_MID);

    // limits for pulse_min
    uint16_t lowerLimitMin = lowerMid - (MIN_MAX_CALIBRATE_LOWER_LIMIT * (lowerMid / 100));
    uint16_t upperLimitMin = lowerMid + (MIN_MAX_CALIBRATE_LOWER_LIMIT * (lowerMid / 100));

    // limits for pulse_max
    uint16_t lowerLimitMax = upperMid - (MIN_MAX_CALIBRATE_UPPER_LIMIT * (upperMid / 100));
    uint16_t upperLimitMax = upperMid + (MIN_MAX_CALIBRATE_UPPER_LIMIT * (upperMid / 100));

    if (((esp_timer_get_time() - data.stable_time) / 1e6) >= AUTO_MIN_MAX_CALIBRATE_TIME_SEC) {  // stable since AUTO_MIN_MAX_CALIBRATE_TIME_SEC ?
        // check clausibility
        if (data.auto_pulse_min > lowerLimitMin && data.auto_pulse_min < upperLimitMin) {
            config.pulse_min = data.auto_pulse_min;
        }
        if (data.auto_pulse_max > lowerLimitMax && data.auto_pulse_max < upperLimitMax) {
            config.pulse_max = data.auto_pulse_max;
        }

        // clear auto_min_max flag
        config.auto_min_max = false;

    } else {
        // calculate and save min/max
        data.auto_pulse_min = data.auto_pulse_min > 0
                                  ? MIN(data.highus, data.auto_pulse_min)
                                  : data.highus;
        data.auto_pulse_max = MAX(data.highus, data.auto_pulse_max);
    }
}

/*!
 * @brief Calculates and returns pulse_neutral based on current PWM position
 *        at status change from NOT_CONNECTED to STABLE.
 *        The current pwm impulse length must be default pulse_neutral +- ZERO_CALC_UPPER_LIMIT/-LOWER_LIMIT %
 *        DON'T USE IT FOR CHANNELS WITHOUT SPRING CENTERED NEUTRAL POSITION AND SWITCHES
 * @param channel The channel number
 * @return The new pulse_neutral of the channel
 */
uint16_t calculateChannelZero(const uint8_t channel) {
    const auto &data = pwm_channels[channel].data;
    const auto &config = pwm_channels[channel].config;

    uint16_t lowerLimit = config.pulse_neutral - ((ZERO_CALC_LOWER_LIMIT * config.pulse_neutral) / 100);
    uint16_t upperLimit = config.pulse_neutral + ((ZERO_CALC_UPPER_LIMIT * config.pulse_neutral) / 100);

    if (data.highus > lowerLimit && data.highus < upperLimit) {
        return data.highus;
    }

    return config.pulse_neutral;
}

/*!
 * @brief FSM for calculate and returns the new state for a PWM channel
 * @param channel The channel number
 * @return The new state of the channel (NOT_CONNECTED, UNSTABLE, or STABLE)
 */
PwmState getNewState(const uint8_t channel) {
// period is not in valid range or successive periods differ by 20% (within the stable counter)
#define PWM_OUT_OF_RANGE ((data.totalus > PERIOD_MIN_VALID) || (data.totalus < PERIOD_MAX_VALID) && (data.totalus <= 0.8 * data.lastTotalus || data.totalus >= 1.2 * data.lastTotalus))

// NO_SIGNAL detection
#define NO_SIGNAL ((currentTime - data.lastIr) > (2 * (uint64_t)config.period_max))

    auto &data = pwm_channels[channel].data;
    auto &config = pwm_channels[channel].config;

    PwmState newState = data.state;  // default newState = oldState

    int64_t currentTime = esp_timer_get_time();

    switch (data.state) {
        case PwmState::NOT_CONNECTED:
            newState = PwmState::UNSTABLE;
            data.stableCounter = 0;
            break;

        case PwmState::UNSTABLE:
            if (NO_SIGNAL) {
                newState = PwmState::NOT_CONNECTED;
                break;
            }
            if (PWM_OUT_OF_RANGE) {
                newState = PwmState::UNSTABLE;
                data.stableCounter = 0;
                break;
            }
            if (data.stableCounter == STABLE_COUNTER) {  // in range and STABLE_COUNTER reached
                data.stableCounter = 0;
                data.stable_time = currentTime;
                newState = PwmState::STABLE;
                if (config.auto_zero == true) {
                    config.pulse_neutral = calculateChannelZero(channel);
                }
                break;
            }
            data.stableCounter++;  // in range and STABLE_COUNTER not reached
            break;

        case PwmState::STABLE:
            if (NO_SIGNAL) {
                newState = PwmState::NOT_CONNECTED;
                break;
            }
            if (PWM_OUT_OF_RANGE) {
                newState = PwmState::UNSTABLE;
                data.stableCounter = 0;
                break;
            }
            if (config.auto_min_max) {
                calculateChannelMinMax(channel);
                break;
            }
    }
    data.state = newState;
    return newState;
}
