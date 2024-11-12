#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_saadc.h>
#include <nrfx_gpiote.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>

#include "adc_ble_max_throughput_service.h"
#include "hal/nrf_saadc.h"

LOG_MODULE_REGISTER(adc);

#define TIMER_INST_IDX 1
nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);

#define ACQ_TIME_10K 3UL
#define CONV_TIME 2UL

// Using a more reasonable frequency for BLE transmission
#define MAX_SAADC_SAMPLE_FREQUENCY 200000UL
#define SAADC_SAMPLE_FREQUENCY 32000UL

#define TIME_TO_WAIT_US (uint32_t)(1000000UL / SAADC_SAMPLE_FREQUENCY)

#define RESOLUTION NRF_SAADC_RESOLUTION_12BIT
#define BUFFER_COUNT 4
#define BUFFER_SIZE 120

// For continuous sampling the sample rate should fulfill: SAADC_SAMPLE_FREQUENCY <= (1000000 / (ACQ_TIME_10K + CONV_TIME))
NRFX_STATIC_ASSERT(SAADC_SAMPLE_FREQUENCY <= (1000000UL / (ACQ_TIME_10K + CONV_TIME)));

static const nrfx_saadc_channel_t channels[] = {
    {
        .channel_config = {
            .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
            .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
            .gain = NRF_SAADC_GAIN1_6,
            .reference = NRF_SAADC_REFERENCE_INTERNAL,
            .acq_time = NRF_SAADC_ACQTIME_3US,
            .mode = NRF_SAADC_MODE_SINGLE_ENDED,
            .burst = NRF_SAADC_BURST_DISABLED,
        },
        .pin_p = NRF_SAADC_INPUT_AIN0,
        .pin_n = NRF_SAADC_INPUT_DISABLED,
        .channel_index = 0,
    },
    {
        .channel_config = {
            .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
            .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
            .gain = NRF_SAADC_GAIN1_6,
            .reference = NRF_SAADC_REFERENCE_INTERNAL,
            .acq_time = NRF_SAADC_ACQTIME_3US,
            .mode = NRF_SAADC_MODE_SINGLE_ENDED,
            .burst = NRF_SAADC_BURST_DISABLED,
        },
        .pin_p = NRF_SAADC_INPUT_AIN1,
        .pin_n = NRF_SAADC_INPUT_DISABLED,
        .channel_index = 1,
    },
    {
        .channel_config = {
            .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
            .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
            .gain = NRF_SAADC_GAIN1_6,
            .reference = NRF_SAADC_REFERENCE_INTERNAL,
            .acq_time = NRF_SAADC_ACQTIME_3US,
            .mode = NRF_SAADC_MODE_SINGLE_ENDED,
            .burst = NRF_SAADC_BURST_DISABLED,
        },
        .pin_p = NRF_SAADC_INPUT_AIN2,
        .pin_n = NRF_SAADC_INPUT_DISABLED,
        .channel_index = 2,
    },
    {
        .channel_config = {
            .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
            .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
            .gain = NRF_SAADC_GAIN1_6,
            .reference = NRF_SAADC_REFERENCE_INTERNAL,
            .acq_time = NRF_SAADC_ACQTIME_3US,
            .mode = NRF_SAADC_MODE_SINGLE_ENDED,
            .burst = NRF_SAADC_BURST_DISABLED,
        },
        .pin_p = NRF_SAADC_INPUT_AIN3,
        .pin_n = NRF_SAADC_INPUT_DISABLED,
        .channel_index = 3,
    },
};

#define CHANNEL_COUNT (sizeof(channels) / sizeof(nrfx_saadc_channel_t))

static uint16_t samples_buffers[BUFFER_COUNT][BUFFER_SIZE];
static uint8_t m_gppi_channels[2];

typedef enum {
    SAADC_SAMPLING,     // Triggers SAADC sampling task on external timer event
    SAADC_START_ON_END, // Triggers SAADC start task on SAADC end event
} gppi_channels_purpose_t;

static void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    // Empty handler - GPPI handles the timer->SAADC connection
}

extern struct k_msgq abmt_data_queue;

volatile int total_samples_events = 0;

static void saadc_handler(nrfx_saadc_evt_t const * p_event)
{
    nrfx_err_t status;
    static uint16_t buffer_index = 1;

    switch (p_event->type) {
        case NRFX_SAADC_EVT_CALIBRATEDONE:
            LOG_DBG("SAADC event: CALIBRATEDONE");
            status = nrfx_saadc_mode_trigger();
            if (status != NRFX_SUCCESS) {
                LOG_ERR("Failed to trigger mode: %d", status);
            }
            break;

        case NRFX_SAADC_EVT_READY:
            LOG_DBG("SAADC event: READY");
            nrfx_gppi_channels_enable(NRFX_BIT(m_gppi_channels[SAADC_SAMPLING]));
            break;

        case NRFX_SAADC_EVT_BUF_REQ:
            LOG_DBG("SAADC event: BUF_REQ");
            // Set next buffer in the rotation
            status = nrfx_saadc_buffer_set(samples_buffers[buffer_index], BUFFER_SIZE);
            if (status != NRFX_SUCCESS) {
                LOG_ERR("Failed to set buffer: %d", status);
            }
            buffer_index = (buffer_index + 1) % BUFFER_COUNT;
            break;

        case NRFX_SAADC_EVT_DONE:
            LOG_DBG("SAADC event: DONE - Buffer %p, Size %d", 
                   (void *)p_event->data.done.p_buffer, 
                   p_event->data.done.size);
            
            total_samples_events += p_event->data.done.size;
            
            // Send completed buffer to BLE service queue
            status = k_msgq_put(&abmt_data_queue, 
						  (void *)p_event->data.done.p_buffer, 
						  K_NO_WAIT);
			if (status != 0 && status != -ENOMSG) {
				LOG_ERR("Failed to submit message to queue: %d", status);
			}
            break;

        case NRFX_SAADC_EVT_FINISHED:
            LOG_DBG("SAADC event: FINISHED");
            nrfx_gppi_channels_disable(NRFX_BIT(m_gppi_channels[SAADC_SAMPLING]));
            break;

        default:
            LOG_WRN("Unhandled SAADC event: %d", p_event->type);
            break;
    }
}

int main(void)
{
    nrfx_err_t status;

    // Connect IRQ handlers
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, 
                nrfx_saadc_irq_handler, 0, 0);
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER1), IRQ_PRIO_LOWEST, 
                nrfx_timer_1_irq_handler, 0, 0);

    LOG_INF("Starting ADC example with %ld Hz sampling", SAADC_SAMPLE_FREQUENCY);

    // Initialize SAADC
    status = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("SAADC init failed: %d", status);
        return -1;
    }

    // Configure all channels
    for (size_t i = 0; i < CHANNEL_COUNT; i++) {
        status = nrfx_saadc_channel_config(&channels[i]);
        if (status != NRFX_SUCCESS) {
            LOG_ERR("Channel %d config failed: %d", i, status);
            return -1;
        }
    }

    // Initialize timer
    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_config.p_context = &timer_inst;

    status = nrfx_timer_init(&timer_inst, &timer_config, timer_handler);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("Timer init failed: %d", status);
        return -1;
    }

    nrfx_timer_clear(&timer_inst);

    // Calculate and set up timer period
    uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer_inst, TIME_TO_WAIT_US);
    LOG_INF("Timer period: %u us (%u ticks)", TIME_TO_WAIT_US, desired_ticks);
    
    nrfx_timer_extended_compare(&timer_inst,
                               NRF_TIMER_CC_CHANNEL0,
                               desired_ticks,
                               NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                               false);

    // Configure SAADC advanced mode
    nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    adv_config.internal_timer_cc = 0;   // Disable internal timer - using external timer
    adv_config.start_on_end = false;    // Using GPPI for this instead
    adv_config.burst = NRF_SAADC_BURST_ENABLED;
    adv_config.oversampling = NRF_SAADC_OVERSAMPLE_2X;

    uint32_t channel_mask = nrfx_saadc_channels_configured_get();
    status = nrfx_saadc_advanced_mode_set(channel_mask,
                                         RESOLUTION,
                                         &adv_config,
                                         saadc_handler);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("Advanced mode set failed: %d", status);
        return -1;
    }

    // Set up initial buffers
    status = nrfx_saadc_buffer_set(samples_buffers[0], BUFFER_SIZE);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("Initial buffer set failed: %d", status);
        return -1;
    }

    // Configure GPPI for timer->SAADC sampling trigger
    status = nrfx_gppi_channel_alloc(&m_gppi_channels[SAADC_SAMPLING]);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("GPPI sampling channel allocation failed: %d", status);
        return -1;
    }

    nrfx_gppi_channel_endpoints_setup(m_gppi_channels[SAADC_SAMPLING],
        nrfx_timer_compare_event_address_get(&timer_inst, NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    // Configure GPPI for SAADC start-on-end
    status = nrfx_gppi_channel_alloc(&m_gppi_channels[SAADC_START_ON_END]);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("GPPI start-on-end channel allocation failed: %d", status);
        return -1;
    }

    nrfx_gppi_channel_endpoints_setup(m_gppi_channels[SAADC_START_ON_END],
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    // Enable timer and GPPI
    nrfx_timer_enable(&timer_inst);
    nrfx_gppi_channels_enable(NRFX_BIT(m_gppi_channels[SAADC_START_ON_END]));

    // Calibrate and start SAADC
    status = nrfx_saadc_offset_calibrate(saadc_handler);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("Offset calibration failed: %d", status);
        return -1;
    }

    LOG_INF("ADC setup complete, entering main loop");

    while (1) {
        LOG_INF("Total samples collected: %d", total_samples_events);
        k_msleep(1000);
    }

    return 0;
}
