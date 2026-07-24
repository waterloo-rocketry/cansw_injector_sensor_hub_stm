#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "canlib.h"
#include "common.h"
#include "low_pass_filter.h"

#include "main.h"
#include "platform.h"
#include "sd_log.h"
#include "sensor.h"

// ADC raw to mV scaling
#define VREF_mV 3300 // Assumed, we can maybe calibrate this better
#define ADC_RESOLUTION 65535 // 16-bit

// Used to convert mV -> 0.1 PSI.

// Kulite: Max range 70 bar = 1015 PSI, corresponding to 5V, scaled to 3.066 V at ADC
#define KULITE_PSI_PER_10_mV (1015.0 * 10.0 / 3066.0)
// IFM 5402: 4 to 20mA scaled to 600 to 3000 mV at ADC corresponds to 0 to 2000 PSI
#define IFM_5402_PSI_PER_10_mV (2000.0 * 10.0 / 2400.0)
#define IFM_5402_OFFSET_mV 600

// multiply by four to get actual battery voltage
#define V_BATT_UPPER_THRESHOLD_mV 3250 // 13 V
#define V_BATT_LOWER_THRESHOLD_mV 2500 // 10 V

// Delay between sending each CAN message
#define CAN_SEND_DELAY_ms 5

// ADC raw to PSI scaling for PTs
// #define PT_OFFSET_mV 600
// #define PT_RANGE_PSI 1450
// #define PT_RANGE_AFTER_OFFSET_mV 2400 // after subtracting PT_OFFSET_mV

w_status_t handle_adc_scan_ready(volatile uint16_t *adc_channels_buffer,
								 analog_sensor_handle_t *sensor_handles,
								 uint8_t analog_sensor_count) {
	w_status_t status = W_SUCCESS;

	for (uint8_t i = 0; i < analog_sensor_count; ++i) {
		analog_sensor_handle_t *sensor_h = &(sensor_handles[i]);
		if (--(sensor_h->freq_div_counter) == 0) {
			sensor_h->freq_div_counter = sensor_h->config.sample_freq_divider;

			uint16_t value_mv = ((uint32_t)adc_channels_buffer[sensor_h->config.signal_pos_adc_index]) * VREF_mV / ADC_RESOLUTION;
			if (sensor_h->config.is_differential) {
			  value_mv -= ((uint32_t)adc_channels_buffer[sensor_h->config.signal_neg_adc_index]) * VREF_mV / ADC_RESOLUTION;
			}
			if (sensor_h->config.low_pass_enabled) {
				double low_pass_output = (double)sensor_h->reading_value_mv;
				if (update_low_pass(sensor_h->config.low_pass_alpha, value_mv, &low_pass_output) ==
					W_SUCCESS) {
					sensor_h->reading_value_mv = (uint16_t)low_pass_output;
				} else {
					sensor_h->reading_value_mv = value_mv;
				}
			} else {
				sensor_h->reading_value_mv = value_mv;
			}

			w_status_t on_read_status =
				sensor_h->config.on_read(sensor_h->reading_value_mv, sensor_h->config.sensor_id);

			if (on_read_status != W_SUCCESS) {
				status = on_read_status;
			}
		}
	}

	return status;
}

static w_status_t sensor_on_read_pt(uint32_t value_tenth_psi, can_analog_sensor_id_t sensor_id) {
    w_status_t status = W_SUCCESS;
    can_msg_t sensor_msg;
    build_analog_sensor_16bit_msg(PRIO_LOW, (uint16_t)millis(), sensor_id, value_tenth_psi, &sensor_msg);
    if (!stm32h7_can_send(&sensor_msg)) {
        status = W_FAILURE;
    }
    if (!sd_fs_init_failed) {
        sd_log_can_message(&sensor_msg, millis());
    }
    // USB debug drops messages when sending at very small interval apart
    HAL_Delay(CAN_SEND_DELAY_ms);

    return status;
}

w_status_t sensor_on_read_pt_kulite(uint32_t value_mv, can_analog_sensor_id_t sensor_id) {
  return sensor_on_read_pt((uint32_t) (value_mv * KULITE_PSI_PER_10_mV), sensor_id);
}

w_status_t sensor_on_read_pt_ifm_5402(uint32_t value_mv, can_analog_sensor_id_t sensor_id) {
  if (value_mv < IFM_5402_OFFSET_mV) {
    value_mv = 600;
  }
  return sensor_on_read_pt((uint32_t) ((value_mv - IFM_5402_OFFSET_mV) * IFM_5402_PSI_PER_10_mV), sensor_id);
}

w_status_t sensor_on_read_v_batt(uint32_t value_mv, can_analog_sensor_id_t sensor_id) {
	w_status_t status = W_SUCCESS;
	can_msg_t sensor_msg;
	build_analog_sensor_16bit_msg(PRIO_LOW, (uint16_t)millis(), sensor_id, value_mv, &sensor_msg);
	if (!stm32h7_can_send(&sensor_msg)) {
		status = W_FAILURE;
	}
	if (value_mv > V_BATT_UPPER_THRESHOLD_mV) {
		general_board_status |= (1 << E_12V_OVER_VOLT_OFFSET);
	} else if (value_mv < V_BATT_LOWER_THRESHOLD_mV) {
		general_board_status |= (1 << E_12V_UNDER_VOLT_OFFSET);
	}
	// USB debug drops messages when sending at very small interval apart
	HAL_Delay(CAN_SEND_DELAY_ms);

	return status;
}

