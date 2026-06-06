#ifndef INJ_SENSOR_HUB_SENSOR_H
#define INJ_SENSOR_HUB_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "canlib.h"
#include "common.h"

/**
 * @brief Config type of an analog sensor (one adc channel).
 *
 * @param sample_freq_divider  Only sample one of every sample_freq_divider ADC readings (divides
 * global ADC sample rate).
 * @param on_read              Callback invoked with the processed reading. Takes the value in mV
 * and the sensor's CAN ID.
 * @param sensor_id            The sensor's canlib ID.
 * @param low_pass_enabled     Whether low pass filter is enabled.
 * @param low_pass_alpha       Alpha value for low pass filter, within range (0, 1] where 1 is no
 * smoothing/filter at all.
 */
typedef struct {
	uint8_t sample_freq_divider;
	w_status_t (*on_read)(uint32_t value_mv, can_analog_sensor_id_t sensor_id);
	can_analog_sensor_id_t sensor_id;
	bool low_pass_enabled;
	double low_pass_alpha;
} analog_sensor_config_t;

/*
 * @brief Handle type of an analog sensor (one adc channel).
 *
 * @param config Configuration of sensor.
 * @param reading_value_mv Reading value in millivolts, also used to get previous value in low pass
 * filter.
 * @param freq_div_counter Counter used to apply sample_freq_divider.
 */
typedef struct {
	analog_sensor_config_t config;
	uint16_t reading_value_mv;
	uint8_t freq_div_counter;
} analog_sensor_handle_t;

w_status_t handle_adc_scan_ready(volatile uint16_t *adc_channels_buffer,
								 analog_sensor_handle_t *sensor_handles, uint8_t adc_channel_count);

w_status_t sensor_on_read_pt(uint32_t value_mv, can_analog_sensor_id_t sensor_id);

w_status_t sensor_on_read_v_batt(uint32_t value_mv, can_analog_sensor_id_t sensor_id);

#endif /* INJ_SENSOR_HUB_SENSOR_H */
