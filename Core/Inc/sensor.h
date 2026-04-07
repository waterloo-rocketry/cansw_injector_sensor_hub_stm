#ifndef INJ_SENSOR_HUB_SENSOR_H
#define INJ_SENSOR_HUB_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx_hal.h"

#include "common.h"

/*
 * @brief Get the raw value of the specified ADC and ADC channel.
 *
 * Set the active channel for an ADC using default channel config settings and read the raw value from
 * that ADC. The ADC's number of conversions must be already configured to 1. For differential input,
 * this returns (V_INP - V_INN) + ADC_FULL_SCALE/2, i.e. it's centered around ADC_FULL_SCALE/2. (25.4.7 in ref manual)
 *
 * @param hadc Pointer to the HAL ADC handle.
 * @param adc_channel ADC channel to read from. Value of ADC_HAL_EC_CHANNEL.
 * @param single_differential Whether to read in single or differential mode. Value of ADC_HAL_EC_CHANNEL_SINGLE_DIFF_ENDING.
 * @param result Pointer to store read ADC value.
 * @return Status of the config and/or read operation. One of W_SUCCESS, W_IO_ERROR, W_IO_TIMEOUT.
 */
w_status_t read_from_adc_channel(ADC_HandleTypeDef * hadc, uint32_t adc_channel, uint32_t single_differential, uint32_t * result);

/*
 * @brief Convert a raw single-ended ADC value to millivolts.
 *
 * @param raw_value Raw ADC value to convert.
 * @return Converted ADC value in millivolts.
 */
uint16_t adc_raw_single_to_mv(const uint32_t raw_value);

/*
 * @brief Convert a raw single-ended ADC value from a pressure transducer to PSI.
 *
 * @param raw_value Raw ADC value to convert.
 * @return Converted ADC value in PSI.
 */
uint16_t pt_adc_raw_single_to_psi(const uint32_t raw_value);

#endif /* INJ_SENSOR_HUB_SENSOR_H */
