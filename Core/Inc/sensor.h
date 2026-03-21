#ifndef INJ_SENSOR_HUB_SENSOR_H
#define INJ_SENSOR_HUB_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx_hal.h"

/*
 * Set the active channel for an ADC using default channel config settings and read the raw value from
 * that ADC. ADC number of conversions must be configured to 1.
 *
 * Returns whether or not value was successfully read.
 */
bool read_from_adc_channel(ADC_HandleTypeDef * hadc, uint32_t adc_channel, uint32_t single_differential, uint32_t * result);

/*
 * Convert a raw ADC value to millivolts.
 *
 * For single-ended input, this returns the difference between input voltage (V_INP) and ground (V_REF-).
 * For differential input, this returns (V_INP - V_INN) + ADC_FULL_SCALE/2, i.e. it's centered
 * around ADC_FULL_SCALE/2. (25.4.7 in ref manual)
 */
uint16_t adc_raw_to_mv(const uint32_t raw_value);

/*
 * Convert a raw ADC value from a pressure transducer (PT) to PSI.
 */
uint16_t pt_adc_raw_to_psi(const uint32_t raw_value);

#endif /* INJ_SENSOR_HUB_SENSOR_H */
