#ifndef INJ_SENSOR_ADC_H
#define INJ_SENSOR_ADC_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx_hal.h"

/*
 * Set the active channel for an ADC using default channel config settings and read the raw value from
 * that ADC. ADC number of conversions must be configured to 1.
 *
 * Returns whether or not value was successfully read.
 */
bool read_from_adc_channel(ADC_HandleTypeDef * hadc, uint32_t adc_channel, uint32_t * result);

#endif /* INJ_SENSOR_ADC_H */
