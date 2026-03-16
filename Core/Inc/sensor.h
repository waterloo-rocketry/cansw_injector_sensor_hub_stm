#ifndef INJ_SENSOR_HUB_SENSOR_H
#define INJ_SENSOR_HUB_SENSOR_H

#include <stdint.h>

/*
 * Convert a raw ADC value to millivolts.
 */
uint16_t adc_raw_to_mv(const uint32_t raw_value);

/*
 * Convert a raw ADC value from a pressure transducer (PT) to PSI.
 */
uint16_t pt_adc_raw_to_psi(const uint32_t raw_value);

#endif /* INJ_SENSOR_HUB_SENSOR_H */
