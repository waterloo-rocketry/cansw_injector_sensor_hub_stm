#include "sensor.h"

#include <stdint.h>

#define VREF_mV 3300 // Assumed, we can probably calibrate this better
#define ADC_RESOLUTION 65535 // 16-bit

#define PT_OFFSET_mV 600
#define PT_RANGE_PSI 1450
#define PT_RANGE_AFTER_OFFSET_mV 2400 // after subtracting PT_OFFSET_mV


uint16_t adc_raw_to_mv(const uint32_t raw_value) {
  // 16-bit value in uint32_t multiplied by VREF_mV won't wrap around
  return (uint16_t) (raw_value * VREF_mV / ADC_RESOLUTION);
}

uint16_t pt_adc_raw_to_psi(const uint32_t raw_value) {
  /*
   * Millivolts to PSI calculation:
   *
   * 4 to 20mA PT range * 150R scale resistor = 600 to 3000 mV corresponding to 0 to 1450 PSI
   * Subtract mV by 600 to get: 0 to 2400 mV = 0 to 1450 PSI
   * Therefore value_psi = (value_mv - 600 mV) * 1450 PSI / 2400 mV.
   */
  const uint16_t value_mv = adc_raw_to_mv(raw_value);
  // 16-bit value in uint32_t multiplied by PT_RANGE_PSI won't wrap around
  const uint16_t value_psi = (uint16_t) (((uint32_t) value_mv - PT_OFFSET_mV) * PT_RANGE_PSI / PT_RANGE_AFTER_OFFSET_mV);
  return value_psi;
}
