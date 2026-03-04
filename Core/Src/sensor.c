#include "sensor.h"

#include <stdint.h>

#define VREFINT_mV 2500.0
#define VREFINT_READING_LSB 25772.0

#define PT_OFFSET_mV 600.0
#define PT_RANGE_PSI 1450.0
#define PT_RANGE_AFTER_OFFSET_mV 2400.0 // after subtracting PT_OFFSET_mV


uint16_t adc_raw_to_mv(const uint16_t raw_value) {
  /* ADC raw to millivolts calculation:
   *
   * TODO: Change this calculation as specified in ref manual 25.4.35 instead
   * of just assuming VREFINT=2.5V
   *
   * VREFINT corresponds to ~2.5V as per ref manual so to convert read value X from LSB to mV:
   * VREFINT_mV = 2500 mV
   * VREFINT_READING_LSB = raw ADC value reading of VREFINT (took average over 1000 reads)
   * (X LSB) * (VREFINT_mV) / (VREFINT_READING_LSB) mV
  */
  return (uint16_t) (raw_value * VREFINT_mV / VREFINT_READING_LSB);
}

uint16_t pt_adc_raw_to_psi(const uint16_t raw_value) {
  /*
   * Millivolts to PSI calculation:
   *
   * 4 to 20mA PT range * 150R scale resistor = 600 to 3000 mV corresponding to 0 to 1450 PSI
   * Subtract mV by 600 to get: 0 to 2400 mV = 0 to 1450 PSI
   * giving 1450PSI/2400mV = 29/48 PSI/mV.
   * Therefore value_psi = (value_mv - 600 mV) * 29 / 48.
   */
  const uint16_t value_mv = adc_raw_to_mv(raw_value);
  const uint16_t value_psi = (uint16_t) ((value_mv - PT_OFFSET_mV) * PT_RANGE_PSI / PT_RANGE_AFTER_OFFSET_mV);
  return value_psi;
}
