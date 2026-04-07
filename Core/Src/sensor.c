#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx_hal.h"

#include "common.h"
#include "sensor.h"

#define ADC_POLL_TIMEOUT_ms 10
// General ADC channel config settings
#define ADC_SAMPLING_TIME ADC_SAMPLETIME_16CYCLES_5
#define ADC_SINGLE_DIFF ADC_SINGLE_ENDED
#define ADC_OFFSET_NUMBER ADC_OFFSET_NONE
#define ADC_OFFSET 0
#define ADC_OFFSET_SIGNED_SATURATION DISABLE;

// ADC raw to mV scaling
#define VREF_mV 3300 // Assumed, we can probably calibrate this better
#define ADC_RESOLUTION 65535 // 16-bit

// ADC raw to PSI scaling for PTs
#define PT_OFFSET_mV 600
#define PT_RANGE_PSI 1450
#define PT_RANGE_AFTER_OFFSET_mV 2400 // after subtracting PT_OFFSET_mV


w_status_t read_from_adc_channel(ADC_HandleTypeDef * hadc, uint32_t adc_channel, uint32_t single_differential, uint32_t * result) {
  ADC_ChannelConfTypeDef adc_channel_config = {0};
  adc_channel_config.SamplingTime = ADC_SAMPLING_TIME;
  adc_channel_config.SingleDiff = single_differential;
  adc_channel_config.OffsetNumber = ADC_OFFSET_NONE;
  adc_channel_config.Offset = ADC_OFFSET;
  adc_channel_config.OffsetSignedSaturation = ADC_OFFSET_SIGNED_SATURATION;
  adc_channel_config.Rank = ADC_REGULAR_RANK_1; // assume this is the only channel
  adc_channel_config.Channel = adc_channel;

  if (HAL_ADC_ConfigChannel(hadc, &adc_channel_config) != HAL_OK) {
    return W_IO_ERROR;
  }

  HAL_ADC_Start(hadc);
  if (HAL_ADC_PollForConversion(hadc, ADC_POLL_TIMEOUT_ms) != HAL_OK) {
    return W_IO_TIMEOUT;
  }

  *result = HAL_ADC_GetValue(hadc);
  return W_SUCCESS;
}


uint16_t adc_raw_single_to_mv(const uint32_t raw_value) {
  // 16-bit value in uint32_t multiplied by VREF_mV won't wrap around
  return (uint16_t) (raw_value * VREF_mV / ADC_RESOLUTION);
}

uint16_t pt_adc_raw_single_to_psi(const uint32_t raw_value) {
  /*
   * Millivolts to PSI calculation:
   *
   * 4 to 20mA PT range * 150R scale resistor = 600 to 3000 mV corresponding to 0 to 1450 PSI
   * Subtract mV by 600 to get: 0 to 2400 mV = 0 to 1450 PSI
   * Therefore value_psi = (value_mv - 600 mV) * 1450 PSI / 2400 mV.
   */
  const uint16_t value_mv = adc_raw_single_to_mv(raw_value);
  // 16-bit value in uint32_t multiplied by PT_RANGE_PSI won't wrap around
  const uint16_t value_psi = (uint16_t) (((uint32_t) value_mv - PT_OFFSET_mV) * PT_RANGE_PSI / PT_RANGE_AFTER_OFFSET_mV);
  return value_psi;
}

