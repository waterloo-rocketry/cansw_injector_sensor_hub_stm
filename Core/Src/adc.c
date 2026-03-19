#include "adc.h"

#define ADC_POLL_TIMEOUT_ms 10

// General ADC channel config settings
#define ADC_SAMPLING_TIME ADC_SAMPLETIME_16CYCLES_5
#define ADC_SINGLE_DIFF ADC_SINGLE_ENDED
#define ADC_OFFSET_NUMBER ADC_OFFSET_NONE
#define ADC_OFFSET 0
#define ADC_OFFSET_SIGNED_SATURATION DISABLE;

bool read_from_adc_channel(ADC_HandleTypeDef * hadc, uint32_t adc_channel, uint32_t * result) {
  ADC_ChannelConfTypeDef adc_channel_config = {0};
  adc_channel_config.SamplingTime = ADC_SAMPLING_TIME;
  adc_channel_config.SingleDiff = ADC_SINGLE_ENDED;
  adc_channel_config.OffsetNumber = ADC_OFFSET_NONE;
  adc_channel_config.Offset = ADC_OFFSET;
  adc_channel_config.OffsetSignedSaturation = ADC_OFFSET_SIGNED_SATURATION;
  adc_channel_config.Rank = ADC_REGULAR_RANK_1; // assume this is the only channel
  adc_channel_config.Channel = adc_channel;

  if (HAL_ADC_ConfigChannel(hadc, &adc_channel_config) != HAL_OK) {
    return false;
  }

  HAL_ADC_Start(hadc);
  if (HAL_ADC_PollForConversion(hadc, ADC_POLL_TIMEOUT_ms) != HAL_OK) {
    return false;
  }

  *result = HAL_ADC_GetValue(hadc);
  return true;
}
