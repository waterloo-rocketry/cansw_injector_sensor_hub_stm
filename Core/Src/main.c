/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "canlib.h"
#include "common.h"
#include "low_pass_filter.h"
#include "platform.h"
#include "sd_fs.h"
#include "sd_log.h"
#include "sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_BUS_DEAD_TIME_ms 1000

#define ADC_SAMPLE_FREQ_Hz 50
#define ADC_SAMPLE_INTERVAL_ms (1000.0 / ADC_SAMPLE_FREQ_Hz)

#define ADC1_CHANNEL_COUNT 4
#define ADC3_CHANNEL_COUNT 2

#define PT1_LOW_PASS_ENABLED false
#define PT1_LOW_PASS_RESPONSE_TIME_ms 2500.0
#define PT1_FREQ_DIVIDER 1

#define PT2_LOW_PASS_ENABLED false
#define PT2_LOW_PASS_RESPONSE_TIME_ms 2500.0
#define PT2_FREQ_DIVIDER 1

#define PT3_LOW_PASS_ENABLED false
#define PT3_LOW_PASS_RESPONSE_TIME_ms 2500.0
#define PT3_FREQ_DIVIDER 1

#define PT4_LOW_PASS_ENABLED false
#define PT4_LOW_PASS_RESPONSE_TIME_ms 2500.0
#define PT4_FREQ_DIVIDER 1

#define PT5_LOW_PASS_ENABLED false
#define PT5_LOW_PASS_RESPONSE_TIME_ms 2500.0
#define PT5_FREQ_DIVIDER 1

#define PT6_LOW_PASS_ENABLED false
#define PT6_LOW_PASS_RESPONSE_TIME_ms 2500.0
#define PT6_FREQ_DIVIDER 1

#define STATUS_CHECK_INTERVAL_ms 500

// When defined, disable sd card logging
 #define SD_DISABLE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LOW_PASS_ALPHA(RESPONSE_TIME, SAMPLE_FREQ_Hz)                                              \
	(((double)SAMPLE_FREQ_Hz * RESPONSE_TIME / 5.0) /                                              \
	 (1 + (double)SAMPLE_FREQ_Hz * RESPONSE_TIME / 5.0))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

FDCAN_HandleTypeDef hfdcan1;

SD_HandleTypeDef hsd1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_ADC3_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
static void can_callback(const can_msg_t *msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static volatile bool seen_can_msg = false;

static volatile bool adc1_data_ready = false;
static volatile bool adc3_data_ready = false;

volatile uint16_t adc1_read_buffer[ADC1_CHANNEL_COUNT];
volatile uint16_t adc3_read_buffer[ADC3_CHANNEL_COUNT];

// Currently only internal variable to check how often this happens
static volatile uint32_t adc1_overrun_count = 0;
static volatile uint32_t adc3_overrun_count = 0;

// Declared extern in main.h
uint32_t general_board_status = 0;

/* Handler for CAN messages. */
static void can_callback(const can_msg_t *msg) {
	seen_can_msg = true;
	if (get_board_type_unique_id(msg) == BOARD_TYPE_UNIQUE_ID) {
		return;
	}

	switch (get_message_type(msg)) {
		case MSG_LEDS_ON:
			HAL_GPIO_WritePin(LED_D2_REG, LED_D2_PIN, LED_ON);
			HAL_GPIO_WritePin(LED_D3_REG, LED_D3_PIN, LED_ON);
			break;

		case MSG_LEDS_OFF:
			HAL_GPIO_WritePin(LED_D2_REG, LED_D2_PIN, LED_OFF);
			HAL_GPIO_WritePin(LED_D3_REG, LED_D3_PIN, LED_OFF);
			break;

		case MSG_RESET_CMD:
			bool board_need_reset;
			w_status_t parse_status = check_board_need_reset(msg, &board_need_reset);
			if (parse_status == W_SUCCESS && board_need_reset) {
				HAL_NVIC_SystemReset();
			}
			break;

		default:
			break;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		if (adc1_data_ready) {
			++adc1_overrun_count;
		}
		adc1_data_ready = true;
	} else if (hadc->Instance == ADC3) {
		if (adc3_data_ready) {
			++adc3_overrun_count;
		}
		adc3_data_ready = true;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SDMMC1_SD_Init();
  MX_ADC3_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

	uint32_t last_msg_millis = 0;
	uint32_t last_adc_reading_millis = 0;
	uint32_t last_status_millis = 0;

	uint32_t can_send_failure_count = 0;

	// ADC1: PT3, PT4, PT5, PT6
	// ADC3: PT1, PT2
	analog_sensor_handle_t adc1_sensor_handles[ADC1_CHANNEL_COUNT] = {
		{
			.config =
				{
					.sample_freq_divider = PT3_FREQ_DIVIDER,
					.on_read = sensor_on_read_pt,
					.sensor_id = SENSOR_PT_CHANNEL_3,
					.low_pass_enabled = PT3_LOW_PASS_ENABLED,
					.low_pass_alpha = LOW_PASS_ALPHA(PT3_LOW_PASS_RESPONSE_TIME_ms,
													 ADC_SAMPLE_FREQ_Hz / PT3_FREQ_DIVIDER),
				},
			.freq_div_counter = 1,
		},
		{
			.config =
				{
					.sample_freq_divider = PT4_FREQ_DIVIDER,
					.on_read = sensor_on_read_pt,
					.sensor_id = SENSOR_PT_CHANNEL_4,
					.low_pass_enabled = PT4_LOW_PASS_ENABLED,
					.low_pass_alpha = LOW_PASS_ALPHA(PT4_LOW_PASS_RESPONSE_TIME_ms,
													 ADC_SAMPLE_FREQ_Hz / PT4_FREQ_DIVIDER),
				},
			.freq_div_counter = 1,
		},
		{
			.config =
				{
					.sample_freq_divider = PT5_FREQ_DIVIDER,
					.on_read = sensor_on_read_pt,
					.sensor_id = SENSOR_PT_CHANNEL_5,
					.low_pass_enabled = PT5_LOW_PASS_ENABLED,
					.low_pass_alpha = LOW_PASS_ALPHA(PT5_LOW_PASS_RESPONSE_TIME_ms,
													 ADC_SAMPLE_FREQ_Hz / PT5_FREQ_DIVIDER),
				},
			.freq_div_counter = 1,
		},
		{
			.config =
				{
					.sample_freq_divider = PT6_FREQ_DIVIDER,
					.on_read = sensor_on_read_pt,
					.sensor_id = SENSOR_PT_CHANNEL_6,
					.low_pass_enabled = PT6_LOW_PASS_ENABLED,
					.low_pass_alpha = LOW_PASS_ALPHA(PT6_LOW_PASS_RESPONSE_TIME_ms,
													 ADC_SAMPLE_FREQ_Hz / PT6_FREQ_DIVIDER),
				},
			.freq_div_counter = 1,
		},

	};
	analog_sensor_handle_t adc3_sensor_handles[ADC3_CHANNEL_COUNT] = {
		{
			.config =
				{
					.sample_freq_divider = PT1_FREQ_DIVIDER,
					.on_read = sensor_on_read_pt,
					.sensor_id = SENSOR_PT_CHANNEL_1,
					.low_pass_enabled = PT1_LOW_PASS_ENABLED,
					.low_pass_alpha = LOW_PASS_ALPHA(PT1_LOW_PASS_RESPONSE_TIME_ms,
													 ADC_SAMPLE_FREQ_Hz / PT1_FREQ_DIVIDER),
				},
			.freq_div_counter = 1,
		},
		{
			.config =
				{
					.sample_freq_divider = PT2_FREQ_DIVIDER,
					.on_read = sensor_on_read_pt,
					.sensor_id = SENSOR_PT_CHANNEL_2,
					.low_pass_enabled = PT2_LOW_PASS_ENABLED,
					.low_pass_alpha = LOW_PASS_ALPHA(PT2_LOW_PASS_RESPONSE_TIME_ms,
													 ADC_SAMPLE_FREQ_Hz / PT2_FREQ_DIVIDER),
				},
			.freq_div_counter = 1,
		},
	};

	if (!stm32h7_can_init(&hfdcan1, can_callback)) {
		Error_Handler();
	}

#ifndef SD_DISABLE
	if (sd_fs_init() != W_SUCCESS) {
		Error_Handler();
	}
	sd_log_init();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (seen_can_msg) {
			seen_can_msg = false;
			last_msg_millis = millis();
		}

		if (millis() - last_msg_millis > MAX_BUS_DEAD_TIME_ms) {
			HAL_NVIC_SystemReset();
		}

		if (millis() - last_adc_reading_millis > ADC_SAMPLE_INTERVAL_ms) {
			last_adc_reading_millis = millis();
			if (!adc1_data_ready) {
				HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_read_buffer, ADC1_CHANNEL_COUNT);
			} else {
				++adc1_overrun_count;
			}
			if (!adc3_data_ready) {
				HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_read_buffer, ADC3_CHANNEL_COUNT);
			} else {
				++adc3_overrun_count;
			}
		}

		if (adc1_data_ready) {
			w_status_t handle_read_status =
				handle_adc_scan_ready(adc1_read_buffer, adc1_sensor_handles, ADC1_CHANNEL_COUNT);
			if (handle_read_status != W_SUCCESS) {
				++can_send_failure_count;
			}
			adc1_data_ready = false;
		}

		if (adc3_data_ready) {
			w_status_t handle_read_status =
				handle_adc_scan_ready(adc3_read_buffer, adc3_sensor_handles, ADC3_CHANNEL_COUNT);
			if (handle_read_status != W_SUCCESS) {
				++can_send_failure_count;
			}
			adc3_data_ready = false;
		}

		if (millis() - last_status_millis > STATUS_CHECK_INTERVAL_ms) {
			last_status_millis = millis();
			can_msg_t status_msg;
			build_general_board_status_msg(
				PRIO_MEDIUM, (uint16_t)millis(), general_board_status, &status_msg);
			if (!stm32h7_can_send(&status_msg)) {
				++can_send_failure_count;
			}
			HAL_GPIO_TogglePin(LED_D2_REG, LED_D2_PIN);
		}

#ifndef SD_DISABLE
		sd_log_flush();
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV8;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 96;
  PeriphClkInitStruct.PLL2.PLL2P = 6;
  PeriphClkInitStruct.PLL2.PLL2Q = 32;
  PeriphClkInitStruct.PLL2.PLL2R = 6;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 64;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 32;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */
#ifdef SD_DISABLE
	return;
#endif // SD_DISABLE defined

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 12;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line
	   number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
	   line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
