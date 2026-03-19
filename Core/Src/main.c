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
#include <stdint.h>
#include <stdbool.h>

#include "canlib.h"
#include "low_pass_filter.h"

#include "platform.h"
#include "sensor.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_BUS_DEAD_TIME_ms 1000

// Setting any SAMPLE_INTERVAL_ms to 0 disables sampling for that channel.

// PT gets downsampled for sending
#define PT1_SAMPLE_INTERVAL_ms 50 // 20 Hz
#define PT2_SAMPLE_INTERVAL_ms 50
#define PT3_SAMPLE_INTERVAL_ms 50

// Hall gets read and sampled at same frequency
#define HALL1_SAMPLE_INTERVAL_ms 250 // 4 Hz
#define HALL2_SAMPLE_INTERVAL_ms 250

// Temperature (TC: thermocouple) gets read and sampled at same frequency
#define TC1_SAMPLE_INTERVAL_ms 250 // 4 hz
#define TC2_SAMPLE_INTERVAL_ms 250
#define TC3_SAMPLE_INTERVAL_ms 250

// Sends value once for every PTx_SEND_DOWNSAMPLE_MASK + 1 readings
#define PT1_SEND_DOWNSAMPLE_MASK 0x3 // 1 in 4
#define PT2_SEND_DOWNSAMPLE_MASK 0x3
#define PT3_SEND_DOWNSAMPLE_MASK 0x3

#define PT1_LOW_PASS_RESPONSE_TIME_ms 2500.0
#define PT2_LOW_PASS_RESPONSE_TIME_ms 2500.0
#define PT3_LOW_PASS_RESPONSE_TIME_ms 2500.0


// Schematic unclear but at least on dev board D2 is white and D6 is blue
#define LED_D2_REG GPIOD
#define LED_D2_PIN GPIO_PIN_10
#define LED_D6_REG GPIOD
#define LED_D6_PIN GPIO_PIN_9

#define LED_ON GPIO_PIN_SET
#define LED_OFF GPIO_PIN_RESET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
static void can_callback(const can_msg_t * msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile bool seen_can_msg = false;

/* Handler for CAN messages. */
static void can_callback(const can_msg_t * msg) {
  seen_can_msg = true;
  if (get_board_type_unique_id(msg) == BOARD_TYPE_UNIQUE_ID) {
    return;
  }

  switch (get_message_type(msg)) {
     case MSG_LEDS_ON:
      HAL_GPIO_WritePin(LED_D2_REG, LED_D2_PIN, LED_ON);
      HAL_GPIO_WritePin(LED_D6_REG, LED_D6_PIN, LED_ON);
      break;

    case MSG_LEDS_OFF:
      HAL_GPIO_WritePin(LED_D2_REG, LED_D2_PIN, LED_OFF);
      HAL_GPIO_WritePin(LED_D6_REG, LED_D6_PIN, LED_OFF);
      break;

    case MSG_RESET_CMD:
      if (check_board_need_reset(msg)) {
        HAL_NVIC_SystemReset();
      }
      break;

    default:
      break;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  // Used to configure all

  // Stagger initial millis to lower peak CAN bus message rate
  uint32_t last_msg_millis = 0;

  uint16_t last_pt1_reading_millis = 1;
  uint16_t last_pt2_reading_millis = 2;
  uint16_t last_pt3_reading_millis = 3;
  uint16_t last_hall1_reading_millis = 4;
  uint16_t last_hall2_reading_millis = 5;
  uint16_t last_tc1_reading_millis = 6;
  uint16_t last_tc2_reading_millis = 7;
  uint16_t last_tc3_reading_millis = 8;

  // Used to send value over CAN once every PTx_SEND_DOWNSAMPLE_MASK+1 readings
  uint8_t pt1_reading_count = 0;
  uint8_t pt2_reading_count = 0;
  uint8_t pt3_reading_count = 0;

  double pt1_low_pass_state = 0;
  double pt2_low_pass_state = 0;
  double pt3_low_pass_state = 0;

  double pt1_low_pass_alpha;
  double pt2_low_pass_alpha;
  double pt3_low_pass_alpha;
  low_pass_filter_init(&pt1_low_pass_alpha, PT1_LOW_PASS_RESPONSE_TIME_ms);
  low_pass_filter_init(&pt2_low_pass_alpha, PT2_LOW_PASS_RESPONSE_TIME_ms);
  low_pass_filter_init(&pt3_low_pass_alpha, PT3_LOW_PASS_RESPONSE_TIME_ms);

  stm32h7_can_init(&hfdcan1, can_callback);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (seen_can_msg) {
      seen_can_msg = false;
      last_msg_millis = millis();
    }

    if (millis() - last_msg_millis > MAX_BUS_DEAD_TIME_ms) {
      HAL_NVIC_SystemReset();
    }

    /*
     * Read from ADC1. Number of conversions should be configured to 1.
     */

    // Pressure transducers

#if PT1_SAMPLE_INTERVAL_ms
    // PT_1: ADC1 Channel 4, pin PC4
    if (millis() - last_pt1_reading_millis > PT1_SAMPLE_INTERVAL_ms) {
      last_pt1_reading_millis = millis();
      uint32_t pt1_raw;
      bool read_success = read_from_adc_channel(&hadc1, ADC_CHANNEL_4, &pt1_raw);
      if (read_success) {
        update_low_pass(pt1_low_pass_alpha, pt_adc_raw_to_psi(pt1_raw), &pt1_low_pass_state);
        if ((pt1_reading_count & PT1_SEND_DOWNSAMPLE_MASK) == 0) {
          can_msg_t sensor_msg;
          build_analog_data_16bit_msg(
            PRIO_LOW,
            millis(),
            SENSOR_PT_CHANNEL_1,
            pt1_low_pass_state,
            &sensor_msg
          );
          if (stm32h7_can_send_rdy()) {
            stm32h7_can_send(&sensor_msg);
          }
        }
        ++pt1_reading_count;
      }
    }
#endif

#if PT2_SAMPLE_INTERVAL_ms
    // PT_2: ADC1 Channel 5, pin PB1
    if (millis() - last_pt2_reading_millis > PT2_SAMPLE_INTERVAL_ms) {
      last_pt2_reading_millis = millis();
      uint32_t pt2_raw;
      bool read_success = read_from_adc_channel(&hadc1, ADC_CHANNEL_5, &pt2_raw);
      if (read_success) {
        update_low_pass(pt2_low_pass_alpha, pt_adc_raw_to_psi(pt2_raw), &pt2_low_pass_state);
        if ((pt2_reading_count & PT2_SEND_DOWNSAMPLE_MASK) == 0) {
          can_msg_t sensor_msg;
          build_analog_data_16bit_msg(
            PRIO_LOW,
            millis(),
            SENSOR_PT_CHANNEL_2,
            pt2_low_pass_state,
            &sensor_msg
          );
          if (stm32h7_can_send_rdy()) {
            stm32h7_can_send(&sensor_msg);
          }
        }
        ++pt2_reading_count;
      }
    }
#endif

#if PT3_SAMPLE_INTERVAL_ms
    // PT_3: ADC1 Channel 9, pin PB0
    if (millis() - last_pt3_reading_millis > PT3_SAMPLE_INTERVAL_ms) {
      last_pt3_reading_millis = millis();
      uint32_t pt3_raw;
      bool read_success = read_from_adc_channel(&hadc1, ADC_CHANNEL_9, &pt3_raw);
      if (read_success) {
        update_low_pass(pt3_low_pass_alpha, pt_adc_raw_to_psi(pt3_raw), &pt3_low_pass_state);
        if ((pt3_reading_count & PT3_SEND_DOWNSAMPLE_MASK) == 0) {
          can_msg_t sensor_msg;
          build_analog_data_16bit_msg(
            PRIO_LOW,
            millis(),
            SENSOR_PT_CHANNEL_3,
            pt3_low_pass_state,
            &sensor_msg
          );
          if (stm32h7_can_send_rdy()) {
            stm32h7_can_send(&sensor_msg);
          }
        }
        ++pt3_reading_count;
      }
    }
#endif

#if HALL1_SAMPLE_INTERVAL_ms
    // NOTE: Placeholder, possibly temporarily jumpered (will be changed on revised board)
    // HALL_1: ADC1 Channel 10, pin PC0
    if (millis() - last_hall1_reading_millis > HALL1_SAMPLE_INTERVAL_ms) {
      last_hall1_reading_millis = millis();
      uint32_t hall1_raw;
      bool read_success = read_from_adc_channel(&hadc1, ADC_CHANNEL_10, &hall1_raw);
      if (read_success) {
        can_msg_t sensor_msg;
        build_analog_data_16bit_msg(
          PRIO_LOW,
          millis(),
          SENSOR_HALL_CHANNEL_1,
          adc_raw_to_mv(hall1_raw),
          &sensor_msg
        );
        if (stm32h7_can_send_rdy()) {
          stm32h7_can_send(&sensor_msg);
        }
      }
    }
#endif

#if HALL2_SAMPLE_INTERVAL_ms
    // NOTE: Placeholder, possibly temporarily jumpered (will be changed on revised board)
    // HALL_2: ADC1 Channel 11, pin PC1
    if (millis() - last_hall2_reading_millis > HALL2_SAMPLE_INTERVAL_ms) {
      last_hall2_reading_millis = millis();
      uint32_t hall2_raw;
      bool read_success = read_from_adc_channel(&hadc1, ADC_CHANNEL_11, &hall2_raw);
      if (read_success) {
        can_msg_t sensor_msg;
        build_analog_data_16bit_msg(
          PRIO_LOW,
          millis(),
          SENSOR_HALL_CHANNEL_2,
          adc_raw_to_mv(hall2_raw),
          &sensor_msg
        );
        if (stm32h7_can_send_rdy()) {
          stm32h7_can_send(&sensor_msg);
        }
      }
    }
#endif

#if TC1_SAMPLE_INTERVAL_ms
    // TODO: Read from MAX6675 using SPI
#endif

#if TC2_SAMPLE_INTERVAL_ms
    // NOTE: Placeholder, possibly temporarily jumpered (will be changed on revised board)
    // TC2: ADC1 Channel 16 (DIFFERENTIAL), pins PA0 (INP/TC2+) and PA1 (INN/TC2-)
    if (millis() - last_tc2_reading_millis > TC2_SAMPLE_INTERVAL_ms) {
      last_tc2_reading_millis = millis();
      uint32_t tc2_raw;
      bool read_success = read_from_adc_channel(&hadc1, ADC_CHANNEL_16, &tc2_raw);
      if (read_success) {
        can_msg_t sensor_msg;
        build_analog_data_16bit_msg(
          PRIO_LOW,
          millis(),
          SENSOR_INJECTOR_BOARD_TEMP_2,
          // Note this is differential reading so the sent value is V_diff + ADC_RESOLUTION/2.
          adc_raw_to_mv(tc2_raw),
          &sensor_msg
        );
        if (stm32h7_can_send_rdy()) {
          stm32h7_can_send(&sensor_msg);
        }
      }
    }
#endif

#if TC3_SAMPLE_INTERVAL_ms
    // NOTE: Placeholder, possibly temporarily jumpered (will be changed on revised board)
    // TC3: ADC1 Channel 14, pin PA2
    if (millis() - last_tc3_reading_millis > TC3_SAMPLE_INTERVAL_ms) {
      last_tc3_reading_millis = millis();
      uint32_t tc3_raw;
      bool read_success = read_from_adc_channel(&hadc1, ADC_CHANNEL_14, &tc3_raw);
      if (read_success) {
        can_msg_t sensor_msg;
        build_analog_data_16bit_msg(
          PRIO_LOW,
          millis(),
          SENSOR_INJECTOR_BOARD_TEMP_3,
          adc_raw_to_mv(tc3_raw),
          &sensor_msg
        );
        if (stm32h7_can_send_rdy()) {
          stm32h7_can_send(&sensor_msg);
        }
      }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 13;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
