/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include <string>
#include <sstream>

#include "SHT30.hpp"
#include "selector.hpp"
#include "logger.hpp"
#include "ESP32.hpp"
#include "data.hpp"
#include "SMU.hpp"
#include "real_time_clock.hpp"
#include "MPL3115A2.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct BufferRange {
	size_t start;
	size_t end;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define SHT30_D
//#define SELECTOR_D
#define ESP32_D
#define SHT30_D
#define SMU_D
#define PRESSURE_D

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* RTOS */
const osThreadAttr_t httpTask_attributes = {
    .name = "httpTask",
    .stack_size = 128 * 64,
    .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t selectorTaskHandle;
osThreadId_t httpSendTaskHandle;
osThreadId_t processHandle;
osThreadId_t smuProcessHandle;

osMessageQueueId_t esp_msg_rx_queue;
osMessageQueueId_t smu_data_rx_queue;


osSemaphoreId_t esp_data_ready_sem;
osSemaphoreId_t esp_messages_sem;

osSemaphoreId_t smu_done_sem;

osMutexId_t data_packet_mutex;

/* CONFIG */
// ID -> decoder #
std::unordered_map<uint8_t, uint8_t> panels = {
		  {0, 0},
		  {1, 1},
		  {2, 2},
		  {3, 3},
		  {4, 4}
};

/* PERIPHERALS */
Logger logger(huart1, LogLevel::Debug);
SHT30 sht(hi2c1);
MPL3115A2 pressure_sensor(hi2c2);
Selector selector(panels, {GPIOD, GPIO_PIN_12}, {GPIOD, GPIO_PIN_13}, {GPIOD, GPIO_PIN_14}, 7);
ESP32 esp(huart2, esp_messages_sem, esp_data_ready_sem);
SMU smu(huart3);
RealTimeClock rtc(hrtc);

/* BUFFERS */
DataPacket data_packet;
uint8_t esp_buf;
char msg[100];
float temp = 0.0f;
float rh = 0.0f;
BufferRange esp_buffer_range = {0,0};

size_t esp_usart_pos = 0;
uint8_t esp_usart_rx_buffer[ESP_MAX_RESP_LENGTH];

size_t smu_usart_pos = 0;
uint8_t smu_usart_rx_buffer[SMU_BUFFER_LENGTH];
uint8_t curr_cell_id = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void ScheduledUpdateUploadTask(void* argument);
void RequestedUpdateUploadTask(void* argument);

void EspUsartRxTask(void* arg);
void SmuUsartRxTask(void* arg);

void update_data();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  //MX_SDIO_SD_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  Logger::registerInstance(&logger);
  logger.debug("Init");


#ifdef ESP32_D
  HAL_Delay(500); // allow ESP to finish any commands from before reset
  if (!esp.init()) {
	  logger.error("ESP32 init FAIL!");
  } else {
	  logger.info("ESP32 init SUCCESS");
  }

  HAL_Delay(500);
//  HAL_UART_Receive_IT(&huart2, &esp_buf, 1);

#endif


#ifdef SHT30_D
  HAL_UART_Transmit(&huart1, (uint8_t*) msg, sizeof(msg), 100);
  if (!sht.init()) {
	  logger.error("SHT30 init FAIL");
	  return 0;
  }

  logger.info("SHT30 init OK");
#endif

#ifdef PRESSURE_D
  if (pressure_sensor.init() != HAL_OK) {
	  logger.error("Pressure sensor init FAIL");
	  return 0;
  } else {
	  logger.info("Pressure sensor init OK");
  }
#endif

#ifdef SELECTOR_D
  selector.deselect_all();
#endif

#ifdef SMU_D
  smu.config_voltage_sweep();
#endif

  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  const osMutexAttr_t DataPacket_Mutex_attr = {
    "dataPacketMutex",                          // human readable mutex name
    osMutexPrioInherit,    // attr_bits
    NULL,                                     // memory for control block
    0U                                        // size for control block
  };

  data_packet_mutex = osMutexNew(&DataPacket_Mutex_attr);

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  esp_messages_sem = osSemaphoreNew(10U, 0U, NULL);
  esp_data_ready_sem = osSemaphoreNew(1U, 0U, NULL);
  smu_done_sem = osSemaphoreNew(1U, 0U, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  esp_msg_rx_queue = osMessageQueueNew(10U, sizeof(BufferRange), NULL);
  smu_data_rx_queue = osMessageQueueNew(100U, sizeof(BufferRange), NULL);

  if (esp_msg_rx_queue == NULL) {
       logger.error("Queue creation failed"); // Message Queue object not created, handle failure
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
#ifdef SELECTOR_D
  selectorTaskHandle = osThreadNew(SelectorCycleTask, NULL, &defaultTask_attributes);
#endif

  const osThreadAttr_t processTask_attributes = {
    .name = "processTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityHigh,
  };

  processHandle = osThreadNew(EspUsartRxTask, NULL, &processTask_attributes);

  const osThreadAttr_t smu_processTask_attributes = {
      .name = "processTask",
      .stack_size = 128 * 16,
      .priority = (osPriority_t) osPriorityHigh,
  };

  smuProcessHandle = osThreadNew(SmuUsartRxTask, NULL, &smu_processTask_attributes);


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Panel0_Pin|Panel1_Pin|Panel2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Panel0_Pin Panel1_Pin Panel2_Pin */
  GPIO_InitStruct.Pin = Panel0_Pin|Panel1_Pin|Panel2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ScheduledUpdateUploadTask(void* argument) {
	uint32_t tick = osKernelGetTickCount();
	std::string esp_resp;

	for(;;) {
//		tick += 900000U; // 15 minutes
		tick += 300000U; // 5 minutes
//		tick += 60000U; // 1 minute

		update_data();

		esp.flush();

//		esp.send_cmd("AT+SYSRAM?");
//		osMessageQueueGet(esp_messages_queue, &d, NULL, osWaitForever);
//		esp_resp = esp.consume_message();
//		logger.info("ESP RAM: " + esp_resp);

		data_packet.serialize_json();
		logger.debug("JSON length: " + std::to_string(data_packet.serialized_json.length()));

		esp.send_data_packet_start(data_packet.serialized_json.length());

		osSemaphoreAcquire(esp_messages_sem, osWaitForever);
		esp_resp = esp.consume_message();

		if (esp_resp.find(ESP_OK) == std::string::npos) {
			logger.error("Unexpected HTTP start response: " + esp_resp);
		}

		osSemaphoreAcquire(esp_data_ready_sem, osWaitForever);
		esp.send_cmd(data_packet.serialized_json, false);
		osSemaphoreRelease(esp_data_ready_sem);

		data_packet.serialized_json = ""; // free up memory

		osSemaphoreAcquire(esp_messages_sem, 5000);
		esp_resp = esp.consume_message();
		logger.debug(esp_resp);

		osDelayUntil(tick);
	}
}

void EspUsartRxTask(void* arg) {
	BufferRange buffer_range;

	for(;;) {
		osMessageQueueGet(esp_msg_rx_queue, &buffer_range, NULL, osWaitForever);

		if (buffer_range.start <= buffer_range.end) {
			esp.push_message(std::string((char*) &esp_usart_rx_buffer[buffer_range.start],
					buffer_range.end + 1 - buffer_range.start));
		} else {
			std::string res((char*) &esp_usart_rx_buffer[buffer_range.start],
					ESP_MAX_RESP_LENGTH - buffer_range.start);

			res += std::string((char*) &esp_usart_rx_buffer[0], buffer_range.end+1);
			esp.push_message(res);
		}
	}
}

void SmuUsartRxTask(void* arg) {
	BufferRange buffer_range;

	for(;;) {
		osMessageQueueGet(smu_data_rx_queue, &buffer_range, NULL, osWaitForever);
		std::string data_str;

		if (buffer_range.start <= buffer_range.end) {
			data_str = std::string((char*)&smu_usart_rx_buffer[buffer_range.start],
				buffer_range.end + 1 - buffer_range.start);
		} else {
			// the buffer wrapped around
			data_str = std::string((char*) &smu_usart_rx_buffer[buffer_range.start],
								SMU_BUFFER_LENGTH - buffer_range.start);

			data_str += std::string((char*) &smu_usart_rx_buffer[0], buffer_range.end+1);
		}

		logger.debug(data_str);

		// find the position of the comma
		size_t comma_pos = data_str.find(',');

		if (comma_pos != std::string::npos) {
			// extract the current and voltage substrings
			std::string voltage = data_str.substr(0, comma_pos);
			std::string current = data_str.substr(comma_pos + 1);

			if (voltage[0] == '+') {
				voltage.erase(0,1);
			}

			if (current[0] == '+') {
				current.erase(0,1);
			}

			// create a CurrentVoltagePair using the current and voltage values
			CurrentVoltagePair pair {current, voltage};

			// push the CurrentVoltagePair to your data structure
			data_packet.iv_curves[curr_cell_id].push_back(pair);
			if (data_packet.iv_curves[curr_cell_id].size() == 141) {
				osSemaphoreRelease(smu_done_sem);
			}
		} else {
			logger.error("Invalid SMU data sent to rx thread: " + data_str);
		}
	}
}

void update_data() {
	// TODO: fix RTC so we don't have to re-sync every time
	esp.flush();
	esp.send_cmd("AT+HTTPCLIENT=2,0,\"http://18.220.103.162:5050/api/v1/sensorCellData/getCurrentTime\",,,1");
	osSemaphoreAcquire(esp_messages_sem, 10000U);
	std::string time_resp = esp.consume_message();

	if (time_resp == "ERROR" || !rtc.parse_and_sync(time_resp)) {
	  logger.warn("Unable to sync clock, using battery backup...");
	} else {
	  logger.info("RTC synchronized to " + std::to_string(rtc.get_current_timestamp()));
	}

	osMutexAcquire(data_packet_mutex, osWaitForever);
	data_packet.clear();

	data_packet.timestamp = rtc.get_current_timestamp();
	logger.info("Capture timestamp: " + std::to_string(data_packet.timestamp));

#ifdef SHT30_D
	sht.read_temp_humidity(data_packet.ambient_temp, data_packet.humidity);
	logger.info("Temp: " + std::to_string(data_packet.ambient_temp) + " Humid: " + std::to_string(data_packet.humidity));
#else
	data_packet.ambient_temp = 70.23;
	data_packet.humidity = 53.75;
#endif

#ifdef PRESSURE_D
	data_packet.barometric_pressure = pressure_sensor.read_baromateric_pressure();
	logger.info("Pressure: " + std::to_string(data_packet.barometric_pressure));
#else
	data_packet.barometric_pressure = 1000.53;
#endif


#ifdef SMU_D
#ifdef SELECTOR_D // smu, selector
	for (auto& el : panels) {
		selector.select(el.first);
		curr_cell_id = el.first; // panel ID
		smu.run_voltage_sweep();
		osSemaphoreAcquire(smu_done_sem, osWaitForever);
	}
#else // smu, no selector (use connected panel for all panel IDs)
	for (auto& el : panels) {
		curr_cell_id = el.first; // panel ID
		smu.run_voltage_sweep();
		osSemaphoreAcquire(smu_done_sem, osWaitForever);
	}
#endif
#else // no SMU, even if we have selector we have to generate fake data
	for (auto& el : panels) {
		data_packet.iv_curves[el.first] = std::vector<CurrentVoltagePair>();
		for (size_t i = 0; i < 100; i++) {
			data_packet.iv_curves[el.first].push_back({std::to_string(i*0.34), std::to_string(i*3.75)});
		}
	}
#endif

#ifdef THERMISTORS_D
	// TODO: read from ADC
#else // no thermistors
	for (auto& el : panels) {
		data_packet.cell_temperatures[el.first] = 30.2;
	}

#endif

	osMutexRelease(data_packet_mutex);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &esp.get_uart_handle()) {
		// short circuit to avoid unnecessary strncmp() calls
		if (esp_usart_rx_buffer[esp_usart_pos] == '\n') {
			if (esp_usart_pos >= 3 &&
					!strncmp((char*) &esp_usart_rx_buffer[esp_usart_pos-3], ESP_OK, 4)) {
				esp_buffer_range.end = esp_usart_pos;
				osMessageQueuePut(esp_msg_rx_queue, &esp_buffer_range, 0U, 0U);

				if (esp_usart_pos == ESP_MAX_RESP_LENGTH-1) {
					esp_buffer_range.start = 0;
				} else {
					esp_buffer_range.start = esp_usart_pos+1;
				}
			} else {
				// circular buffer look back
				size_t idx = esp_usart_pos;
				bool match = true;
				for (int i = 3; i >= 0; i--) {// ESP_OK length
					if (ESP_OK[i] != (char) esp_usart_rx_buffer[idx]) {
						match = false;
						break;
					}

					if (idx == 0) {
						idx = ESP_MAX_RESP_LENGTH-1;
					} else {
						--idx;
					}
				}

				if (match) {
					esp_buffer_range.end = esp_usart_pos;
					osMessageQueuePut(esp_msg_rx_queue, &esp_buffer_range, 0U, 0U);
					esp_buffer_range.start = esp_usart_pos+1;
				}
			}
		} else if (esp_usart_rx_buffer[esp_usart_pos] == '>') {
			osSemaphoreRelease(esp_data_ready_sem);
		}

		if (esp_usart_pos == ESP_MAX_RESP_LENGTH-1) {
			esp_usart_pos = 0;
		} else {
			++esp_usart_pos;
		}

		HAL_UART_Receive_IT(&esp.get_uart_handle(), &esp_usart_rx_buffer[esp_usart_pos], 1);
	} else if (huart == &smu.get_uart_handle()) {
		static BufferRange smu_buffer_range = {0, 0};
		static bool isValVoltage = true;

		// TODO: add some error detection for this (identifying bad packets)
		// this is all assuming data is a constant stream of "current,voltage,current,voltage," etc.
		if (isValVoltage && smu_usart_rx_buffer[smu_usart_pos] == ',') {
			// we got the first value (current), now get next value (voltage)
			isValVoltage = false;
		} else if (!isValVoltage && (smu_usart_rx_buffer[smu_usart_pos] == ',' || smu_usart_rx_buffer[smu_usart_pos] == '\r')) {
			// when a comma is found again, we know this is the end of the voltage reading
			smu_buffer_range.end = smu_usart_pos-1; // don't include comma

			// put "current,voltage" pair (the start and end indices) in the queue
			osMessageQueuePut(smu_data_rx_queue, &smu_buffer_range, 0U, 0U);

			// the next current val will start one position after this voltage value
			isValVoltage = true;
			smu_buffer_range.start = smu_usart_pos + 1;
		}

		if (smu_usart_pos == SMU_BUFFER_LENGTH - 1) {
			smu_usart_pos = 0;
		} else {
			++smu_usart_pos;
		}

		HAL_UART_Receive_IT(&smu.get_uart_handle(), &smu_usart_rx_buffer[smu_usart_pos], 1);
	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  for (;;) {
	  HAL_UART_Receive_IT(&huart2, &esp_usart_rx_buffer[esp_usart_pos], 1);
	  HAL_UART_Receive_IT(&huart3, &smu_usart_rx_buffer[smu_usart_pos], 1);

	  // sync RTC
	  esp.send_cmd("AT+HTTPCLIENT=2,0,\"http://18.220.103.162:5050/api/v1/sensorCellData/getCurrentTime\",,,1");
	  osSemaphoreAcquire(esp_messages_sem, osWaitForever);
	  std::string time_resp = esp.consume_message();

	  if (!rtc.parse_and_sync(time_resp)) {
		  logger.warn("Unable to sync clock, using battery backup...");
	  } else {
		  logger.info("RTC synchronized to " + std::to_string(rtc.get_current_timestamp()));
	  }

	  httpSendTaskHandle = osThreadNew(ScheduledUpdateUploadTask, NULL, &httpTask_attributes);
	  osThreadSuspend(defaultTaskHandle);
  }
  /* USER CODE END 5 */
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
