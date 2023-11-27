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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include <string>

#include "SHT30.hpp"
#include "selector.hpp"
#include "logger.hpp"
#include "ESP32.hpp"
#include "data.hpp"
#include "SMU.hpp"
#include "VC0706.hpp"
#include "real_time_clock.hpp"
#include "MPL3115A2.hpp"
#include "thermistor_array.hpp"
#include <optional>
#include <cmath>

extern "C" {
	#include "ff_gen_drv.h"
}
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
//#define SHT30_D
#define SMU_D
//#define PRESSURE_D
//#define THERMISTORS_D
// #define CAMERA_D

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

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
osThreadId_t scheduledSendTaskHandle;
osThreadId_t requestedSendTaskHandle;
osThreadId_t processHandle;
osThreadId_t smuProcessHandle;

osMessageQueueId_t esp_msg_rx_queue;
osMessageQueueId_t smu_data_rx_queue;


osSemaphoreId_t esp_data_ready_sem;
osSemaphoreId_t esp_messages_sem;
osSemaphoreId_t esp_poll_cmd_sem;

osSemaphoreId_t camera_rx_complete;

osSemaphoreId_t smu_done_sem;
osSemaphoreId_t adc_done_sem;

osMutexId_t data_packet_mutex;

void * operator new( size_t size )
{
	void* ret = pvPortMalloc( size );
	if (ret == NULL) {
		HeapStats_t stats;
		vPortGetHeapStats(&stats);
		Error_Handler();
	}

    return ret;
}

void * operator new[]( size_t size )
{
	void* ret = pvPortMalloc( size );
	if (ret == NULL) {
		Logger::getInstance()->debug(std::to_string(xPortGetFreeHeapSize()));
		Error_Handler();
	}

	return ret;
}

void operator delete( void * ptr )
{
    vPortFree ( ptr );
}

void operator delete[]( void * ptr )
{
    vPortFree ( ptr );
}

/* CONFIG */
// ID -> decoder #
std::unordered_map<uint8_t, uint8_t> panels = {
		  {0, 0U},
		  {1, 1U},
		  {2, 2U},
		  {3, 3U},
		  {4, 4U}
};

/* PERIPHERALS */
Logger logger(huart3, LogLevel::Debug); // TODO: change uart
SHT30 sht(hi2c1);
MPL3115A2 pressure_sensor(hi2c2);
Selector selector(panels, 7U, {GPIOE, GPIO_PIN_8}, {GPIOE, GPIO_PIN_9}, {GPIOE, GPIO_PIN_10});
ESP32 esp(huart2, esp_messages_sem, esp_data_ready_sem);
SMU smu(huart6);
ThermistorArray thermistor_array(&hadc1, 1);
VC0706 camera(&huart4, &camera_rx_complete);

/* CONSTANTS */
static constexpr uint32_t SCHEDULED_UPLOAD_PERIOD_MS = 15 * 60 * 1000;

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

FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_UART4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void ScheduledUpdateUploadTask(void* argument);
void RequestedUpdateUploadTask(void* argument);

void EspUsartRxTask(void* arg);
void SmuUsartRxTask(void* arg);

void update_upload();
void update_photo();
void update_data();
void sd_test();


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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


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
  esp_poll_cmd_sem = osSemaphoreNew(1U, 0U, NULL);
  smu_done_sem = osSemaphoreNew(1U, 0U, NULL);
  adc_done_sem = osSemaphoreNew(1U, 0U, NULL);
  camera_rx_complete = osSemaphoreNew(1U, 0U, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  esp_msg_rx_queue = osMessageQueueNew(10U, sizeof(BufferRange), NULL);
  smu_data_rx_queue = osMessageQueueNew(50U, sizeof(BufferRange), NULL);

  if (esp_msg_rx_queue == NULL) {
       logger.error("Queue creation failed"); // Message Queue object not created, handle failure
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  hsd.Init.ClockDiv = 127;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 38400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SEL0_Pin|SEL1_Pin|SEL2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Panel0_Pin|Panel1_Pin|Panel2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEL0_Pin SEL1_Pin SEL2_Pin */
  GPIO_InitStruct.Pin = SEL0_Pin|SEL1_Pin|SEL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Panel0_Pin Panel1_Pin Panel2_Pin */
  GPIO_InitStruct.Pin = Panel0_Pin|Panel1_Pin|Panel2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ScheduledUpdateUploadTask(void* argument) {
	uint32_t tick = osKernelGetTickCount();

	for(;;) {
		tick += SCHEDULED_UPLOAD_PERIOD_MS;
        update_upload();
		osDelayUntil(tick);
	}
}

void RequestedUpdateUploadTask(void* argumnent) {
    for(;;) {
        osSemaphoreAcquire(esp_poll_cmd_sem, osWaitForever);
        update_upload();
    }
}

void EspUsartRxTask(void* arg) {
	BufferRange buffer_range{0,0};

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
	BufferRange buffer_range{0,0};

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
			data_packet.iv_curves[curr_cell_id].push(pair);
			if (data_packet.iv_curves[curr_cell_id].size() == 141) {
				osSemaphoreRelease(smu_done_sem);
			}
		} else {
			logger.error("Invalid SMU data sent to rx thread: " + data_str);
		}
	}
}

void update_upload() {
	osMutexAcquire(data_packet_mutex, osWaitForever);
    std::string esp_resp;
    esp.disconnect_control_server();
    update_data();

    esp.flush();

    data_packet.serialize_json();
    logger.debug("JSON length: " + std::to_string(data_packet.json.size()));

    logger.debug(std::to_string(xPortGetFreeHeapSize()));

    esp.send_data_packet_start(data_packet.json.size());

    auto os_res = osSemaphoreAcquire(esp_messages_sem, 5000U);
    if (os_res != osOK) {
        Error_Handler();
    }
    esp_resp = esp.consume_message();

    if (esp_resp.find(ESP_OK) == std::string::npos) {
        logger.error("Unexpected HTTP start response: " + esp_resp);
        Error_Handler();
    }

    os_res = osSemaphoreAcquire(esp_data_ready_sem, 10000U);
    if (os_res != osOK) {
        Error_Handler();
    }

    while (!data_packet.json.chunks().empty()) {
        esp.send_raw(std::move(data_packet.json.chunks().front()));
        data_packet.json.chunks().pop_front();
    }

    osSemaphoreRelease(esp_data_ready_sem);
    data_packet.json = JsonBuilder();

    os_res = osSemaphoreAcquire(esp_messages_sem, 5000);
    if (os_res != osOK) {
        Error_Handler();
    }
    esp_resp = esp.consume_message();
    logger.debug(esp_resp);

#ifdef CAMERA_D
    update_photo();
#endif
    if (!esp.connect_to_control_server()) {
		Error_Handler();
	}

    osMutexRelease(data_packet_mutex);
}

void update_photo() {
	if (!camera.begin()) {
		Error_Handler();
	}

	if (!camera.set_image_size(VC0706_640x480)) {
		Error_Handler();
	}

	HAL_Delay(500);

    if (!camera.take_picture()) {
        Error_Handler();
    }


    uint32_t jpg_size = camera.frameLength();

    esp.flush();
    esp.send_data_packet_start(static_cast<size_t>(jpg_size),
                               "https://api.umich-ost-pv-dam.org:5050/api/v1/sensorCellData/uploadPhoto",
                               "image/jpeg",
							   data_packet.timestamp);

    osSemaphoreAcquire(esp_messages_sem, 10000U);
    std::string esp_resp = esp.consume_message();
    if (esp_resp.find(ESP_OK) == std::string::npos) {
    	Error_Handler();
    }

    osSemaphoreAcquire(esp_data_ready_sem, osWaitForever);

    const int num_chunks = std::ceil(static_cast<double>(jpg_size) / ESP_PHOTO_CHUNK_LENGTH);
    int jpg_remaining = jpg_size;
    uint32_t actually_read = 0;
    uint32_t bytesToSend = ESP_PHOTO_CHUNK_LENGTH;

    for (int chunk = 0; chunk < num_chunks; ++chunk) {
        std::array<uint8_t, ESP_PHOTO_CHUNK_LENGTH>  buffer;
        uint32_t chunk_size = ESP_PHOTO_CHUNK_LENGTH;
        uint32_t buffer_pos = 0;
        while (chunk_size > 0) {
        	if (jpg_remaining <= 0) {
        		break;
        	}
            // read 32 or 64 bytes at a time
            size_t bytesToRead = std::min((uint32_t) 32, (uint32_t)jpg_remaining);

            // copy camera data into buffer
            camera.read_picture(bytesToRead, buffer, buffer_pos);

            // update counters
            buffer_pos += bytesToRead;
            chunk_size -= bytesToRead;
            jpg_remaining -= bytesToRead;

            HAL_Delay(1);
        }

        // send a different number of bytes on the last chunk
        if (chunk == num_chunks - 1) {
        	bytesToSend = jpg_size % ESP_PHOTO_CHUNK_LENGTH;
        }
        buffer_pos = 0;
        esp.send_raw(buffer, bytesToSend);
        actually_read += bytesToSend;
    }

    osSemaphoreAcquire(esp_messages_sem, 10000U);
    esp_resp = esp.consume_message();
    esp_resp;
}

void update_data() {
	logger.debug(std::to_string(xPortGetFreeHeapSize()));

	data_packet.clear();

	esp.flush();
	esp.send_cmd("AT+SYSTIMESTAMP?");
	osSemaphoreAcquire(esp_messages_sem, 10000U);
	std::string time_resp = esp.consume_message();
    std::optional<time_t> timestamp = get_timestamp_from_api(time_resp);

    if (timestamp) {
        data_packet.timestamp = timestamp.value();
    } else {
    	Error_Handler();
        logger.warn("Unable to get timestamp from ESP RTC");
        data_packet.timestamp = 0; // what's a good fallback?
    }

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

#ifdef THERMISTORS_D
    thermistor_array.update();
    osSemaphoreAcquire(adc_done_sem, osWaitForever);
    data_packet.cell_temperatures[0] = thermistor_array.get_temperature_at(0);
    logger.debug("Thermistor 0: " + std::to_string(data_packet.cell_temperatures[0]));

    for (auto& el : panels) {
        if (el.first == 0) continue;
        data_packet.cell_temperatures[el.first] = 30.2;
    }


#else // no thermistors
    for (auto& el : panels) {
        data_packet.cell_temperatures[el.first] = 30.2;
    }
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
		data_packet.iv_curves[el.first].reserve(141);
	}

	for (auto& el : panels) {
		curr_cell_id = el.first; // panel ID
		smu.run_voltage_sweep();
		osSemaphoreAcquire(smu_done_sem, osWaitForever);
	}
#endif
#else // no SMU, even if we have selector we have to generate fake data
#ifdef SELECTOR_D
		selector.deselect_all();
#endif
	for (auto& el : panels) {
#ifdef SELECTOR_D
		selector.select(el.first);
#endif
		data_packet.iv_curves[el.first] = std::queue<CurrentVoltagePair>();
		for (size_t i = 0; i < 141; i++) {
			data_packet.iv_curves[el.first].push({"-6.000000E+00", "-3.000000E+00"});
		}
		HAL_Delay(1000);
	}

#ifdef SELECTOR_D
		selector.deselect_all();
#endif
#endif

	logger.debug(std::to_string(xPortGetFreeHeapSize()));
}

void sd_test() {
	FRESULT res;                                          /* FatFs function common result code */
	  uint32_t byteswritten, bytesread;                     /* File write/read counts */
	  uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
	  uint8_t rtext[100];                                   /* File read buffer */

	  /*##-1- Link the micro SD disk I/O driver ##################################*/

	    /*##-2- Register the file system object to the FatFs module ##############*/
	    if(res = f_mount(&SDFatFs, (TCHAR const*)SDPath, 1); res != FR_OK)
	    {
	      /* FatFs Initialization Error */
	      Error_Handler();
	    }
	    else
	    {
	      /*##-3- Create a FAT file system (format) on the logical drive #########*/
	      /* WARNING: Formatting the uSD card will delete all content on the device */

//	      if(res = f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, buffer, sizeof(buffer)); res != FR_OK)
//	      {
//	        /* FatFs Format Error */
//	        Error_Handler();
//	      }
//	      else
//	      {
	        /*##-4- Create and Open a new text file object with write access #####*/
	        if(res = f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE); res != FR_OK)
	        {
	          /* 'STM32.TXT' file Open for write Error */
	          Error_Handler();
	        }
	        else
	        {
	          /*##-5- Write data to the text file ################################*/
	          res = f_write(&MyFile, wtext, sizeof(wtext), reinterpret_cast<UINT*>(&byteswritten));

	          if((byteswritten == 0) || (res != FR_OK))
	          {
	            /* 'STM32.TXT' file Write or EOF Error */
	            Error_Handler();
	          }
	          else
	          {
	            /*##-6- Close the open text file #################################*/
	            f_close(&MyFile);

	            /*##-7- Open the text file object with read access ###############*/
	            if(res = f_open(&MyFile, "STM32.TXT", FA_READ); res != FR_OK)
	            {
	              /* 'STM32.TXT' file Open for read Error */
	              Error_Handler();
	            }
	            else
	            {
	              /*##-8- Read data from the text file ###########################*/
	              res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);

	              if((bytesread == 0) || (res != FR_OK))
	              {
	                /* 'STM32.TXT' file Read or EOF Error */
	                Error_Handler();
	              }
	              else
	              {
	                /*##-9- Close the open text file #############################*/
	                f_close(&MyFile);

	                /*##-10- Compare read data with the expected data ############*/
	                if((bytesread != byteswritten))
	                {
	                  /* Read data is different from the expected data */
	                  Error_Handler();
	                }
	                else
	                {
	                  /* Success of the demo: no error occurrence */
	                  logger.info("SD card success!");
	                }
	              }
	            }
	          }
	        }
	      }



	  /*##-11- Unlink the RAM disk I/O driver ####################################*/
	  FATFS_UnLinkDriver(SDPath);

	  /* Infinite Loop */
	  for( ;; )
	  {
	  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    osSemaphoreRelease(adc_done_sem);
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
                    goto add_to_queue; // this is ugly
                }

                idx = esp_usart_pos;
                match = true;

                for (int i = 6; i >= 0; i--) {// ESP_ERROR length
                    if (ESP_ERROR[i] != (char) esp_usart_rx_buffer[idx]) {
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
                    goto add_to_queue; // this is ugly
                }

                idx = esp_usart_pos;
                match = true;

                for (int i = 5; i >= 0; i--) {// ESP_POLL length
                    if (ESP_POLL_CMD[i] != (char) esp_usart_rx_buffer[idx]) {
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
                    osSemaphoreRelease(esp_poll_cmd_sem);
                }

add_to_queue:
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
	} else if (huart == camera.huart) {
		static size_t curr_cam_buf_pos = 0;
		++curr_cam_buf_pos;

		if (curr_cam_buf_pos >= camera.current_num_bytes) {
			osSemaphoreRelease(camera_rx_complete);
			curr_cam_buf_pos = 0;
		} else {
			auto hal_resp = HAL_UART_Receive_IT(camera.huart, &camera.camera_buff[curr_cam_buf_pos], 1);
		}
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
            Error_Handler();
	    }

	    logger.info("SHT30 init OK");
	  #endif

	  #ifdef PRESSURE_D
	    if (pressure_sensor.init() != HAL_OK) {
	  	  logger.error("Pressure sensor init FAIL");
            Error_Handler();
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

#ifdef CAMERA_D
      if (!camera.begin()) {
          // camera failed to initialize
          logger.error("Camera init FAIL");
          Error_Handler();
      } else {
          logger.info("Camera init SUCCESS");
      }

      bool status = camera.set_image_size(VC0706_640x480);

      // HAL_Delay(1000);

      if (!status) {
          logger.error("Camera unable to set image size");
          Error_Handler();
      }
#endif

        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);

        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);

	  HAL_UART_Receive_IT(&huart2, &esp_usart_rx_buffer[esp_usart_pos], 1);
	  HAL_UART_Receive_IT(&huart6, &smu_usart_rx_buffer[smu_usart_pos], 1);

      esp.send_cmd("AT+CIPSNTPCFG=1,-5,\"time.nist.gov\"");

      osSemaphoreAcquire(esp_messages_sem, 500U);

      if (esp.consume_message().find(ESP_OK) == std::string::npos) {
          logger.warn("ESP NTP command failed");
      } else {
    	  osDelay(5000U); // TODO: modify esp message queue to include +TIME_UPDATED somehow
      }

      scheduledSendTaskHandle = osThreadNew(ScheduledUpdateUploadTask, NULL, &httpTask_attributes);
      requestedSendTaskHandle = osThreadNew(RequestedUpdateUploadTask, NULL, &httpTask_attributes);
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
