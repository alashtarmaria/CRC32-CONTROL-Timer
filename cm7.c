/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "openamp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RPMSG_CHAN_NAME              "openamp_demo"  //OpenAMP chanel name
#define ADC_TO_VOLTAGE (4096.0f / 4095.0f)

xy_Typedef_system xy_system;				//System declaration

MCP3208_HandleTypeDef hmcp3208_MAA;			//mcu adc declarations
MCP3208_HandleTypeDef hmcp3208_MAB;
MCP3208_HandleTypeDef hmcp3208_SAD;

MCP3208_HandleTypeDef hmcp3208_PAA;			//power adc declarations
MCP3208_HandleTypeDef hmcp3208_PAB;
MCP3208_HandleTypeDef hmcp3208_PAC;

MCP4922_HandleTypeDef mcp4922_VDA;			//dac declarations
MCP4922_HandleTypeDef mcp4922_VDB;

Memory_HandleTypeDef ROM_U1;				//eeprom daclerations
Memory_HandleTypeDef ROM_U2;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GRAPH_POINTS 11
#define DWT_DELAY_MS 20

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

//NTC sıcaklık hesabı için gereken sabitler. Bu değerler, termistör datasheet'ine göre tanımlanmış.
#define NTC_R25 10000.0f     // 25°C'deki nominal direnç (10k)
#define NTC_B25_85 3570.0f   // NTCS0805E3103*MT modeli için B25/85 değeri
#define KELVIN_OFFSET 273.15f
#define SERIES_RESISTOR 10000.0f  // Seri bağlı direnç değeri (10k)



typedef struct __attribute__((packed)) {
    float volt_input;
    float volt_sys;
    float volt_3;
    float volt_5;
    float volt_12v;
    float volt_15v;
} AdcPacket_t;



typedef struct __attribute__((packed)) {
   // uint32_t graph_data[GRAPH_POINTS];
    AdcPacket_t adc_values;
    uint32_t checksum;
} ScreenPacket_t;


#define NUM_BUFFERS 2
volatile uint8_t current_write = 0;
volatile uint8_t buffer_lock[NUM_BUFFERS] = {0};
ScreenPacket_t buffers[NUM_BUFFERS];
//volatile uint8_t current_read = 1;
volatile uint8_t buffer_ready[NUM_BUFFERS] = {0};

volatile int message_received;
char received_response[64];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

static volatile int service_created;  //CM4 tarafı endpoint’i oluşturduğunda 1 olur.
static struct rpmsg_endpoint rp_endpoint; // OpenAMP haberleşmesinde kullanılan uç nokta.
uint32_t start, end;
float elapsed_us;
float bps;
float elapsed_s;
float elapsed_ms;
uint32_t t_now;
//static uint32_t t_prev = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
void service_destroy_cb(struct rpmsg_endpoint *ept);
void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//MCP3208 üzerindeki her kanalın ismi ve voltaj/sensör değerine dönüşüm katsayısı burada.

const char* channelNamesPAA[8] = {
    "VOLTAGE INPUT",
    "VOLTAGE SYSTEM",
    "VOLTAGE 3V",
    "VOLTAGE 5V",
    "VOLTAGE 12V",
    "VOLTAGE 15V",
    "BUTTON VOLTAGE",
	"SYSTEM CURRENT mA"
};
float channelScalesPAA[8] = {
    10.0f, // VOLTAGE INPUT
    10.0f, // VOLTAGE SYSTEM
    5.0f, // VOLTAGE 3V
    5.0f, // VOLTAGE 5V
    5.0f, // VOLTAGE 12V
    5.0f, // VOLTAGE 15V
    10.0f, // BUTTON VOLTAGE
    4000.0f  // SYSTEM CURRENT mA
};

const char* channelNamesPAB[8] = {
    "BAT1_CURRENT",
	"BAT1_VOLTAGE",
	"BAT2_CURRENT",
	"BAT2_VOLTAGE",
	"BAT3_CURRENT",
	"BAT3_VOLTAGE",
    "CURRENT_NEBUL",
	"VOLTAGE_NEBUL"
};
float channelScalesPAB[8] = {
    50.0f,//BAT1_CURRENT
    10.0f,//BAT1_VOLTAGE
    50.0f,//BAT2_CURRENT
    10.0f,//BAT2_VOLTAGE
	(1.0f/8.0f),//BAT3_CURRENT
    (5.0f/3.0f),//BAT3_VOLTAGE
	(1000.0f/920.0f),//CURRENT_NEBUL
    10.0f  //"VOLTAGE_NEBUL"
};

const char* channelNamesPAC[8] = {
    "VVI VOLTAGE",
    "VVI CURRENT",
    "VVO VOLTAGE",
    "VVO CURRENT",
    "VCI CURRENT mA",
    "VCO CURRENT mA",
    "BLOWER VOLTAGE",
    "BLOWER CURRENT"
};

float channelScalesPAC[8] = {
    10.0f, // VVI VOLTAGE
    10.0f, // VVI CURRENT
    10.0f, // VVO VOLTAGE
    10.0f, // VVO CURRENT
    50.0f, // VCI CURRENT
    50.0f, // VCO CURRENT
    10.0f, // BLOWER VOLTAGE
    2000.0f  // BLOWER CURRENT
};

const char* channelNamesMAA[8] = {
    "DISPLAY_CURRENT",
    "SPO2_CURRENT",
	"PM_CURRENT",
	"ETCO2_CURRENT",
    "PROX_CURRENT",
	"COMM_CURRENT",
	"COMM_VOLTAGE",
	"LIGHT_INTENSITY"

};

float channelScalesMAA[8] = {
	994.0f,
	201.0f,
	201.0f,
	201.0f,
	201.0f,
	50.0f,
	10.0f,
	1.0f
};

const char* channelNamesMAB[8] = {
    "TEMP1",
    "TEMP2",
	"TEMP3",
	"TEMP4",
    "TEMP_FLOW",
	"TEMP_ENVIRONMENT",
	"TEMP_HUMIDITY",
	"HUMIDITY"

};

float channelScalesMAB[8] = {
	1.0f,
	1.0f,
	1.0f,
	1.0f,
	1.0f,
	1.0f,
	1.25f,
	1.25f
};
const char* channelNamesSAD[8] = {
    "O2_CHANNEL_FLOW",
    "O2_CHANNEL_FiO2",
	"AIR_CHANNEL_FLOW",
	"AIR_CHANNEL_FiO2",
    "PRESSURE_O2",
	"PRESSURE_INSP",
	"PRESSURE_PEEP",
	"SAD_TP1"

};

float channelScalesSAD[8] = {
	0.625f,
	0.625f,
	0.625f,
	0.625f,
	1.12195f,
	1.12195f,
	1.12195f
};


//ARM Cortex-M7 DWT sayaç aktif hâle getirilir. Cycle sayar (Cycle Count Register) açılır.

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable TRC
    DWT->CYCCNT = 0;                                // Reset the clock cycle counter value
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable  clock cycle counter
}



void read_adc_all(xy_Typedef_system *xy_system)

{
    // PAA = Power ADC A grubu
	xy_system->power_adc.voltage_input 		= (float)MCP3208_spi_read_adc(&hmcp3208_PAA, 0) * ADC_TO_VOLTAGE * channelScalesPAA[0];
	xy_system->power_adc.voltage_system 	= (float)MCP3208_spi_read_adc(&hmcp3208_PAA, 1) * ADC_TO_VOLTAGE * channelScalesPAA[1];
	xy_system->power_adc.voltage_3v 		= (float)MCP3208_spi_read_adc(&hmcp3208_PAA, 2) * ADC_TO_VOLTAGE * channelScalesPAA[2];
	xy_system->power_adc.voltage_5v 		= (float)MCP3208_spi_read_adc(&hmcp3208_PAA, 3) * ADC_TO_VOLTAGE * channelScalesPAA[3];
	xy_system->power_adc.voltage_12v 		= (float)MCP3208_spi_read_adc(&hmcp3208_PAA, 4) * ADC_TO_VOLTAGE * channelScalesPAA[4];
	xy_system->power_adc.voltage_15v 		= (float)MCP3208_spi_read_adc(&hmcp3208_PAA, 5) * ADC_TO_VOLTAGE * channelScalesPAA[5];


    float adc_voltage = MCP3208_spi_read_adc(&hmcp3208_MAB, 0) * ADC_TO_VOLTAGE;

}


// CRC32 hesaplama fonksiyonu
uint32_t crc32(const void *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *bytes = (const uint8_t *)data;

    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint32_t)bytes[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }

    return crc ^ 0xFFFFFFFF;
}

void fill_screen_packet(ScreenPacket_t *pkt, const xy_Typedef_system *sys)
{
    pkt->adc_values.volt_input = sys->power_adc.voltage_input;
    pkt->adc_values.volt_sys   = sys->power_adc.voltage_system;
    pkt->adc_values.volt_3     = sys->power_adc.voltage_3v;
    pkt->adc_values.volt_5     = sys->power_adc.voltage_5v;
    pkt->adc_values.volt_12v   = sys->power_adc.voltage_12v;
    pkt->adc_values.volt_15v   = sys->power_adc.voltage_15v;

    pkt->checksum = crc32(pkt, sizeof(ScreenPacket_t) - sizeof(uint32_t));
}


void send_screen_packet_pingpong(void)
{
    uint8_t index = current_write;

    if (buffer_lock[index]) return;  // Eğer bu buffer kilitliyse yazma

    read_adc_all(&xy_system);  // ADC oku
    fill_screen_packet(&buffers[index], &xy_system);  // Paket oluştur

    __DSB();
    start = DWT->CYCCNT;
    OPENAMP_send(&rp_endpoint, (void *)&buffers[index], sizeof(ScreenPacket_t));
    end = DWT->CYCCNT;
    __DSB();

    buffer_lock[index] = 1;
    buffer_ready[index] = 1;

    // BPS hesapla
    elapsed_us = (end - start) * (1.0f / (SystemCoreClock / 1000000.0f));
    elapsed_s = elapsed_us / 1000000.0f;
    bps = (float)(sizeof(ScreenPacket_t) * 8) / elapsed_s;

    current_write = (current_write + 1) % NUM_BUFFERS;
    buffer_lock[index] = 0;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        send_screen_packet_pingpong();  // Her 20 ms'de veri gönder

        char msg[64];
        sprintf(msg, "[%lu ms] CM7 paket sent\r\n", HAL_GetTick());
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
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
  int32_t timeout;

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */

/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
	   //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Timeout while waiting for CPU2 to boot\r\n", strlen("Timeout while waiting for CPU2 to boot\r\n"), HAL_MAX_DELAY);
       Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);

/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  DWT_Init();
  spi_init();

  /* Initialize the mailbox use notify the other core on new message */
  MAILBOX_Init();
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"CM7 started\r\n", strlen("CM7 started\r\n"), HAL_MAX_DELAY);
  rpmsg_init_ept(&rp_endpoint, "openamp_demo", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, rpmsg_recv_callback, service_destroy_cb);

  MX_OPENAMP_Init(RPMSG_MASTER, new_service_cb);
  OPENAMP_Wait_EndPointready(&rp_endpoint);



  char clk_msg[64];
  uint32_t hclk = HAL_RCC_GetHCLKFreq();
  sprintf(clk_msg, "HCLK Frequency: %lu Hz\r\n", hclk);
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)clk_msg, strlen(clk_msg), HAL_MAX_DELAY);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    static uint32_t last_send_time = 0;

	    OPENAMP_check_for_message();  // Zaten var

	    uint32_t now = HAL_GetTick();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 212;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 106;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1199;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 3999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 239;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MCU_SPI2_CSEEPROM_3V_Pin|buzzer_high_low_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_SPI2_CS2_EEPROM_3V_GPIO_Port, MCU_SPI2_CS2_EEPROM_3V_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, BUZZER_SOURCE_LEFT_3V_Pin|BUZZER_SOURCE_RIGHT_3V_Pin|BUZZER_SINK_LEFT_3V_Pin|BUZZER_SINK_RIGHT_3V_Pin
                          |BUZZER_VOLUME_HI_3V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DISPLAY_ONOFF_3V_Pin|VALVE_SPI_CSDAC1_3V_Pin|MCU_SPI_CSMAB_3V_Pin|MCU_SPI_CSMAA_3V_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VALVE_SPI_CSDAC2_3V_GPIO_Port, VALVE_SPI_CSDAC2_3V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MCU_SPI_CSEEPROM_3V_Pin|MCU_SPI_CSSAD_3V_Pin|VALVE_SPI_SHDNDAC_3V_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BLOWER_ENABLE_MOSI_3V_Pin|BLOWER_BRAKE_MOSI_3Vs_Pin|RGB_RED_3V_Pin|RGB_GREEN_3V_Pin
                          |RGB_BLUE_3V_Pin|BLOWER_ONOFF_3V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(COMM_ONOFF_3V_GPIO_Port, COMM_ONOFF_3V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, POWER_SPI_CSADC3_3V_Pin|POWER_SPI_CSADC1_3V_Pin|POWER_SPI_CSADC2_3V_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MCU_SPI2_CSEEPROM_3V_Pin MCU_SPI2_CS2_EEPROM_3V_Pin */
  GPIO_InitStruct.Pin = MCU_SPI2_CSEEPROM_3V_Pin|MCU_SPI2_CS2_EEPROM_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_SOURCE_LEFT_3V_Pin BUZZER_SOURCE_RIGHT_3V_Pin BUZZER_SINK_LEFT_3V_Pin BUZZER_SINK_RIGHT_3V_Pin
                           BUZZER_VOLUME_HI_3V_Pin */
  GPIO_InitStruct.Pin = BUZZER_SOURCE_LEFT_3V_Pin|BUZZER_SOURCE_RIGHT_3V_Pin|BUZZER_SINK_LEFT_3V_Pin|BUZZER_SINK_RIGHT_3V_Pin
                          |BUZZER_VOLUME_HI_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPLAY_ONOFF_3V_Pin VALVE_SPI_CSDAC2_3V_Pin VALVE_SPI_CSDAC1_3V_Pin MCU_SPI_CSMAB_3V_Pin
                           MCU_SPI_CSMAA_3V_Pin */
  GPIO_InitStruct.Pin = DISPLAY_ONOFF_3V_Pin|VALVE_SPI_CSDAC2_3V_Pin|VALVE_SPI_CSDAC1_3V_Pin|MCU_SPI_CSMAB_3V_Pin
                          |MCU_SPI_CSMAA_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_SPI_CSEEPROM_3V_Pin MCU_SPI_CSSAD_3V_Pin VALVE_SPI_SHDNDAC_3V_Pin */
  GPIO_InitStruct.Pin = MCU_SPI_CSEEPROM_3V_Pin|MCU_SPI_CSSAD_3V_Pin|VALVE_SPI_SHDNDAC_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENA_PHASE_A_3V_Pin ENA_PHASE_B_3V_Pin */
  GPIO_InitStruct.Pin = ENA_PHASE_A_3V_Pin|ENA_PHASE_B_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ENA_BUTTON_3V_Pin BUTTON1_3V_Pin BUTTON2_3V_Pin BUTTON3_3V_Pin
                           BUTTON4_3V_Pin */
  GPIO_InitStruct.Pin = ENA_BUTTON_3V_Pin|BUTTON1_3V_Pin|BUTTON2_3V_Pin|BUTTON3_3V_Pin
                          |BUTTON4_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BLOWER_ENABLE_MOSI_3V_Pin BLOWER_BRAKE_MOSI_3Vs_Pin RGB_RED_3V_Pin RGB_GREEN_3V_Pin
                           RGB_BLUE_3V_Pin BLOWER_ONOFF_3V_Pin */
  GPIO_InitStruct.Pin = BLOWER_ENABLE_MOSI_3V_Pin|BLOWER_BRAKE_MOSI_3Vs_Pin|RGB_RED_3V_Pin|RGB_GREEN_3V_Pin
                          |RGB_BLUE_3V_Pin|BLOWER_ONOFF_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : COMM_ONOFF_3V_Pin */
  GPIO_InitStruct.Pin = COMM_ONOFF_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(COMM_ONOFF_3V_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_SPI_CSADC3_3V_Pin POWER_SPI_CSADC1_3V_Pin POWER_SPI_CSADC2_3V_Pin */
  GPIO_InitStruct.Pin = POWER_SPI_CSADC3_3V_Pin|POWER_SPI_CSADC1_3V_Pin|POWER_SPI_CSADC2_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : buzzer_high_low_Pin */
  GPIO_InitStruct.Pin = buzzer_high_low_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(buzzer_high_low_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void spi_init(void)
{
	  hmcp3208_PAA.hspi = &hspi1;
	  hmcp3208_PAA.CS_GPIO_Port = GPIOA;
	  hmcp3208_PAA.CS_Pin = GPIO_PIN_11;
	  MCP3208_Init(&hmcp3208_PAA);

	  hmcp3208_PAB.hspi = &hspi1;
	  hmcp3208_PAB.CS_GPIO_Port = GPIOA;
	  hmcp3208_PAB.CS_Pin = GPIO_PIN_12;
	  MCP3208_Init(&hmcp3208_PAB);

	  hmcp3208_PAC.hspi = &hspi1;
	  hmcp3208_PAC.CS_GPIO_Port = GPIOA;
	  hmcp3208_PAC.CS_Pin = GPIO_PIN_8;
	  MCP3208_Init(&hmcp3208_PAC);

	  mcp4922_VDA.hspi = &hspi2;
	  mcp4922_VDA.CS_Port = GPIOC;
	  mcp4922_VDA.CS_Pin = GPIO_PIN_4;

	  mcp4922_VDB.hspi = &hspi2;
	  mcp4922_VDB.CS_Port = GPIOC;
	  mcp4922_VDB.CS_Pin = GPIO_PIN_3;

	  hmcp3208_MAA.hspi = &hspi3;
	  hmcp3208_MAA.CS_GPIO_Port = GPIOC;
	  hmcp3208_MAA.CS_Pin = GPIO_PIN_9;
	  MCP3208_Init(&hmcp3208_MAA);

	  hmcp3208_MAB.hspi = &hspi3;
	  hmcp3208_MAB.CS_GPIO_Port = GPIOC;
	  hmcp3208_MAB.CS_Pin = GPIO_PIN_8;
	  MCP3208_Init(&hmcp3208_MAB);

	  hmcp3208_SAD.hspi = &hspi3;
	  hmcp3208_SAD.CS_GPIO_Port = GPIOB;
	  hmcp3208_SAD.CS_Pin = GPIO_PIN_1;
	  MCP3208_Init(&hmcp3208_SAD);

	 ROM_U1.hspi = &hspi4;
	  ROM_U1.CS_Port = GPIOE;
	  ROM_U1.CS_Pin = GPIO_PIN_3;

	  ROM_U2.hspi = &hspi4;
	  ROM_U2.CS_Port = GPIOE;
	  ROM_U2.CS_Pin = GPIO_PIN_4;
}

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
    memcpy(received_response, data, len);
    received_response[len] = '\0';
    message_received = 1;
    return 0;
}

void service_destroy_cb(struct rpmsg_endpoint *ept)
{
    service_created = 0;
}

void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
    OPENAMP_create_endpoint(&rp_endpoint, name, dest, rpmsg_recv_callback, service_destroy_cb);
    service_created = 1;
}


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
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

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
