/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define XST_SUCCESS                     0L
#define XST_FAILURE                     1L
#define XST_DEVICE_NOT_FOUND            2L
#define XST_DEVICE_BLOCK_NOT_FOUND      3L
#define XST_INVALID_VERSION             4L
#define XST_DEVICE_IS_STARTED           5L
#define XST_DEVICE_IS_STOPPED           6L
#define XST_FIFO_ERROR                  7L	/*!< An error occurred during an
						   operation with a FIFO such as
						   an underrun or overrun, this
						   error requires the device to
						   be reset */
#define XST_RESET_ERROR                 8L	/*!< An error occurred which requires
						   the device to be reset */
#define XST_DMA_ERROR                   9L	/*!< A DMA error occurred, this error
						   typically requires the device
						   using the DMA to be reset */
#define XST_NOT_POLLED                  10L	/*!< The device is not configured for
						   polled mode operation */
#define XST_FIFO_NO_ROOM                11L	/*!< A FIFO did not have room to put
						   the specified data into */
#define XST_BUFFER_TOO_SMALL            12L	/*!< The buffer is not large enough
						   to hold the expected data */
#define XST_NO_DATA                     13L	/*!< There was no data available */
#define XST_REGISTER_ERROR              14L	/*!< A register did not contain the
						   expected value */
#define XST_INVALID_PARAM               15L	/*!< An invalid parameter was passed
						   into the function */
#define XST_NOT_SGDMA                   16L	/*!< The device is not configured for
						   scatter-gather DMA operation */
#define XST_LOOPBACK_ERROR              17L	/*!< A loopback test failed */
#define XST_NO_CALLBACK                 18L	/*!< A callback has not yet been
						   registered */
#define XST_NO_FEATURE                  19L	/*!< Device is not configured with
						   the requested feature */
#define XST_NOT_INTERRUPT               20L	/*!< Device is not configured for
						   interrupt mode operation */
#define XST_DEVICE_BUSY                 21L	/*!< Device is busy */
#define XST_ERROR_COUNT_MAX             22L	/*!< The error counters of a device
						   have maxed out */
#define XST_IS_STARTED                  23L	/*!< Used when part of device is
						   already started i.e.
						   sub channel */
#define XST_IS_STOPPED                  24L	/*!< Used when part of device is
						   already stopped i.e.
						   sub channel */
#define XST_DATA_LOST                   26L	/*!< Driver defined error */
#define XST_RECV_ERROR                  27L	/*!< Generic receive error */
#define XST_SEND_ERROR                  28L	/*!< Generic transmit error */
#define XST_NOT_ENABLED                 29L	/*!< A requested service is not
						   available because it has not
						   been enabled */
#define XST_NO_ACCESS			30L	/* Generic access error */
#define XST_TIMEOUT                     31L	/*!< Event timeout occurred */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
// I2C addresses of BCM supervisor chips. Each communicates with 4 BCMs
typedef enum {BCM_SUP_0_3=0x50, BCM_SUP_4_7=0x51 } bcm_supervisor_t;

typedef enum {
	BCM_PAGE = 0x00,
	BCM_OPERATION = 0x01,
	BCM_CLEAR_FAULTS = 0x03,
	BCM_STATUS_BYTE = 0x78,
	BCM_STATUS_WORD = 0x79,
	BCM_STATUS_MFR_SPECIFIC = 0x80,
	BCM_READ_VIN = 0x88,
	BCM_READ_IIN = 0x89,
	BCM_READ_VOUT = 0x8B,
	BCM_READ_IOUT = 0x8C,
	BCM_TEMP_1 = 0x8D,
	BCM_MFR_ID = 0x99,
	BCM_MFR_MODEL = 0x9a,
	BCM_SERIAL = 0x9e,
	BCM_DISABLE_FAULT = 0xD7,
} bcm_cmd_t;

#define u8     uint8_t
#define u16    uint16_t
#define u32    uint16_t

#define false  0
#define true   1
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t bcm_print_supervisor_serial(bcm_supervisor_t sup)
{
	static u8 serial[16+1];
	static u8 mfr_model[18];
	int status;
	u8 header = 0;

	select_supervisor_page(sup);

	u16 bcm_status = 0;
	header = BCM_STATUS_WORD;
	//status = bcm_i2c_receive((u8)sup, &header, 1, (u8*)&bcm_status, 2);
	if (status != XST_SUCCESS) {
		//printf("%s: can't read status byte, return value: %d\n", __func__, status);
		return false;
	}
	printf("status word: x%04x\n", (unsigned int) bcm_status);

	header = BCM_SERIAL;
	status = bcm_i2c_receive((u8)sup, &header, 1, serial, sizeof(serial));
	if (status != XST_SUCCESS) {
		printf("%s: can't read serial number: %d\n", __func__, status);
		return false;
	}
	//volo
	printf("sup = x%02x: read serial number OK: %d\n", (unsigned int)sup, status);

	header = BCM_MFR_MODEL;
	status = bcm_i2c_receive((u8)sup, &header, 1, mfr_model, sizeof(mfr_model));
	if (status != XST_SUCCESS) {
		printf("%s: can't read mfr_model: %d\n", __func__, status);
		return false;
	}
	//volo
	//printf("sup = x%02x: read mfr_model OK: %d\n", (unsigned int)sup, status);

	printf("BCM supervisor x%02x mfr_model:", (unsigned int)sup);
	for (size_t i=1; i<sizeof(mfr_model); i++) {
		putchar((char)mfr_model[i]);
		printf(" x%02x (%c) ", (unsigned int)mfr_model[i], (char)mfr_model[i]);
	}
	//volo
	//printf("sup = x%02x: BCM OK: %d\n", (unsigned int)sup, status);


	printf("  serial no:");
	for (size_t i=1; i<sizeof(serial); i++) {
		//printf(" x%02x (%c) ", (unsigned int)serial[i], (char)serial[i]);
		putchar((char)serial[i]);
	}
	putchar('\n');
	//volo
	//printf("sup = x%02x: SERIAL OK: %d\n", (unsigned int)sup, status);

	if (strncmp((char*)(&mfr_model[1]), "D44TL1A0", 8) != 0) {
		printf("BCM Supervisor part number fault\n");
		return false;
		//return true; // HACK: accept broken devices
	}
	return true;
}


static int bcm_read_pmbus_value(size_t bcm_id, bcm_cmd_t cmd, float* val)
{
	int16_t raw_value = 0;
	int status;
	//status = bcm_read(bcm_id, cmd, (void*)&raw_value, 2);
	if (status != XST_SUCCESS) {
		//printf("bcm_read_pmbus_value: read failed: %d (x%08x)\n", status, (unsigned int)status);
		return status;
	}
	// rescale to physical units (V, A, degC, etc.) according to data sheet
	u32 m = 0;
	float R = 0;
	u32 b = 0;
	switch(cmd) {
	case BCM_READ_VIN:  m = 1; R = 0.1F;   b = 0; break;
	case BCM_READ_IIN:  m = 1; R = 0.001F; b = 0; break;
	case BCM_READ_VOUT: m = 1; R = 0.1F;   b = 0; break;
	case BCM_READ_IOUT: m = 1; R = 0.01F;  b = 0; break;
	case BCM_TEMP_1:    m = 1; R = 1;      b = 0; break;
	default: return XST_INVALID_PARAM;
	}

	*val = ((raw_value * R) - b) / m;
	return XST_SUCCESS;
}


#define BCM_DEV_IND_MIN 	0
#define BCM_DEV_IND_MAX		3//7 =YS=

#define BCM_NUMBER_DEV		(1+BCM_DEV_IND_MAX-BCM_DEV_IND_MIN)

#define BCM_ARRAY_LEN		(BCM_DEV_IND_MAX+1)

static const size_t bcm_to_supervisor_ch[4] = {0, 1, 2, 3};//{0, 1, 2, 3}; //bcm_to_supervisor_ch[8] = {3, 2, 5, 4, 7, 6, 1, 0};
static const size_t supervisor_to_bcm_ch[4] = {0, 1, 2, 3};//{0, 1, 2, 3};  //supervisor_to_bcm_ch[8] = {7, 6, 1, 0, 3, 2, 5, 4};
static const size_t psu_no_to_user_scheme[4] = {4, 1, 2, 3};

static uint16_t bcm_status[BCM_ARRAY_LEN] = {0};
static float bcm_meas_vin[BCM_ARRAY_LEN] = {0};
static float bcm_meas_vout[BCM_ARRAY_LEN] = {0};
static float bcm_meas_iout[BCM_ARRAY_LEN] = {0};
static float bcm_meas_iin[BCM_ARRAY_LEN] = {0};
static float bcm_meas_temp[BCM_ARRAY_LEN] = {0};
static uint32_t bcm_update_timestamp[BCM_ARRAY_LEN] = {0};


void bcm_poll()
{
	int status = 0;
	static uint32_t next_update = 0;
	static size_t next_bcm = 0;
	const size_t bcm_id = 0;
	const size_t sup_ch = bcm_to_supervisor_ch[bcm_id];
	// periodically poll BCM devices
	/*if (st_get_tick() > next_update) {
		next_update += 125;
		// get info for one BCM device
		while (next_bcm < BCM_DEV_IND_MIN) {
			next_bcm++;
		}
		const size_t bcm_id = next_bcm;
		next_bcm++;
		if (next_bcm > BCM_DEV_IND_MAX) {
			next_bcm = 0;
		}*/

		/*const size_t sup_ch = bcm_to_supervisor_ch[bcm_id];
		status = bcm_read(sup_ch, BCM_STATUS_WORD, (void*)&(bcm_status[bcm_id]), 2);
		// Remove upper 8 bits from status, these contain a code added by us to identified where the error was created
		if ((status & 0x00FFFFFF) == XST_TIMEOUT) {
			// i2c problem? Let's restart the peripheral...
			XIic_Stop(&bcm_i2c_1);
			bcm_setup_i2c();
			status = bcm_read(sup_ch, BCM_STATUS_WORD, (void*)&(bcm_status[bcm_id]), 2);
			if (status != XST_SUCCESS) {
				return;
			}
		}*/

		status = bcm_read_pmbus_value(sup_ch, BCM_READ_VIN, &(bcm_meas_vin[bcm_id]));
		if (status != XST_SUCCESS) {
			return;
		}
		status = bcm_read_pmbus_value(sup_ch, BCM_READ_VOUT, &(bcm_meas_vout[bcm_id]));
		if (status != XST_SUCCESS) {
			return;
		}
		status = bcm_read_pmbus_value(sup_ch, BCM_READ_IOUT, &(bcm_meas_iout[bcm_id]));
		if (status != XST_SUCCESS) {
			return;
		}
		status = bcm_read_pmbus_value(sup_ch, BCM_READ_IIN, &(bcm_meas_iin[bcm_id]));
		if (status != XST_SUCCESS) {
			return;
		}
		status = bcm_read_pmbus_value(sup_ch, BCM_TEMP_1, &(bcm_meas_temp[bcm_id]));
		if (status != XST_SUCCESS) {
			return;
		}
		// all reads successful, updated timestamp
		bcm_update_timestamp[bcm_id] = st_get_tick();
	//}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BCM_EN0_Pin|BCM_EN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BCM_EN0_Pin BCM_EN1_Pin */
  GPIO_InitStruct.Pin = BCM_EN0_Pin|BCM_EN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
