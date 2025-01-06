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
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RPM 6666.0f
#define MAF 66.66f
#define ECT 66.0f
#define RPM_FREEZE 7777.0f
#define MAF_FREEZE 77.77f
#define ECT_FREEZE 77.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
uint8_t canDatacheck = 0;

FDCAN_TxHeaderTypeDef txHeader;
uint8_t txData[8];

uint8_t pids_supported_01_20_service_01[8] =
{ 0x06, 0x41, 0x00, 0xC8, 0x11, 0x00, 0x00, 0xAA }; // Monitor status, DTC that caused freeze frame, ECT, RPM, MAF

// https://x-engineer.org/obd-diagnostic-service-mode-02-request-powertrain-freeze-frame-data/
uint8_t pids_supported_01_20_service_02[8] =
{ 0x07, 0x42, 0x00, 0x00, 0x48, 0x11, 0x00, 0x00 }; // DTC that caused freeze frame, ECT, RPM, MAF

uint32_t frame_separation_time = 0;

uint8_t malfunction_indicator_light = 1;

float rpm = RPM;
float ect = ECT;
float maf = MAF;
float rpm_freeze = RPM_FREEZE;
float ect_freeze = ECT_FREEZE;
float maf_freeze = MAF_FREEZE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SendCanFrame(void);
void Dec2HexTwoBytes(uint16_t dec_value, uint8_t *msb_value, uint8_t *lsb_value);
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
	MX_USART2_UART_Init();
	MX_FDCAN1_Init();
	/* USER CODE BEGIN 2 */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK)
	{
		Error_Handler();
	}

	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (canDatacheck == 1)
		{
			canDatacheck = 0;
			/************************************************************************
			 *							Service 01									*
			 ************************************************************************/
			// Service 01 supported PIDs
			if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x02)
					&& (rxData[1] == 0x01) && (rxData[2] == 0x00))
			{
				txHeader.Identifier = 0x7E8;
				memcpy(txData, pids_supported_01_20_service_01, 8);
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_BLUE_LED_GPIO_Port, USER_BLUE_LED_Pin);
			}
			// Service 01 PID 0x01: Monitor status since DTCs cleared
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x02)
					&& (rxData[1] == 0x01) && (rxData[2] == 0x01))
			{
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x06;
				txData[1] = 0x41;
				txData[2] = 0x01;
				txData[3] = 0x84; // MIL on, 4 DTCs
				txData[4] = 0x08; // Engine type: compression ignition (e.g. Diesel engines)
				txData[5] = 0x00;
				txData[6] = 0x00;
				txData[7] = 0xAA; // padding bytes
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_GREEN_LED_GPIO_Port,
				USER_GREEN_LED_Pin);
			}
			// Service 01 PID 0x02: DTC that caused freeze frame to be stored
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x02)
					&& (rxData[1] == 0x01) && (rxData[2] == 0x02))
			{
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x04;
				txData[1] = 0x41;
				txData[2] = 0x02;
				txData[3] = 0x00 | 0x01;
				txData[4] = 0x13;
				txData[5] = 0xAA; // padding bytes
				txData[6] = 0xAA;
				txData[7] = 0xAA;
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_GREEN_LED_GPIO_Port,
				USER_GREEN_LED_Pin);
			}
			// Service 01: stream RPM
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x02)
					&& (rxData[1] == 0x01) && (rxData[2] == 0x0C))
			{
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x04;
				txData[1] = 0x41;
				txData[2] = 0x0C;
				Dec2HexTwoBytes((uint16_t) (rpm * 4.0f), &txData[3],
						&txData[4]);
				txData[5] = 0xAA;
				txData[6] = 0xAA;
				txData[7] = 0xAA;
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_YELLOW_LED_GPIO_Port,
				USER_YELLOW_LED_Pin);
			}
			// Service 01: stream ECT
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x02)
					&& (rxData[1] == 0x01) && (rxData[2] == 0x05))
			{
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x03;
				txData[1] = 0x41;
				txData[2] = 0x05;
				txData[3] = (uint8_t) (ect + 40);
				txData[4] = 0xAA;
				txData[5] = 0xAA;
				txData[6] = 0xAA;
				txData[7] = 0xAA;
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_YELLOW_LED_GPIO_Port,
				USER_YELLOW_LED_Pin);
			}
			// Service 01: stream MAF
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x02)
					&& (rxData[1] == 0x01) && (rxData[2] == 0x10))
			{
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x04;
				txData[1] = 0x41;
				txData[2] = 0x10;
				Dec2HexTwoBytes((uint16_t) (maf * 100.0f), &txData[3],
						&txData[4]);
				txData[5] = 0xAA;
				txData[6] = 0xAA;
				txData[7] = 0xAA;
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_YELLOW_LED_GPIO_Port,
				USER_YELLOW_LED_Pin);
			}
			/************************************************************************
			 *							Service 02									*
			 ************************************************************************/
			// DTC that caused freeze frame 0x00 to be stored
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x03)
					&& (rxData[1] == 0x02) && (rxData[2] == 0x02)
					&& (rxData[3] == 0x00))
			{
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x05;
				txData[1] = 0x42;
				txData[2] = 0x02;
				txData[3] = 0x00;
				txData[4] = 0x00 | 0x00;
				txData[5] = 0xB7; // P00B7
				txData[6] = 0xAA; // padding bytes
				txData[7] = 0xAA;
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_GREEN_LED_GPIO_Port,
				USER_GREEN_LED_Pin);
			}
			// Service 02 supported PIDs in freeze frame 0x00
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x03)
					&& (rxData[1] == 0x02) && (rxData[2] == 0x00)
					&& (rxData[3] == 0x00)) // Freeze PIDs list
			{
				txHeader.Identifier = 0x7E8;
				memcpy(txData, pids_supported_01_20_service_02, 8);
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_ORANGE_LED_GPIO_Port,
				USER_ORANGE_LED_Pin);
			}
			// Freeze RPM
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x03)
					&& (rxData[1] == 0x02) && (rxData[2] == 0x0C /* PID */)
					&& (rxData[3] == 0x00 /* frame */))
			{
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x05;
				txData[1] = 0x42;
				txData[2] = 0x0C; // PID
				txData[3] = 0x00; // frame
				Dec2HexTwoBytes((uint16_t) (rpm_freeze * 4.0f), &txData[4],
						&txData[5]);
				txData[6] = 0xAA;
				txData[7] = 0xAA;
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_GREEN_LED_GPIO_Port,
				USER_GREEN_LED_Pin);
			}
			// Freeze ECT
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x03)
					&& (rxData[1] == 0x02) && (rxData[2] == 0x05)
					&& (rxData[3] == 0x00))
			{
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x04;
				txData[1] = 0x42;
				txData[2] = 0x05;
				txData[3] = 0x00;
				txData[4] = (uint8_t) (ect_freeze + 40);
				txData[5] = 0xAA;
				txData[6] = 0xAA;
				txData[7] = 0xAA;
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_GREEN_LED_GPIO_Port,
				USER_GREEN_LED_Pin);
			}
			// Freeze MAF
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x03)
					&& (rxData[1] == 0x02) && (rxData[2] == 0x10)
					&& (rxData[3] == 0x00))
			{
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x05;
				txData[1] = 0x42;
				txData[2] = 0x10;
				txData[3] = 0x00;
				Dec2HexTwoBytes((uint16_t) (maf_freeze * 100.0f), &txData[4],
						&txData[5]);
				txData[6] = 0xAA;
				txData[7] = 0xAA;
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_GREEN_LED_GPIO_Port,
				USER_GREEN_LED_Pin);
			}
			/************************************************************************
			 *							Service 03									*
			 ************************************************************************/
			// Exemplary DTCs
			else if ((malfunction_indicator_light == 1)
					&& (rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x01)
					&& (rxData[1] == 0x03))
			{
				txHeader.Identifier = 0x7E8;
				// P00B7, B1927, U0294 and P0601
				// Frame 1
				txData[0] = 0x10; // extended frame
				txData[1] = 0x0A; // payload length
				txData[2] = 0x43;      // response for Service 03
				txData[3] = 0x04; // number of DTCs
				txData[4] = 0x00 | 0x00;   // first DTC
				txData[5] = 0xB7; // P00B7
				txData[6] = 0x80 | 0x19;      // second DTC
				txData[7] = 0x27; // B1927
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_ORANGE_LED_GPIO_Port,
				USER_ORANGE_LED_Pin);
			}
			else if ((malfunction_indicator_light == 1)
					&& (rxHeader.Identifier == 0x7E8 - 0x008)
					&& (rxData[0] == 0x30) && (rxData[1] == 0x00)) // control frame
			{
				frame_separation_time = rxData[2]; // ms
				// Frame 2
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x21; // Continuation frame identifier [PCI and seq num]
				txData[1] = 0xC0 | 0x02; // third DTC
				txData[2] = 0x94; // U0294
				txData[3] = 0x00 | 0x06; // fourth DTC
				txData[4] = 0x01; // P0601
				txData[5] = 0xAA; // padding bytes
				txData[6] = 0xAA;
				txData[7] = 0xAA;
				HAL_Delay(frame_separation_time);
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_ORANGE_LED_GPIO_Port,
				USER_ORANGE_LED_Pin);
			}
			// Service 03 no DTCs
			else if ((malfunction_indicator_light == 0)
					&& (rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x01)
					&& (rxData[1] == 0x03))
			{
				txHeader.Identifier = 0x7E8;
				// No DTC
				txData[0] = 0x02; // Number of additional bytes (including the DTC)
				txData[1] = 0x43; // response for Service 03
				txData[2] = 0x00; // number of DTCs
				txData[3] = 0xAA; // padding bytes
				txData[4] = 0xAA;
				txData[5] = 0xAA;
				txData[6] = 0xAA;
				txData[7] = 0xAA;
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_ORANGE_LED_GPIO_Port,
				USER_ORANGE_LED_Pin);
			}
			/************************************************************************
			 *							Service 04									*
			 ************************************************************************/
			// Service 04: clear all DTCs
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x01)
					&& (rxData[1] == 0x04))
			{
				malfunction_indicator_light = 0;
			}
			else
			{
				__NOP();
			}
		}
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{

		memset(rxData, 0x00, 8);
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData)
				!= HAL_OK)
		{
			Error_Handler();
		}
		else
		{
			canDatacheck = 1;
		}
	}
}

void SendCanFrame(void)
{
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
	{
		Error_Handler();
	}
}

void Dec2HexTwoBytes(uint16_t dec_value, uint8_t *msb_value, uint8_t *lsb_value)
{
	*lsb_value = dec_value & 0x00FF;
	*msb_value = dec_value >> 8;
}
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
