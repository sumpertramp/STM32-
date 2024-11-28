/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef can1RxHeader;
CAN_TxHeaderTypeDef can1TxHeader;
CAN_FilterTypeDef can1FilterConfig;

uint8_t can1RxData[8];
uint8_t can1TxData[8];
uint32_t can1TxMailbox;

uint32_t canBusTimeout = 0;
uint32_t canBusMessageBroadcastTimer = 0;


uint8_t priority;
uint8_t dataID;
uint8_t bmsAddress;
uint8_t pcAddress;
uint8_t dataContent[4];

/* Protokole uygun ID ve format */
#define UPPER_COMPUTER_ID_0 0x18100140 // Dokümana göre CAN ID
#define UPPER_COMPUTER_ID_1 0x18900140 // Dokümana göre CAN ID
#define UPPER_COMPUTER_ID_2 0x18910140 // Dokümana göre CAN ID
#define UPPER_COMPUTER_ID_3 0x18920140 // Dokümana göre CAN ID
#define UPPER_COMPUTER_ID_4 0x18930140 // Dokümana göre CAN ID
#define UPPER_COMPUTER_ID_5 0x18940140 // Dokümana göre CAN ID
#define UPPER_COMPUTER_ID_6 0x18950140 // Dokümana göre CAN ID
#define UPPER_COMPUTER_ID_7 0x18960140 // Dokümana göre CAN ID
#define UPPER_COMPUTER_ID_8 0x18970140 // Dokümana göre CAN ID
#define UPPER_COMPUTER_ID_9 0x18980140 // Dokümana göre CAN ID

#define BMS_RESPONSE_ID 0x18104001   // BMS'nin cevap ID'si

_Bool can1ErrorFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1);
void CAN_SendMessage(uint32_t dataID, uint8_t* payload);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim7.Instance)////1ms Timer Interrupt
	{
		canBusTimeout++;
		canBusMessageBroadcastTimer++;
		if (canBusMessageBroadcastTimer > 200)
		{
			can1TxHeader.ExtId = 0x18900140;
			can1TxHeader.RTR = CAN_RTR_DATA;
			can1TxHeader.IDE = CAN_ID_EXT;
			can1TxHeader.DLC = 8;
			can1TxHeader.TransmitGlobalTime = DISABLE;

			can1TxData[0] = 0;
			can1TxData[1] = 1;
			can1TxData[2] = 255;
			can1TxData[3] = 255;
			can1TxData[4] = 255;
			can1TxData[5] = 0;
			can1TxData[6] = 0;
			can1TxData[7] = 0;

            HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxData, &can1TxMailbox);
        	canBusMessageBroadcastTimer = 0;
		}

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
  MX_TIM7_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
  {
	  Error_Handler();
  }

  // CAN filtrelerini yapılandırma
 /* can1FilterConfig.FilterBank =               0;
  can1FilterConfig.SlaveStartFilterBank =     0;
  can1FilterConfig.FilterMode =               CAN_FILTERMODE_IDMASK; // ID Mask Mode
  can1FilterConfig.FilterScale =              CAN_FILTERSCALE_32BIT; // 32-bit filtreleme
  can1FilterConfig.FilterIdHigh =             (0x18104001 >> 13) & 0xFFFF; // ID'nin üst 16 biti
  can1FilterConfig.FilterIdLow =              ((0x18104001 << 3) & 0xFFF8) | 4; // ID'nin alt 16 biti ve RTR/IDE bitleri
  can1FilterConfig.FilterMaskIdHigh =         (0xFFFFFFFF >> 13) & 0xFFFF; // Maskenin üst 16 biti
  can1FilterConfig.FilterMaskIdLow =          ((0xFFFFFFFF << 3) & 0xFFF8) | 4; // Maskenin alt 16 biti ve RTR/IDE bitleri
  can1FilterConfig.FilterFIFOAssignment =     CAN_FILTER_FIFO0; // FIFO 0
  can1FilterConfig.FilterActivation =         ENABLE; // Filtreyi aktif et
  */

  can1FilterConfig.SlaveStartFilterBank =     0;
  can1FilterConfig.FilterMode =               CAN_FILTERMODE_IDMASK; // ID Mask Mode
  can1FilterConfig.FilterScale =              CAN_FILTERSCALE_32BIT; // 32-bit filtreleme
  can1FilterConfig.FilterIdHigh =             0x00000000; // ID'nin üst 16 biti
  can1FilterConfig.FilterIdLow =              0x00000000; // ID'nin alt 16 biti ve RTR/IDE bitleri
  can1FilterConfig.FilterMaskIdHigh =         0x00000000; // Maskenin üst 16 biti
  can1FilterConfig.FilterMaskIdLow =          0x00000000; // Maskenin alt 16 biti ve RTR/IDE bitleri
  can1FilterConfig.FilterFIFOAssignment =     CAN_FILTER_FIFO0; // FIFO 0
  can1FilterConfig.FilterActivation =         ENABLE; // Filtreyi aktif et
  if(HAL_CAN_ConfigFilter(&hcan1, &can1FilterConfig) != HAL_OK)
  {
	  /* Filter configuration Error */
	  Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  /* Start Error */
	  Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
	  /* Notification Error */
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if((CAN1->ESR & CAN_ESR_BOFF) == 4)
	  {
		  HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING | CAN_IT_TX_MAILBOX_EMPTY);
		  HAL_CAN_DeInit(&hcan1);
		  MX_CAN1_Init();
		  HAL_CAN_Start(&hcan1);
		  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING | CAN_IT_TX_MAILBOX_EMPTY);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  htim7.Init.Prescaler = 199;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 359;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	;//flagCanTxComplete = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1RxHeader, can1RxData) != HAL_OK)

	{
		Error_Handler();

	}

	if(can1RxHeader.ExtId == 0x18914001 && can1RxHeader.IDE == CAN_ID_EXT && can1RxHeader.DLC == 8)
	{
		// Burada bataryadan can rx ile gelen veri ayrıştırılacak
		canBusTimeout = 0;
	}
}

/*void CAN_SendMessage(uint32_t dataID, uint8_t* payload) {

	can1TxHeader.DLC = 8;             // Veri uzunluğu
    can1TxHeader.IDE = CAN_ID_EXT;         // Extended ID kullanılacak
    can1TxHeader.RTR = CAN_RTR_DATA;       // Veri çerçevesi
    can1TxHeader.ExtId = (0x18000000) |    // 0x18 üst byte olarak ekleniyor
                     (dataID << 16) | // DataID orta byte'a yerleştiriliyor
                     0x0140;          // En alt byte sabit 0x0140

    if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, payload, &can1TxMailbox) != HAL_OK) {
        can1ErrorFlag = 1; // Gönderim hatası
    }
} */


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

