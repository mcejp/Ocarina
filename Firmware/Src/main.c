/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include <eforce/tx.h>

#include "ocarina/ocarina.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(int silent, int loopback, int prescaler);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback() {
}

void ConfigureCAN(int silent, int loopback, int bitrate) {
    HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);

    switch (bitrate) {
        case  125000: MX_CAN_Init(silent, loopback, 24); break;
        case  250000: MX_CAN_Init(silent, loopback, 12); break;
        case  500000: MX_CAN_Init(silent, loopback, 6); break;
        case 1000000: MX_CAN_Init(silent, loopback, 3); break;
    }

    HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
}

static void setOutput(GPIO_TypeDef* gpio, uint16_t mask) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = mask;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStructure);
}

static void setPullDown(GPIO_TypeDef* gpio, uint16_t mask) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = mask;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(gpio, &GPIO_InitStructure);
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  MX_GPIO_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //ConfigureCAN(0, 0, 500000);
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  HAL_SYSTICK_Config(SystemCoreClock / 1000);

  ocarinaMain();
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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#if BOARD >= 3000
  // 12MHz RCC, 48MHz PLL
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
#else
  // 8MHz RCC, 48MHz PLL
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
#endif
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      ocarinaPanic(PANIC_CLOCK_STARTUP);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
      ocarinaPanic(PANIC_CLOCK_STARTUP);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      ocarinaPanic(PANIC_CLOCK_STARTUP);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
void MX_CAN_Init(int silent, int loopback, int prescaler)
{
  hcan.Instance = CAN;
  hcan.Init.Prescaler = prescaler;
  hcan.Init.Mode = (silent ? CAN_MODE_SILENT : 0) | (loopback ? CAN_MODE_LOOPBACK : 0);
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_13TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK) {
      // Could be clock misconfiguration as well as transceiver failure
      ocarinaPanic(PANIC_CAN_INIT);
  }

  CAN_FilterConfTypeDef filter;
  filter.FilterIdLow = 0;
  filter.FilterIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterNumber = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterActivation = ENABLE;
  filter.BankNumber = 0;
  HAL_CAN_ConfigFilter(&hcan, &filter);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

#if BOARD >= 3000
    setOutput(GPIOA, LED_500K_MASK | LED_250K_MASK | LED_1M_MASK | LED_125K_MASK | LED_TX_MASK);
    setOutput(GPIOB, LED_RX_MASK | LED_ERROR_MASK | LED_READY_MASK);

    setPullDown(ISO_SENSE_GPIO, ISO_SENSE_MASK);
#else
    setOutput(GPIOB, 0xff);
#endif
}

/* USER CODE BEGIN 4 */

uint32_t txGetTimeMillis() {
	return HAL_GetTick();
}

void WriteMailbox(uint8_t mailbox, CAN_ID_t id, const uint8_t* data, size_t length) {
	CAN_TypeDef* instance = CAN;

    if (IS_STD_ID(id)) {
        instance->sTxMailBox[mailbox].TIR = (GET_STD_ID(id) << 21);
    }
    else {
        instance->sTxMailBox[mailbox].TIR = (GET_EXT_ID(id) << 3) | CAN_TI0R_IDE;
    }

    instance->sTxMailBox[mailbox].TDTR = length;

	uint32_t regs[2];
	memcpy(regs, data, length);
	instance->sTxMailBox[mailbox].TDLR = regs[0];
	instance->sTxMailBox[mailbox].TDHR = regs[1];
}

int txSendCANMessage(CAN_ID_t id, const void* data, size_t length) {
	CAN_TypeDef* instance = CAN;
	int mailbox = 0;

	instance->sTxMailBox[mailbox].TIR &= CAN_TI0R_TXRQ;
	WriteMailbox(mailbox, id, data, length);
	instance->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;

    ocarinaBlinkTx();
	return 0;
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
