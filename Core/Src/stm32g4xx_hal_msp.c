/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        MSP Initialization
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_DisableUCPDDeadBattery();
}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED  = 0;
static uint32_t HAL_RCC_ADC345_CLK_ENABLED = 0;

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (hadc->Instance == ADC1)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        PeriphClkInit.Adc12ClockSelection  = RCC_ADC12CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        HAL_RCC_ADC12_CLK_ENABLED++;
        if (HAL_RCC_ADC12_CLK_ENABLED == 1U)
            __HAL_RCC_ADC12_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_12;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    }
    else if (hadc->Instance == ADC2)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        PeriphClkInit.Adc12ClockSelection  = RCC_ADC12CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        HAL_RCC_ADC12_CLK_ENABLED++;
        if (HAL_RCC_ADC12_CLK_ENABLED == 1U)
            __HAL_RCC_ADC12_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin  = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_11;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    }
    else if (hadc->Instance == ADC3)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
        PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        HAL_RCC_ADC345_CLK_ENABLED++;
        if (HAL_RCC_ADC345_CLK_ENABLED == 1U)
            __HAL_RCC_ADC345_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin  = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (hadc->Instance == ADC4)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
        PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        HAL_RCC_ADC345_CLK_ENABLED++;
        if (HAL_RCC_ADC345_CLK_ENABLED == 1U)
            __HAL_RCC_ADC345_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin  = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        HAL_RCC_ADC12_CLK_ENABLED--;
        if (HAL_RCC_ADC12_CLK_ENABLED == 0U)
            __HAL_RCC_ADC12_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_2);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_12);
        HAL_DMA_DeInit(hadc->DMA_Handle);
    }
    else if (hadc->Instance == ADC2)
    {
        HAL_RCC_ADC12_CLK_ENABLED--;
        if (HAL_RCC_ADC12_CLK_ENABLED == 0U)
            __HAL_RCC_ADC12_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
        HAL_DMA_DeInit(hadc->DMA_Handle);
    }
    else if (hadc->Instance == ADC3)
    {
        HAL_RCC_ADC345_CLK_ENABLED--;
        if (HAL_RCC_ADC345_CLK_ENABLED == 0U)
            __HAL_RCC_ADC345_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
    }
    else if (hadc->Instance == ADC4)
    {
        HAL_RCC_ADC345_CLK_ENABLED--;
        if (HAL_RCC_ADC345_CLK_ENABLED == 0U)
            __HAL_RCC_ADC345_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);
    }
}

void HAL_COMP_MspInit(COMP_HandleTypeDef *hcomp)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hcomp->Instance == COMP1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin  = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(COMP1_2_3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
    }
    else if (hcomp->Instance == COMP2)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin  = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(COMP1_2_3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
    }
}

void HAL_COMP_MspDeInit(COMP_HandleTypeDef *hcomp)
{
    if (hcomp->Instance == COMP1)
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);
    else if (hcomp->Instance == COMP2)
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
}

void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
    if (hdac->Instance == DAC1)
        __HAL_RCC_DAC1_CLK_ENABLE();
}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac)
{
    if (hdac->Instance == DAC1)
        __HAL_RCC_DAC1_CLK_DISABLE();
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM20)
        __HAL_RCC_TIM20_CLK_ENABLE();
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM20)
        __HAL_RCC_TIM20_CLK_DISABLE();
}

static void TimIcGpio(GPIO_TypeDef *port, uint16_t pin, uint32_t af)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = af;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim_ic)
{
    if (htim_ic->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        TimIcGpio(GPIOA, GPIO_PIN_1, GPIO_AF1_TIM2);
    }
    else if (htim_ic->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        TimIcGpio(GPIOA, GPIO_PIN_4, GPIO_AF2_TIM3);
    }
    else if (htim_ic->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        TimIcGpio(GPIOA, GPIO_PIN_11, GPIO_AF10_TIM4);
    }
    else if (htim_ic->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        TimIcGpio(GPIOA, GPIO_PIN_8, GPIO_AF6_TIM1);
    }
    else if (htim_ic->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        TimIcGpio(GPIOB, GPIO_PIN_2, GPIO_AF2_TIM5);
    }
    else if (htim_ic->Instance == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        TimIcGpio(GPIOB, GPIO_PIN_6, GPIO_AF5_TIM8);
    }
    else if (htim_ic->Instance == TIM16)
    {
        __HAL_RCC_TIM16_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        TimIcGpio(GPIOA, GPIO_PIN_12, GPIO_AF1_TIM16);
    }
    else if (htim_ic->Instance == TIM17)
    {
        __HAL_RCC_TIM17_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        TimIcGpio(GPIOB, GPIO_PIN_9, GPIO_AF1_TIM17);
    }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim_ic)
{
    if (htim_ic->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
    }
    else if (htim_ic->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
    }
    else if (htim_ic->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
    }
    else if (htim_ic->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
        HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    }
    else if (htim_ic->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);
        HAL_NVIC_DisableIRQ(TIM5_IRQn);
    }
    else if (htim_ic->Instance == TIM8)
    {
        __HAL_RCC_TIM8_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
        HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
    }
    else if (htim_ic->Instance == TIM16)
    {
        __HAL_RCC_TIM16_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    }
    else if (htim_ic->Instance == TIM17)
    {
        __HAL_RCC_TIM17_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
        HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (huart->Instance == USART2)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_15;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_PIN_3;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
}
