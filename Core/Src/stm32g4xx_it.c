/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32g4xx_it.h"

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim20;
extern UART_HandleTypeDef huart2;

void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void DMA1_Channel1_IRQHandler(void)
{
    AcquisitionDmaIRQ(0U);
}

void DMA1_Channel2_IRQHandler(void)
{
    AcquisitionDmaIRQ(1U);
}

void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
    HAL_ADC_IRQHandler(&hadc2);
}

void ADC3_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc3);
}

void ADC4_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc4);
}

void TIM1_CC_IRQHandler(void)
{
    AcquisitionCaptureIRQ(&htim1);
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    AcquisitionCaptureIRQ(&htim16);
}

void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    AcquisitionCaptureIRQ(&htim17);
}

void TIM2_IRQHandler(void)
{
    AcquisitionCaptureIRQ(&htim2);
}

void TIM3_IRQHandler(void)
{
    AcquisitionCaptureIRQ(&htim3);
}

void TIM4_IRQHandler(void)
{
    AcquisitionCaptureIRQ(&htim4);
}

void TIM5_IRQHandler(void)
{
    AcquisitionCaptureIRQ(&htim5);
}

void TIM8_CC_IRQHandler(void)
{
    AcquisitionCaptureIRQ(&htim8);
}

void TIM20_BRK_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim20);
}

void COMP1_2_3_IRQHandler(void)
{
    HAL_COMP_IRQHandler(&hcomp1);
    HAL_COMP_IRQHandler(&hcomp2);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
