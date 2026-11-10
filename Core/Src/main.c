/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32G474CEU6 — 8 pulse channels, variant B COMP protection
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "acquisition_math.h"
#include <string.h>

#define NUM_CHANNELS             8U
#define ADC_RANKS                4U
/* 512 scans @ 2 µs ≈ 1 ms of ring per ADC — headroom for 8 concurrent CTs. */
#define ADC_BURST_SAMPLES        512U
#define ADC_BUF_LEN              (ADC_BURST_SAMPLES * ADC_RANKS)
#define ADC_SCAN_PERIOD_US       2U
#define NVIC_PRIO_DMA_ADC        0U
#define NVIC_PRIO_TIM_BREAK      0U
#define NVIC_PRIO_TIM_CAPTURE    1U
#define NVIC_PRIO_USART          2U

#define VREF                     3.3f
#define R_SHUNT                  0.95f
#define ADC_MAX                  4095.0f
#define SIGNAL_TIMEOUT_MS        1000U
#define ACQ_FAULT_REARM_MS       SIGNAL_TIMEOUT_MS

#define MODBUS_SLAVE_ADDR        0x01U
#define MODBUS_FUNC_READ_HOLDING 0x03U
#define MODBUS_REG_SIGNAL_VALID    0U
#define MODBUS_REG_CURRENT_AVG     1U
#define MODBUS_REG_PULSE_WIDTH_US  2U
#define MODBUS_REG_ALARM           3U
#define MODBUS_REGS_PER_CH         4U
#define MODBUS_MAX_REGS            (NUM_CHANNELS * MODBUS_REGS_PER_CH)
#define MODBUS_RX_BUF_LEN        64U
#define MODBUS_TX_BUF_LEN        128U
#define MODBUS_FRAME_GAP_MS      4U
#define MODBUS_EX_ILLEGAL_FUNC   0x01U
#define MODBUS_EX_ILLEGAL_ADDR   0x02U
#define MODBUS_EX_ILLEGAL_VALUE  0x03U

#define OVERCURRENT_LIMIT_MA     2.5f
/* R_SHUNT is the calibrated transfer factor in V/mA, not bare ohms. */
#define DAC_OVERCURRENT_CODE     ((uint32_t)(OVERCURRENT_LIMIT_MA * R_SHUNT * 4096.0f / VREF + 0.5f))
#define ALARM_PULSE_WIDTH_US     300U
#define ENERGY_SAMPLE_PERIOD_MS  10U
/* External 100 Ω + C and GPIO Schmitt thresholds shorten the digital pulse.
   Trim restores generator width (300 µs in → ~273 µs raw → 300 µs reported). */
#define PULSE_WIDTH_SLEW_TRIM_US 27U

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
TIM_HandleTypeDef htim20;

UART_HandleTypeDef huart2;

typedef struct
{
    volatile uint8_t  capture_active;
    volatile uint8_t  capture_done;
    volatile uint32_t capture_sum;
    volatile uint16_t capture_peak;
    volatile uint32_t capture_count;
    volatile uint32_t live_sum;
    volatile uint32_t live_count;
    volatile uint16_t live_peak;
    volatile uint32_t first_scan;
    volatile uint32_t end_scan;
    volatile uint8_t end_scan_valid;
    volatile uint8_t deferred_fall;
    volatile uint32_t deferred_fall_time;
    volatile uint8_t pending_stop;
    volatile uint8_t measurement_valid;
    volatile uint8_t capture_error;
    volatile uint8_t have_rise;
    volatile uint32_t rise_tick;

    volatile uint16_t adc_result;
    volatile float    voltage;
    volatile float    current_mA;
    volatile float    current_peak_mA;
    volatile uint32_t adc_last_tick;

    volatile uint32_t pulse_period_ticks;
    volatile uint16_t pulse_width_us;

    volatile uint8_t  comp_overcurrent;
    volatile uint8_t  comp_overcurrent_latched;

    volatile uint8_t  expect_falling;
    volatile uint32_t ic_rise;
} Channel_t;

static Channel_t ch[NUM_CHANNELS];

static volatile uint16_t adc1_buf[ADC_BUF_LEN];
static volatile uint16_t adc2_buf[ADC_BUF_LEN];

static volatile uint8_t  adc1_burst_running = 0U;
static volatile uint8_t  adc2_burst_running = 0U;
static volatile uint8_t  tim20_running      = 0U;
static volatile uint8_t  adc1_active_mask   = 0U;
static volatile uint8_t  adc2_active_mask   = 0U;
static volatile uint8_t  pending_stop_mask  = 0U;
static volatile uint32_t dma_epochs[2];
static uint32_t consumed_scans[2];
static volatile uint32_t ic_epochs[2];
static volatile uint8_t protection_tripped;
static volatile uint8_t acquisition_fault;
static volatile uint32_t protection_last_high;
static volatile uint32_t acquisition_fault_tick;

/* Bench diagnostics: keep these visible in the debugger. */
volatile uint32_t adc_diag_start_count[2];
volatile uint32_t adc_diag_stop_count[2];
volatile uint32_t adc_diag_dma_half_count[2];
volatile uint32_t adc_diag_dma_full_count[2];
volatile uint32_t adc_diag_break_count;
volatile uint32_t adc_diag_last_error;
volatile uint32_t adc_diag_update_count[NUM_CHANNELS];
volatile uint8_t  adc_diag_reject_flags[NUM_CHANNELS];
/* First fault in the current fault epoch; cleared on soft-rearm. */
volatile uint32_t adc_diag_first_fault;
volatile uint32_t adc_diag_rearm_count;
volatile uint32_t adc_diag_block_flags;
volatile uint32_t adc_diag_fault_amp;
volatile uint32_t adc_diag_fault_status;
volatile uint32_t adc_diag_fault_adc_error;
volatile uint32_t adc_diag_fault_dma_error;
volatile uint32_t adc_diag_fault_adc_isr;
volatile uint32_t adc_diag_fault_dma_remaining;
volatile uint32_t adc_diag_max_pending[2];
volatile uint32_t adc_diag_produced_scans[2];
volatile uint32_t adc_diag_cursor_remaining[2];
volatile uint32_t adc_diag_cursor_tc[2];
volatile uint32_t adc_diag_fault_produced;
volatile uint32_t adc_diag_fault_consumed;
volatile uint32_t adc_diag_fault_epochs;
volatile uint32_t adc_diag_fault_cursor_remaining;
volatile uint32_t adc_diag_fault_cursor_tc;
volatile uint32_t adc_diag_tc_priority_count[2];

static volatile uint16_t energy1_raw = 0U;
static volatile uint16_t energy2_raw = 0U;
static volatile float    energy1_voltage = 0.0f;
static volatile float    energy2_voltage = 0.0f;
static uint32_t          energy_last_tick = 0U;

static volatile uint32_t modbus_last_byte_tick = 0U;
static volatile uint8_t  modbus_rx_len         = 0U;
static volatile uint8_t  modbus_rx_ready       = 0U;
static volatile uint8_t  modbus_tx_busy        = 0U;

static uint16_t modbus_regs[MODBUS_MAX_REGS];
static uint8_t  modbus_rx_buf[MODBUS_RX_BUF_LEN];
static uint8_t  modbus_tx_buf[MODBUS_TX_BUF_LEN];
static uint8_t  uart_rx_byte = 0U;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_DAC1_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM20_Init(void);
static void MX_USART2_UART_Init(void);

static uint16_t Modbus_CRC16(const uint8_t *buf, uint8_t len);
static void     Modbus_Process(void);
static void     Modbus_SendResponse(uint8_t len);
static void     Modbus_SendException(uint8_t slave, uint8_t func, uint8_t ex_code);
static void     Modbus_ResetRxState(void);

static void     RequestAdcStart(uint8_t idx);
static void     RequestAdcStop(uint8_t idx);
static void     ServicePendingAdcStops(void);
static void     ClearAcquisitionFaultState(void);
static void     StartTim20IfNeeded(void);
static void     StopTim20IfIdle(void);
static void     DrainAdc(uint8_t amp);
static void     ServiceProtection(void);
static void     TriggerPathSelfTest(void);
static void     UpdateMeasuredValues(uint8_t idx);
static uint8_t  ChannelAlarmActive(uint8_t i);
static void     UpdateModbusRegisters(void);
static void     UpdateAlarmOutput(void);
static void     LatchAmpOvercurrent(uint8_t amp);
static void     SampleEnergyAdc(void);
static void     ConfigPwmInput(TIM_HandleTypeDef *htim, uint32_t period,
                               uint8_t rise_on_ch2, IRQn_Type irqn);
static uint16_t PulseWidthFromTicks(uint32_t width_ticks);

static uint16_t PulseWidthFromTicks(uint32_t width_ticks)
{
    if (width_ticks == 0U) return 0U;
    if (width_ticks < (uint32_t)UINT16_MAX - PULSE_WIDTH_SLEW_TRIM_US)
        width_ticks += PULSE_WIDTH_SLEW_TRIM_US;
    else
        width_ticks = UINT16_MAX;
    return (uint16_t)width_ticks;
}

static void StartTim20IfNeeded(void)
{
    if (tim20_running || protection_tripped || acquisition_fault) return;

    __HAL_TIM_CLEAR_FLAG(&htim20, TIM_FLAG_BREAK);
    __HAL_TIM_SET_COUNTER(&htim20, 0U);
    __HAL_TIM_ENABLE_IT(&htim20, TIM_IT_BREAK);
    /* Arms the internal COMP1/COMP2 -> TIM20 BREAK matrix. No GPIO output is used. */
    __HAL_TIM_MOE_ENABLE(&htim20);
    if (HAL_TIM_Base_Start(&htim20) != HAL_OK) Error_Handler();
    tim20_running = 1U;
}

static void StopTim20IfIdle(void)
{
    if (adc1_burst_running || adc2_burst_running) return;
    if (!tim20_running) return;

    HAL_TIM_Base_Stop(&htim20);
    tim20_running = 0U;
}

static ADC_HandleTypeDef *AmpAdc(uint8_t amp)
{
    return amp ? &hadc2 : &hadc1;
}

static void RecordAcquisitionFault(uint32_t reason, uint8_t amp, uint32_t status)
{
    if (adc_diag_first_fault == 0U)
    {
        ADC_HandleTypeDef *adc = AmpAdc(amp);
        adc_diag_first_fault = reason;
        adc_diag_last_error = reason;
        adc_diag_fault_amp = amp;
        adc_diag_fault_status = status;
        adc_diag_fault_adc_error = adc->ErrorCode;
        adc_diag_fault_dma_error = adc->DMA_Handle->ErrorCode;
        adc_diag_fault_adc_isr = adc->Instance->ISR;
        adc_diag_fault_dma_remaining = __HAL_DMA_GET_COUNTER(adc->DMA_Handle);
        adc_diag_fault_produced = adc_diag_produced_scans[amp];
        adc_diag_fault_consumed = consumed_scans[amp];
        adc_diag_fault_epochs = dma_epochs[amp];
        adc_diag_fault_cursor_remaining = adc_diag_cursor_remaining[amp];
        adc_diag_fault_cursor_tc = adc_diag_cursor_tc[amp];
    }
    else
        adc_diag_last_error = reason;
    acquisition_fault = 1U;
    acquisition_fault_tick = HAL_GetTick();
    /* UPDATE/TRGO is not gated by MOE: stop the counter explicitly. */
    CLEAR_BIT(TIM20->CR1, TIM_CR1_CEN);
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
}

static void ResetDualEdgePolarity(void)
{
    TIM16->CCER &= ~TIM_CCER_CC1E;
    TIM17->CCER &= ~TIM_CCER_CC1E;
    TIM16->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    TIM17->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    TIM16->CCER |= TIM_CCER_CC1E;
    TIM17->CCER |= TIM_CCER_CC1E;
}

static void ClearAcquisitionFaultState(void)
{
    if (adc1_burst_running) (void)HAL_ADC_Stop_DMA(&hadc1);
    if (adc2_burst_running) (void)HAL_ADC_Stop_DMA(&hadc2);
    adc1_burst_running = adc2_burst_running = tim20_running = 0U;
    adc1_active_mask = adc2_active_mask = 0U;
    pending_stop_mask = 0U;
    dma_epochs[0] = dma_epochs[1] = 0U;
    consumed_scans[0] = consumed_scans[1] = 0U;
    for (uint8_t i = 0U; i < NUM_CHANNELS; i++)
    {
        ch[i].capture_active = ch[i].capture_done = ch[i].measurement_valid = 0U;
        ch[i].end_scan_valid = ch[i].deferred_fall = ch[i].pending_stop = 0U;
        ch[i].expect_falling = ch[i].have_rise = 0U;
        ch[i].capture_error = 0U;
    }
    ResetDualEdgePolarity();
    acquisition_fault = 0U;
    adc_diag_first_fault = 0U;
    adc_diag_block_flags = 0U;
    adc_diag_rearm_count++;
    if (!protection_tripped)
        HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
}

static uint64_t ProducedWords(uint8_t amp)
{
    DMA_HandleTypeDef *dma = AmpAdc(amp)->DMA_Handle;
    uint32_t before, after, remaining;
    do
    {
        before = __HAL_DMA_GET_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma));
        remaining = __HAL_DMA_GET_COUNTER(dma);
        after = __HAL_DMA_GET_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma));
    } while (before != after);
    uint64_t words = AcquisitionProducedWords(dma_epochs[amp], remaining,
        ADC_BUF_LEN, after != 0U);
    adc_diag_cursor_remaining[amp] = remaining;
    adc_diag_cursor_tc[amp] = after;
    adc_diag_produced_scans[amp] = (uint32_t)(words / ADC_RANKS);
    __DMB();
    return words;
}

static TIM_TypeDef *CaptureTimer(uint8_t idx)
{
    static TIM_TypeDef * const timers[NUM_CHANNELS] = {
        TIM2, TIM3, TIM4, TIM1, TIM8, TIM5, TIM17, TIM16
    };
    return timers[idx];
}

static void LatchCaptureEnd(uint8_t idx)
{
    Channel_t *c = &ch[idx];
    if (!c->capture_active || c->end_scan_valid) return;
    TIM_TypeDef *tim = CaptureTimer(idx);
    /* Snapshot DMA BEFORE CNT: read latency then trims a few extra samples
       inside the pulse rather than including samples after its falling edge.
       All input timers count microseconds. TIM20 issues one scan every 2 us. */
    uint32_t produced = (uint32_t)(ProducedWords(idx / ADC_RANKS) / ADC_RANKS);
    uint32_t falling = (idx < 2U || idx >= 6U) ? tim->CCR1 : tim->CCR2;
    uint32_t counter = tim->CNT;
    uint32_t late = counter - falling;
    if (tim != TIM2 && tim != TIM5) late = (uint16_t)late;
    /* Round conservatively and discard the scan straddling the edge. */
    uint32_t discard = late / ADC_SCAN_PERIOD_US +
        ((late % ADC_SCAN_PERIOD_US) != 0U) + 1U;
    int32_t available = (int32_t)(produced - c->first_scan);
    if (available <= 0)
        c->end_scan = c->first_scan;
    else if (discard >= (uint32_t)available)
        /* Prefer keeping in-pulse samples over publishing an empty window. */
        c->end_scan = produced;
    else
        c->end_scan = produced - discard;
    c->end_scan_valid = 1U;
}

static ACQUISITION_FAST void DrainAdc(uint8_t amp)
{
    if (acquisition_fault) return;
    if (!(amp ? adc2_burst_running : adc1_burst_running)) return;
    uint32_t produced = ProducedWords(amp) / ADC_RANKS;
    uint32_t pending = AcquisitionPending(produced, consumed_scans[amp]);
    if (pending > adc_diag_max_pending[amp]) adc_diag_max_pending[amp] = pending;
    if (pending > ADC_BURST_SAMPLES)
    {
        /* Never publish overwritten data as a valid measurement. */
        RecordAcquisitionFault(0xD1A00001U, amp, pending);
        return;
    }
    const volatile uint16_t *buf = amp ? adc2_buf : adc1_buf;
    AcquisitionSamples batch[ADC_RANKS] = {0};
    for (uint8_t slot = 0U; slot < ADC_RANKS; slot++)
    {
        Channel_t *c = &ch[amp * 4U + slot];
        if (!c->capture_active) continue;
        uint8_t idx = amp * ADC_RANKS + slot;
        uint32_t fall_flag = (idx < 2U || idx >= 6U) ? TIM_SR_CC1IF : TIM_SR_CC2IF;
        /* DMA can observe the falling edge before the TIM IRQ runs. */
        if (!c->end_scan_valid && (CaptureTimer(idx)->SR & fall_flag))
        {
            LatchCaptureEnd(idx);
            c->pending_stop = 1U;
            pending_stop_mask |= (uint8_t)(1U << idx);
        }
        uint32_t limit = produced;
        if (c->end_scan_valid && (int32_t)(limit - c->end_scan) > 0)
            limit = c->end_scan;
        if ((int32_t)(limit - consumed_scans[amp]) <= 0) continue;
        batch[slot] = AcquisitionAccumulate(buf, ADC_BURST_SAMPLES,
            ADC_RANKS, slot, consumed_scans[amp], limit, c->first_scan);
    }
    /* DMA runs during summation. Do not commit a batch overwritten mid-copy. */
    uint32_t end = (uint32_t)(ProducedWords(amp) / ADC_RANKS);
    if (AcquisitionPending(end, consumed_scans[amp]) > ADC_BURST_SAMPLES)
    {
        RecordAcquisitionFault(0xD1A00002U, amp,
            AcquisitionPending(end, consumed_scans[amp]));
        return;
    }
    for (uint8_t slot = 0U; slot < ADC_RANKS; slot++)
    {
        Channel_t *c = &ch[amp * 4U + slot];
        AcquisitionSamples samples = batch[slot];
        c->live_sum += samples.sum;
        c->live_count += samples.count;
        if (samples.peak > c->live_peak) c->live_peak = samples.peak;
    }
    consumed_scans[amp] = produced;
}

/* All acquisition IRQs use the same preemption priority. Account for TC before
   doing any expensive HT work: HAL normally chooses HT first when both flags
   are pending, leaving TC set throughout that callback (wraps can coalesce).
   One full callback drains both halves, so the stale HT callback is redundant.
   HAL still owns clearing TC, advancing its state and dispatching errors. */
void AcquisitionDmaIRQ(uint8_t amp)
{
    DMA_HandleTypeDef *dma = AmpAdc(amp)->DMA_Handle;
    uint32_t flags = dma->DmaBaseAddress->ISR;
    uint32_t enabled = dma->Instance->CCR;
    uint32_t ht = __HAL_DMA_GET_HT_FLAG_INDEX(dma);
    uint32_t tc = __HAL_DMA_GET_TC_FLAG_INDEX(dma);
    if ((flags & tc) && (enabled & DMA_IT_TC))
    {
        /* Mask HT until HAL has acknowledged TC and incremented dma_epochs.
           Merely clearing HT is insufficient: DMA can set it again between
           this snapshot and HAL's ISR read, making HAL choose HT again. */
        __HAL_DMA_DISABLE_IT(dma, DMA_IT_HT);
        if ((flags & ht) && (enabled & DMA_IT_HT))
        {
            __HAL_DMA_CLEAR_FLAG(dma, ht);
            adc_diag_dma_half_count[amp]++;
            adc_diag_tc_priority_count[amp]++;
        }
        HAL_DMA_IRQHandler(dma);
        /* Preserve any NEW HT event raised during the full-buffer callback. */
        if ((enabled & DMA_IT_HT) && dma->State == HAL_DMA_STATE_BUSY)
            __HAL_DMA_ENABLE_IT(dma, DMA_IT_HT);
        return;
    }
    HAL_DMA_IRQHandler(dma);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) { adc_diag_dma_half_count[0]++; DrainAdc(0U); }
    if (hadc->Instance == ADC2) { adc_diag_dma_half_count[1]++; DrainAdc(1U); }
    ServicePendingAdcStops();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* HAL has cleared TC. Advance the epoch before reading the DMA cursor;
       acquisition IRQs share one preemption priority, so this is serialized. */
    if (hadc->Instance == ADC1) { dma_epochs[0]++; adc_diag_dma_full_count[0]++; }
    if (hadc->Instance == ADC2) { dma_epochs[1]++; adc_diag_dma_full_count[1]++; }
    if (hadc->Instance == ADC1) DrainAdc(0U);
    if (hadc->Instance == ADC2) DrainAdc(1U);
    ServicePendingAdcStops();
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 || hadc->Instance == ADC2)
    {
        RecordAcquisitionFault(0xADC00001U, hadc->Instance == ADC2, hadc->ErrorCode);
    }
}

/* TIM capture IRQ path: no DrainAdc. Keep 8 sync rises under a few microseconds. */
static void RequestAdcStart(uint8_t idx)
{
    if (protection_tripped || acquisition_fault ||
        HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUT_LEVEL_HIGH ||
        HAL_COMP_GetOutputLevel(&hcomp2) == COMP_OUTPUT_LEVEL_HIGH)
    {
        adc_diag_block_flags =
            (protection_tripped ? 1U : 0U) |
            (acquisition_fault ? 2U : 0U) |
            ((HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUT_LEVEL_HIGH) ? 4U : 0U) |
            ((HAL_COMP_GetOutputLevel(&hcomp2) == COMP_OUTPUT_LEVEL_HIGH) ? 8U : 0U);
        if (!adc_diag_first_fault) adc_diag_last_error = 0xB10C0000U | adc_diag_block_flags;
        return;
    }
    adc_diag_block_flags = 0U;
    uint8_t amp = idx / 4U;
    volatile uint8_t *running = amp ? &adc2_burst_running : &adc1_burst_running;
    volatile uint8_t *active = amp ? &adc2_active_mask : &adc1_active_mask;

    /* Rare: next rise before deferred stop ran. Finalize this channel only —
       no DrainAdc here (keeps 8-sync TIM IRQs short). */
    if (ch[idx].pending_stop || ch[idx].capture_active)
    {
        if (ch[idx].capture_active)
        {
            ch[idx].capture_sum = ch[idx].live_sum;
            ch[idx].capture_count = ch[idx].live_count;
            ch[idx].capture_peak = ch[idx].live_peak;
            ch[idx].capture_done = 1U;
            ch[idx].capture_error = 1U;
            *active &= (uint8_t)~(1U << (idx % 4U));
        }
        ch[idx].capture_active = 0U;
        ch[idx].pending_stop = 0U;
        ch[idx].end_scan_valid = 0U;
        pending_stop_mask &= (uint8_t)~(1U << idx);
    }
    else
        ch[idx].capture_error = 0U;

    ch[idx].live_sum = ch[idx].live_count = 0U;
    ch[idx].live_peak = 0U;

    if (!*running)
    {
        dma_epochs[amp] = consumed_scans[amp] = 0U;
        HAL_StatusTypeDef status = HAL_ADC_Start_DMA(AmpAdc(amp),
                (uint32_t *)(amp ? adc2_buf : adc1_buf), ADC_BUF_LEN);
        if (status != HAL_OK)
        {
            RecordAcquisitionFault(0x57A20001U, amp, status);
            return;
        }
        *running = 1U;
        adc_diag_start_count[amp]++;
    }
    uint64_t words = ProducedWords(amp);
    ch[idx].first_scan = (uint32_t)((words + ADC_RANKS - 1U) / ADC_RANKS);
    ch[idx].end_scan_valid = 0U;
    ch[idx].capture_active = 1U;
    *active |= (uint8_t)(1U << (idx % 4U));
    StartTim20IfNeeded();
}

static void RequestAdcStop(uint8_t idx)
{
    if (!ch[idx].capture_active) return;
    LatchCaptureEnd(idx);
    ch[idx].pending_stop = 1U;
    pending_stop_mask |= (uint8_t)(1U << idx);
}

static void ServicePendingAdcStops(void)
{
    uint8_t mask = pending_stop_mask;
    if (mask == 0U || acquisition_fault) return;

    if (mask & 0x0FU) DrainAdc(0U);
    if (mask & 0xF0U) DrainAdc(1U);
    if (acquisition_fault) return;

    __disable_irq();
    mask = pending_stop_mask;
    pending_stop_mask = 0U;
    for (uint8_t idx = 0U; idx < NUM_CHANNELS; idx++)
    {
        if ((mask & (uint8_t)(1U << idx)) == 0U) continue;
        Channel_t *c = &ch[idx];
        c->pending_stop = 0U;
        if (!c->capture_active) continue;

        uint8_t amp = idx / 4U;
        volatile uint8_t *running = amp ? &adc2_burst_running : &adc1_burst_running;
        volatile uint8_t *active = amp ? &adc2_active_mask : &adc1_active_mask;

        c->capture_active = 0U;
        c->capture_sum = c->live_sum;
        c->capture_count = c->live_count;
        c->capture_peak = c->live_peak;
        c->capture_done = 1U;
        c->end_scan_valid = 0U;
        *active &= (uint8_t)~(1U << (idx % 4U));
        if (*active == 0U && *running)
        {
            *running = 0U;
            StopTim20IfIdle();
            HAL_StatusTypeDef status = HAL_ADC_Stop_DMA(AmpAdc(amp));
            if (status != HAL_OK)
            {
                __enable_irq();
                RecordAcquisitionFault(0x570F0000U, amp, status);
                return;
            }
            adc_diag_stop_count[amp]++;
        }
    }
    __enable_irq();
}

static void UpdateMeasuredValues(uint8_t idx)
{
    __disable_irq();
    if (!ch[idx].capture_done) { __enable_irq(); return; }
    ch[idx].capture_done   = 0U;
    uint32_t sum           = ch[idx].capture_sum;
    uint16_t peak          = ch[idx].capture_peak;
    uint32_t count         = ch[idx].capture_count;
    ch[idx].capture_sum    = 0U;
    ch[idx].capture_count  = 0U;
    ch[idx].capture_peak   = 0U;
    uint8_t error = ch[idx].capture_error;
    __enable_irq();
    uint8_t reject = (!count ? 1U : 0U) |
                     (error ? 2U : 0U) |
                     (protection_tripped ? 4U : 0U) |
                     (acquisition_fault ? 8U : 0U);
    adc_diag_reject_flags[idx] = reject;
    if (!count)
    {
        ch[idx].measurement_valid = 0U;
        return;
    }

    ch[idx].adc_result      = (uint16_t)(sum / count);
    ch[idx].voltage         = ((float)ch[idx].adc_result / ADC_MAX) * VREF;
    ch[idx].current_mA      = ch[idx].voltage / R_SHUNT;
    float peak_v            = ((float)peak / ADC_MAX) * VREF;
    ch[idx].current_peak_mA = peak_v / R_SHUNT;
    ch[idx].adc_last_tick   = HAL_GetTick();
    ch[idx].measurement_valid = (reject == 0U) ? 1U : 0U;
    adc_diag_update_count[idx]++;

    if (ch[idx].current_peak_mA > OVERCURRENT_LIMIT_MA)
        ch[idx].comp_overcurrent_latched = 1U;
    else if (!ch[idx].comp_overcurrent)
        ch[idx].comp_overcurrent_latched = 0U;
}

static uint8_t ChannelAlarmActive(uint8_t i)
{
    return (protection_tripped || acquisition_fault || ch[i].capture_error ||
            ch[i].comp_overcurrent || ch[i].comp_overcurrent_latched ||
            (ch[i].current_peak_mA > OVERCURRENT_LIMIT_MA) ||
            (ch[i].pulse_width_us > ALARM_PULSE_WIDTH_US)) ? 1U : 0U;
}

static void LatchAmpOvercurrent(uint8_t amp)
{
    protection_tripped = 1U;
    protection_last_high = HAL_GetTick();
    __HAL_TIM_MOE_DISABLE(&htim20);
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
    uint8_t base = (amp == 0U) ? 0U : 4U;
    for (uint8_t i = 0U; i < 4U; i++)
    {
        ch[base + i].comp_overcurrent = 1U;
        ch[base + i].comp_overcurrent_latched = 1U;
    }
}

static void UpdateModbusRegisters(void)
{
    for (uint8_t i = 0U; i < NUM_CHANNELS; i++)
    {
        uint32_t signal_age   = HAL_GetTick() - ch[i].adc_last_tick;
        uint8_t signal_valid = ch[i].measurement_valid && !protection_tripped &&
            !acquisition_fault && (signal_age <= SIGNAL_TIMEOUT_MS);

        if (signal_age > SIGNAL_TIMEOUT_MS)
        {
            ch[i].adc_result = 0U;
            ch[i].voltage = 0.0f;
            ch[i].current_mA = 0.0f;
            ch[i].current_peak_mA = 0.0f;
            ch[i].measurement_valid = 0U;
        }
        uint16_t base = (uint16_t)(i * MODBUS_REGS_PER_CH);
        modbus_regs[base + MODBUS_REG_SIGNAL_VALID]   = signal_valid;
        modbus_regs[base + MODBUS_REG_CURRENT_AVG]    = (uint16_t)(ch[i].current_mA * 100.0f);
        modbus_regs[base + MODBUS_REG_PULSE_WIDTH_US] = ch[i].pulse_width_us;
        modbus_regs[base + MODBUS_REG_ALARM]          = ChannelAlarmActive(i);
    }
}

static void UpdateAlarmOutput(void)
{
    uint8_t alarm = 0U;
    for (uint8_t i = 0U; i < NUM_CHANNELS; i++)
    {
        if (ChannelAlarmActive(i))
        {
            alarm = 1U;
            break;
        }
    }
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin,
                      alarm ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SampleEnergyAdc(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - energy_last_tick) < ENERGY_SAMPLE_PERIOD_MS) return;
    energy_last_tick = now;

    if (HAL_ADC_Start(&hadc3) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(&hadc3, 2U) == HAL_OK)
        {
            energy1_raw = (uint16_t)HAL_ADC_GetValue(&hadc3);
            energy1_voltage = ((float)energy1_raw / ADC_MAX) * VREF;
        }
        HAL_ADC_Stop(&hadc3);
    }

    if (HAL_ADC_Start(&hadc4) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(&hadc4, 2U) == HAL_OK)
        {
            energy2_raw = (uint16_t)HAL_ADC_GetValue(&hadc4);
            energy2_voltage = ((float)energy2_raw / ADC_MAX) * VREF;
        }
        HAL_ADC_Stop(&hadc4);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;
    modbus_tx_busy = 0U;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    if (modbus_rx_len < MODBUS_RX_BUF_LEN)
        modbus_rx_buf[modbus_rx_len++] = uart_rx_byte;

    modbus_last_byte_tick = HAL_GetTick();
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
    if (hcomp->Instance == COMP1)
        LatchAmpOvercurrent(0U);
    else if (hcomp->Instance == COMP2)
        LatchAmpOvercurrent(1U);
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM20) return;
    adc_diag_break_count++;
    protection_tripped = 1U;
    protection_last_high = HAL_GetTick();
    __HAL_TIM_MOE_DISABLE(&htim20);
    HAL_TIM_Base_Stop(&htim20);
    tim20_running = 0U;
    __HAL_TIM_DISABLE_IT(&htim20, TIM_IT_BREAK);
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
    if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUT_LEVEL_HIGH)
        LatchAmpOvercurrent(0U);
    if (HAL_COMP_GetOutputLevel(&hcomp2) == COMP_OUTPUT_LEVEL_HIGH)
        LatchAmpOvercurrent(1U);
}

static uint8_t PulseInputsHigh(void)
{
    return ((GPIOA->IDR & (GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12)) ||
            (GPIOB->IDR & (GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_9))) ? 1U : 0U;
}

static void ServiceProtection(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0U; i < NUM_CHANNELS; i++)
    {
        if (ch[i].capture_active && (now - ch[i].rise_tick > SIGNAL_TIMEOUT_MS))
            RecordAcquisitionFault(0x71AE0001U, i / 4U, i);
    }
    if (!(protection_tripped || acquisition_fault)) return;
    __disable_irq();
    __HAL_TIM_MOE_DISABLE(&htim20);
    HAL_TIM_Base_Stop(&htim20);
    if (adc1_burst_running) HAL_ADC_Stop_DMA(&hadc1);
    if (adc2_burst_running) HAL_ADC_Stop_DMA(&hadc2);
    adc1_burst_running = adc2_burst_running = tim20_running = 0U;
    adc1_active_mask = adc2_active_mask = 0U;
    pending_stop_mask = 0U;
    for (uint8_t i = 0U; i < NUM_CHANNELS; i++)
    {
        ch[i].capture_active = ch[i].capture_done = ch[i].measurement_valid = 0U;
        ch[i].end_scan_valid = ch[i].deferred_fall = ch[i].pending_stop = 0U;
    }
    if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUT_LEVEL_HIGH ||
        HAL_COMP_GetOutputLevel(&hcomp2) == COMP_OUTPUT_LEVEL_HIGH)
        protection_last_high = now;

    /* DMA overrun / stuck capture: soft-rearm after quiet period, no MCU reset. */
    if (acquisition_fault &&
        (now - acquisition_fault_tick >= ACQ_FAULT_REARM_MS) &&
        !PulseInputsHigh())
    {
        ClearAcquisitionFaultState();
    }

    /* Comparator trips rearm only after a full second healthy AND all inputs low. */
    if (!acquisition_fault && protection_tripped &&
        (now - protection_last_high >= SIGNAL_TIMEOUT_MS) &&
        !PulseInputsHigh())
    {
        for (uint8_t i = 0U; i < NUM_CHANNELS; i++)
        {
            ch[i].comp_overcurrent_latched = ch[i].comp_overcurrent = 0U;
            ch[i].expect_falling = ch[i].have_rise = 0U;
        }
        ResetDualEdgePolarity();
        __HAL_TIM_CLEAR_FLAG(&htim20, TIM_FLAG_BREAK);
        protection_tripped = 0U;
        __HAL_TIM_ENABLE_IT(&htim20, TIM_IT_BREAK);
        HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
    }
    __enable_irq();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM16) ic_epochs[0] += 0x10000U;
    if (htim->Instance == TIM17) ic_epochs[1] += 0x10000U;
}

void AcquisitionCaptureIRQ(TIM_HandleTypeDef *htim)
{
    /* Deferred fall path removed: LatchCaptureEnd + pending_stop from TIM/DMA. */
    HAL_TIM_IRQHandler(htim);
}

static void HandleSingleChannelIc(uint8_t idx, TIM_TypeDef *inst, TIM_HandleTypeDef *htim)
{
    uint16_t captured = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t ccr = AcquisitionCaptureTime(ic_epochs[inst == TIM16 ? 0U : 1U],
        captured, (uint16_t)inst->CNT, (inst->SR & TIM_SR_UIF) != 0U);

    if (!ch[idx].expect_falling)
    {
        /* Arm falling edge before any ADC work so a short pulse is not missed. */
        inst->CCER &= ~TIM_CCER_CC1E;
        inst->CCER |= TIM_CCER_CC1P;
        inst->CCER |= TIM_CCER_CC1E;
        ch[idx].pulse_period_ticks = ch[idx].have_rise ? ccr - ch[idx].ic_rise : 0U;
        ch[idx].have_rise = 1U;
        ch[idx].ic_rise = ccr;
        ch[idx].rise_tick = HAL_GetTick();
        ch[idx].expect_falling = 1U;
        RequestAdcStart(idx);
    }
    else
    {
        /* Re-arm rising edge first; ADC finalize is deferred out of this IRQ. */
        inst->CCER &= ~TIM_CCER_CC1E;
        inst->CCER &= (uint32_t)~TIM_CCER_CC1P;
        inst->CCER |= TIM_CCER_CC1E;
        uint32_t width = ccr - ch[idx].ic_rise;
        if (width > UINT16_MAX) width = UINT16_MAX;
        ch[idx].pulse_width_us = PulseWidthFromTicks(width);
        ch[idx].expect_falling = 0U;
        RequestAdcStop(idx);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint8_t idx        = 0xFFU;
    uint8_t rising_ch  = 0U;
    uint8_t falling_ch = 0U;
    uint8_t rise_is_ch2 = 0U;

    if      (htim->Instance == TIM2)  { idx = 0U; rising_ch = HAL_TIM_ACTIVE_CHANNEL_2; falling_ch = HAL_TIM_ACTIVE_CHANNEL_1; rise_is_ch2 = 1U; }
    else if (htim->Instance == TIM3)  { idx = 1U; rising_ch = HAL_TIM_ACTIVE_CHANNEL_2; falling_ch = HAL_TIM_ACTIVE_CHANNEL_1; rise_is_ch2 = 1U; }
    else if (htim->Instance == TIM4)  { idx = 2U; rising_ch = HAL_TIM_ACTIVE_CHANNEL_1; falling_ch = HAL_TIM_ACTIVE_CHANNEL_2; }
    else if (htim->Instance == TIM1)  { idx = 3U; rising_ch = HAL_TIM_ACTIVE_CHANNEL_1; falling_ch = HAL_TIM_ACTIVE_CHANNEL_2; }
    else if (htim->Instance == TIM8)  { idx = 4U; rising_ch = HAL_TIM_ACTIVE_CHANNEL_1; falling_ch = HAL_TIM_ACTIVE_CHANNEL_2; }
    else if (htim->Instance == TIM5)  { idx = 5U; rising_ch = HAL_TIM_ACTIVE_CHANNEL_1; falling_ch = HAL_TIM_ACTIVE_CHANNEL_2; }
    else if (htim->Instance == TIM17) { HandleSingleChannelIc(6U, TIM17, htim); return; }
    else if (htim->Instance == TIM16) { HandleSingleChannelIc(7U, TIM16, htim); return; }
    else return;

    if (htim->Channel == (HAL_TIM_ActiveChannel)rising_ch)
    {
        uint32_t period;
        if (rise_is_ch2)
        {
            period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        }
        else
        {
            period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        }
        ch[idx].pulse_period_ticks = period;
        ch[idx].rise_tick          = HAL_GetTick();
        RequestAdcStart(idx);
    }

    if (htim->Channel == (HAL_TIM_ActiveChannel)falling_ch)
    {
        uint32_t width = HAL_TIM_ReadCapturedValue(
            htim, rise_is_ch2 ? TIM_CHANNEL_1 : TIM_CHANNEL_2);
        if (width > UINT16_MAX || HAL_GetTick() - ch[idx].rise_tick >= 66U)
            width = UINT16_MAX;
        ch[idx].pulse_width_us = PulseWidthFromTicks(width);
        RequestAdcStop(idx);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    /* Halt the ADC trigger with the CPU; otherwise DMA wraps unobserved during
       a breakpoint and Resume trips acquisition_fault (soft-rearms after 1 s idle).
       External generator edges while halted are still not valid measurements. */
    __HAL_DBGMCU_FREEZE_TIM20();
    __HAL_DBGMCU_FREEZE_TIM1();
    __HAL_DBGMCU_FREEZE_TIM2();
    __HAL_DBGMCU_FREEZE_TIM3();
    __HAL_DBGMCU_FREEZE_TIM4();
    __HAL_DBGMCU_FREEZE_TIM5();
    __HAL_DBGMCU_FREEZE_TIM8();
    __HAL_DBGMCU_FREEZE_TIM16();
    __HAL_DBGMCU_FREEZE_TIM17();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_ADC4_Init();
    MX_DAC1_Init();
    MX_COMP1_Init();
    MX_COMP2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM1_Init();
    MX_TIM5_Init();
    MX_TIM8_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_TIM20_Init();
    MX_USART2_UART_Init();

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_OVERCURRENT_CODE);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_OVERCURRENT_CODE);

    TriggerPathSelfTest();
    HAL_COMP_Start(&hcomp1);
    HAL_COMP_Start(&hcomp2);

    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U);

    HAL_TIM_IC_Start_IT(&htim2,  TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2,  TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim3,  TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3,  TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim4,  TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim4,  TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim1,  TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1,  TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim8,  TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim8,  TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim5,  TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim5,  TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
    __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim17, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim17, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim20, TIM_IT_BREAK);

    while (1)
    {
        ch[0].comp_overcurrent = ch[1].comp_overcurrent =
        ch[2].comp_overcurrent = ch[3].comp_overcurrent =
            (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUT_LEVEL_HIGH) ? 1U : 0U;
        ch[4].comp_overcurrent = ch[5].comp_overcurrent =
        ch[6].comp_overcurrent = ch[7].comp_overcurrent =
            (HAL_COMP_GetOutputLevel(&hcomp2) == COMP_OUTPUT_LEVEL_HIGH) ? 1U : 0U;

        for (uint8_t i = 0U; i < NUM_CHANNELS; i++)
        {
            if (ch[i].comp_overcurrent)
                LatchAmpOvercurrent(i / 4U);
            UpdateMeasuredValues(i);
        }

        ServicePendingAdcStops();
        if (adc1_burst_running) DrainAdc(0U);
        if (adc2_burst_running) DrainAdc(1U);
        ServicePendingAdcStops();

        ServiceProtection();
        SampleEnergyAdc();
        UpdateModbusRegisters();
        UpdateAlarmOutput();

        if ((modbus_rx_len > 0U) &&
            (!modbus_rx_ready) &&
            (!modbus_tx_busy) &&
            ((HAL_GetTick() - modbus_last_byte_tick) >= MODBUS_FRAME_GAP_MS))
        {
            modbus_rx_ready = 1U;
        }

        if (modbus_rx_ready)
        {
            Modbus_Process();
            Modbus_ResetRxState();
        }
    }
}

static uint16_t Modbus_CRC16(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint8_t pos = 0U; pos < len; pos++)
    {
        crc ^= buf[pos];
        for (uint8_t i = 0U; i < 8U; i++)
            crc = (crc & 0x0001U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
    }
    return crc;
}

static void Modbus_ResetRxState(void)
{
    __disable_irq();
    modbus_rx_ready = 0U;
    modbus_rx_len   = 0U;
    __enable_irq();
}

static void Modbus_SendResponse(uint8_t len)
{
    if (modbus_tx_busy) return;
    modbus_tx_busy = 1U;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    HAL_UART_Transmit_IT(&huart2, modbus_tx_buf, len);
}

static void Modbus_SendException(uint8_t slave, uint8_t func, uint8_t ex_code)
{
    uint8_t resp_len = 0U;
    modbus_tx_buf[resp_len++] = slave;
    modbus_tx_buf[resp_len++] = (uint8_t)(func | 0x80U);
    modbus_tx_buf[resp_len++] = ex_code;
    uint16_t crc = Modbus_CRC16(modbus_tx_buf, resp_len);
    modbus_tx_buf[resp_len++] = (uint8_t)(crc & 0x00FFU);
    modbus_tx_buf[resp_len++] = (uint8_t)(crc >> 8U);
    Modbus_SendResponse(resp_len);
}

static void Modbus_Process(void)
{
    if (modbus_rx_len < 8U) return;

    uint16_t rx_crc_calc = Modbus_CRC16(modbus_rx_buf, (uint8_t)(modbus_rx_len - 2U));
    uint16_t rx_crc_recv = (uint16_t)modbus_rx_buf[modbus_rx_len - 2U] |
                           ((uint16_t)modbus_rx_buf[modbus_rx_len - 1U] << 8U);
    if (rx_crc_calc != rx_crc_recv) return;

    uint8_t  slave = modbus_rx_buf[0];
    uint8_t  func  = modbus_rx_buf[1];
    uint16_t start = ((uint16_t)modbus_rx_buf[2] << 8U) | modbus_rx_buf[3];
    uint16_t qty   = ((uint16_t)modbus_rx_buf[4] << 8U) | modbus_rx_buf[5];

    if (slave != MODBUS_SLAVE_ADDR) return;

    if (func != MODBUS_FUNC_READ_HOLDING)
    {
        Modbus_SendException(slave, func, MODBUS_EX_ILLEGAL_FUNC);
        return;
    }
    if ((qty == 0U) || (qty > 125U))
    {
        Modbus_SendException(slave, func, MODBUS_EX_ILLEGAL_VALUE);
        return;
    }
    if ((start >= MODBUS_MAX_REGS) || ((start + qty) > MODBUS_MAX_REGS))
    {
        Modbus_SendException(slave, func, MODBUS_EX_ILLEGAL_ADDR);
        return;
    }

    uint8_t byte_count = (uint8_t)(qty * 2U);
    if ((uint16_t)(3U + byte_count + 2U) > MODBUS_TX_BUF_LEN)
    {
        Modbus_SendException(slave, func, MODBUS_EX_ILLEGAL_VALUE);
        return;
    }

    uint8_t resp_len = 0U;
    modbus_tx_buf[resp_len++] = slave;
    modbus_tx_buf[resp_len++] = func;
    modbus_tx_buf[resp_len++] = byte_count;

    for (uint16_t i = 0U; i < qty; i++)
    {
        uint16_t v = modbus_regs[start + i];
        modbus_tx_buf[resp_len++] = (uint8_t)(v >> 8U);
        modbus_tx_buf[resp_len++] = (uint8_t)(v & 0x00FFU);
    }

    uint16_t crc = Modbus_CRC16(modbus_tx_buf, resp_len);
    modbus_tx_buf[resp_len++] = (uint8_t)(crc & 0x00FFU);
    modbus_tx_buf[resp_len++] = (uint8_t)(crc >> 8U);
    Modbus_SendResponse(resp_len);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef   multimode = {0};
    ADC_ChannelConfTypeDef sConfig   = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation      = 0;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = ADC_RANKS;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T20_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;

    sConfig.Channel = ADC_CHANNEL_1;  /* PA0 */
    sConfig.Rank    = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_3;  /* PA2 */
    sConfig.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_15; /* PB0 */
    sConfig.Rank    = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_11; /* PB12 */
    sConfig.Rank    = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    hdma_adc1.Instance                 = DMA1_Channel1;
    hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance                   = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation      = 0;
    hadc2.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hadc2.Init.LowPowerAutoWait      = DISABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.NbrOfConversion       = ADC_RANKS;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T20_TRGO;
    hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;

    sConfig.Channel = ADC_CHANNEL_13; /* PA5 */
    sConfig.Rank    = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_3;  /* PA6 */
    sConfig.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_4;  /* PA7 */
    sConfig.Rank    = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_14; /* PB11 */
    sConfig.Rank    = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();

    hdma_adc2.Instance                 = DMA1_Channel2;
    hdma_adc2.Init.Request             = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode                = DMA_CIRCULAR;
    hdma_adc2.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);
}

static void MX_ADC3_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc3.Instance                   = ADC3;
    hadc3.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc3.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc3.Init.GainCompensation      = 0;
    hadc3.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait      = DISABLE;
    hadc3.Init.ContinuousConvMode    = DISABLE;
    hadc3.Init.NbrOfConversion       = 1;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc3.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_5; /* PB13 */
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_ADC4_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc4.Instance                   = ADC4;
    hadc4.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc4.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc4.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc4.Init.GainCompensation      = 0;
    hadc4.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc4.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc4.Init.LowPowerAutoWait      = DISABLE;
    hadc4.Init.ContinuousConvMode    = DISABLE;
    hadc4.Init.NbrOfConversion       = 1;
    hadc4.Init.DiscontinuousConvMode = DISABLE;
    hadc4.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc4.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc4.Init.DMAContinuousRequests = DISABLE;
    hadc4.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc4.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc4) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_5; /* PB15 */
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_COMP1_Init(void)
{
    hcomp1.Instance          = COMP1;
    hcomp1.Init.InputPlus    = COMP_INPUT_PLUS_IO2; /* PB1 analog-max amp1 */
    hcomp1.Init.InputMinus   = COMP_INPUT_MINUS_DAC1_CH1;
    hcomp1.Init.OutputPol    = COMP_OUTPUTPOL_NONINVERTED;
    hcomp1.Init.Hysteresis   = COMP_HYSTERESIS_MEDIUM;
    hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
    hcomp1.Init.TriggerMode  = COMP_TRIGGERMODE_IT_RISING;
    if (HAL_COMP_Init(&hcomp1) != HAL_OK) Error_Handler();
}

static void MX_COMP2_Init(void)
{
    hcomp2.Instance          = COMP2;
    hcomp2.Init.InputPlus    = COMP_INPUT_PLUS_IO2; /* PA3 analog-max amp2 */
    hcomp2.Init.InputMinus   = COMP_INPUT_MINUS_DAC1_CH2;
    hcomp2.Init.OutputPol    = COMP_OUTPUTPOL_NONINVERTED;
    hcomp2.Init.Hysteresis   = COMP_HYSTERESIS_MEDIUM;
    hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
    hcomp2.Init.TriggerMode  = COMP_TRIGGERMODE_IT_RISING;
    if (HAL_COMP_Init(&hcomp2) != HAL_OK) Error_Handler();
}

static void MX_DAC1_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};

    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) Error_Handler();

    sConfig.DAC_HighFrequency           = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode       = DISABLE;
    sConfig.DAC_SignedFormat            = DISABLE;
    sConfig.DAC_SampleAndHold           = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger                 = DAC_TRIGGER_NONE;
    sConfig.DAC_Trigger2                = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer            = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
    sConfig.DAC_UserTrimming            = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) Error_Handler();
}

static void ConfigPwmInput(TIM_HandleTypeDef *htim, uint32_t period,
                           uint8_t rise_on_ch2, IRQn_Type irqn)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_SlaveConfigTypeDef  sSlaveConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_IC_InitTypeDef      sConfigIC          = {0};

    htim->Init.Prescaler         = 79;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = period;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (htim->Instance == TIM1 || htim->Instance == TIM8)
        htim->Init.RepetitionCounter = 0;
    if (HAL_TIM_IC_Init(htim) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK) Error_Handler();

    uint32_t rise_ch = rise_on_ch2 ? TIM_CHANNEL_2 : TIM_CHANNEL_1;
    uint32_t fall_ch = rise_on_ch2 ? TIM_CHANNEL_1 : TIM_CHANNEL_2;

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, rise_ch) != HAL_OK) Error_Handler();

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    if (HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, fall_ch) != HAL_OK) Error_Handler();

    sSlaveConfig.SlaveMode        = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger     = rise_on_ch2 ? TIM_TS_TI2FP2 : TIM_TS_TI1FP1;
    sSlaveConfig.TriggerPolarity  = TIM_TRIGGERPOLARITY_RISING;
    sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
    sSlaveConfig.TriggerFilter    = 0;
    if (HAL_TIM_SlaveConfigSynchro(htim, &sSlaveConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (htim->Instance == TIM1 || htim->Instance == TIM8)
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(irqn, NVIC_PRIO_TIM_CAPTURE, 0);
    HAL_NVIC_EnableIRQ(irqn);
}

static void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    ConfigPwmInput(&htim2, 0xFFFFFFFFU, 1U, TIM2_IRQn);
}

static void MX_TIM3_Init(void)
{
    htim3.Instance = TIM3;
    ConfigPwmInput(&htim3, 0xFFFFU, 1U, TIM3_IRQn);
}

static void MX_TIM4_Init(void)
{
    htim4.Instance = TIM4;
    ConfigPwmInput(&htim4, 0xFFFFU, 0U, TIM4_IRQn);
}

static void MX_TIM1_Init(void)
{
    htim1.Instance = TIM1;
    ConfigPwmInput(&htim1, 0xFFFFU, 0U, TIM1_CC_IRQn);
}

static void MX_TIM5_Init(void)
{
    htim5.Instance = TIM5;
    ConfigPwmInput(&htim5, 0xFFFFFFFFU, 0U, TIM5_IRQn);
}

static void MX_TIM8_Init(void)
{
    htim8.Instance = TIM8;
    ConfigPwmInput(&htim8, 0xFFFFU, 0U, TIM8_CC_IRQn);
}

static void MX_TIM16_Init(void)
{
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim16.Instance               = TIM16;
    htim16.Init.Prescaler         = 79;
    htim16.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim16.Init.Period            = 0xFFFFU;
    htim16.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim16) != HAL_OK) Error_Handler();

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_PRIO_TIM_CAPTURE, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}

static void MX_TIM17_Init(void)
{
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim17.Instance               = TIM17;
    htim17.Init.Prescaler         = 79;
    htim17.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim17.Init.Period            = 0xFFFFU;
    htim17.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.RepetitionCounter = 0;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim17) != HAL_OK) Error_Handler();

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim17, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, NVIC_PRIO_TIM_CAPTURE, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
}

static void MX_TIM20_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};

    htim20.Instance               = TIM20;
    htim20.Init.Prescaler         = 0;
    htim20.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim20.Init.Period            = 159; /* 2 us, margin over 1.5 us ADC scan */
    htim20.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim20.Init.RepetitionCounter = 0;
    htim20.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim20) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_UPDATE;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim20, &sMasterConfig) != HAL_OK) Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim20, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    sBreakInputConfig.Source   = TIM_BREAKINPUTSOURCE_BKIN;
    sBreakInputConfig.Enable   = TIM_BREAKINPUTSOURCE_DISABLE;
    sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
    if (HAL_TIMEx_ConfigBreakInput(&htim20, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK) Error_Handler();

    sBreakInputConfig.Source   = TIM_BREAKINPUTSOURCE_COMP1;
    sBreakInputConfig.Enable   = TIM_BREAKINPUTSOURCE_ENABLE;
    sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
    if (HAL_TIMEx_ConfigBreakInput(&htim20, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK) Error_Handler();

    sBreakInputConfig.Source   = TIM_BREAKINPUTSOURCE_COMP2;
    sBreakInputConfig.Enable   = TIM_BREAKINPUTSOURCE_ENABLE;
    sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
    if (HAL_TIMEx_ConfigBreakInput(&htim20, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(TIM20_BRK_IRQn, NVIC_PRIO_TIM_BREAK, 0);
    HAL_NVIC_EnableIRQ(TIM20_BRK_IRQn);
}

/* Startup acquisition test. TIM20 UPDATE is the ADC trigger. Break response is
   handled by the highest-priority TIM20_BRK IRQ; the comparator itself remains
   an asynchronous analog path. */
static void TriggerPathSelfTest(void)
{
    __HAL_TIM_DISABLE_IT(&htim20, TIM_IT_BREAK);
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, ADC_BUF_LEN) != HAL_OK ||
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, ADC_BUF_LEN) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_Base_Start(&htim20) != HAL_OK) Error_Handler();
    HAL_Delay(1U);
    uint32_t a = __HAL_DMA_GET_COUNTER(&hdma_adc1);
    uint32_t b = __HAL_DMA_GET_COUNTER(&hdma_adc2);
    if ((a == ADC_BUF_LEN && dma_epochs[0] == 0U) ||
        (b == ADC_BUF_LEN && dma_epochs[1] == 0U))
    {
        RecordAcquisitionFault(0x5E1F0001U,
            (a == ADC_BUF_LEN && dma_epochs[0] == 0U) ? 0U : 1U, 0U);
    }
    HAL_TIM_Base_Stop(&htim20);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    dma_epochs[0] = dma_epochs[1] = 0U;
    __HAL_TIM_CLEAR_FLAG(&htim20, TIM_FLAG_BREAK);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(USART2_IRQn, NVIC_PRIO_USART, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA must preempt capture IRQs so DrainAdc keeps the ring from wrapping. */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_PRIO_DMA_ADC, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_PRIO_DMA_ADC, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = ALARM_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ALARM_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    /* GPIO clocks are already enabled for initialization errors after MX_GPIO. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
