#include "acquisition.h"
#include "scope_config.h"
#include "adc.h"
#include "tim.h"
#include "main.h"
#include <string.h>

_Static_assert((SCOPE_HISTORY_SIZE & (SCOPE_HISTORY_SIZE - 1)) == 0,
               "DMA history must be a power of two");

/* DMA writes SRAM. CCM is only accessed by the CPU after copying a completed half. */
static uint32_t dma_samples[2][SCOPE_DMA_HALF];
static uint32_t scratch[SCOPE_DMA_HALF];
static uint32_t history[SCOPE_HISTORY_SIZE] __attribute__((section(".ccmram")));
static volatile uint32_t completed;
static volatile uint32_t errors;
static uint32_t consumed, dropped, end, available;

/* 启动双规则同时 ADC、循环 DMA 和 TIM2 采样时钟。 */
void Acquisition_Start(void)
{
    uint32_t timer_hz = HAL_RCC_GetPCLK1Freq() * 2U;
    uint32_t divisor = (TIM2->PSC + 1U) * (TIM2->ARR + 1U);
    if (timer_hz / divisor != SCOPE_SAMPLE_HZ ||
        HAL_ADC_Start(&hadc2) != HAL_OK ||
        HAL_ADCEx_MultiModeStart_DMA(&hadc1, dma_samples[0], 2 * SCOPE_DMA_HALF) != HAL_OK ||
        HAL_TIM_Base_Start(&htim2) != HAL_OK) {
        Error_Handler();
    }
}

/* 半传输回调只发布块号。 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc)
{
    if (adc == &hadc1) ++completed;
}

/* 全传输回调只发布块号。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
    if (adc == &hadc1) ++completed;
}

/* 累计 ADC 或采样 DMA 错误。 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc)
{
    if (adc == &hadc1) ++errors;
}

/* 复制已完成的半块，发现覆盖时记录丢块并标记断点。 */
bool Acquisition_Poll(SampleBlock *block)
{
    uint32_t epoch = completed;
    if (epoch == consumed) return false;

    memcpy(scratch, dma_samples[(epoch - 1U) & 1U], sizeof scratch);
    __DMB();
    if (completed != epoch) return false;

    bool gap = epoch - consumed > 1U;
    if (gap) {
        dropped += epoch - consumed - 1U;
        available = 0;
    }
    consumed = epoch;
    uint32_t first = (epoch - 1U) * SCOPE_DMA_HALF;
    for (unsigned i = 0; i < SCOPE_DMA_HALF; ++i) {
        history[(first + i) & (SCOPE_HISTORY_SIZE - 1U)] = scratch[i];
    }
    end = first + SCOPE_DMA_HALF;
    if (available < SCOPE_HISTORY_SIZE) available += SCOPE_DMA_HALF;
    *block = (SampleBlock){scratch, first, SCOPE_DMA_HALF, gap};
    return true;
}

/* 返回下一点的样本序号。 */
uint32_t Acquisition_End(void) { return end; }
/* 返回当前连续保留的样本数。 */
uint32_t Acquisition_Count(void) { return available; }
/* 按绝对序号读取历史缓冲中的一对样本。 */
uint32_t Acquisition_Sample(uint32_t index) { return history[index & (SCOPE_HISTORY_SIZE - 1U)]; }
/* 返回已被覆盖的 DMA 半块数。 */
uint32_t Acquisition_Dropped(void) { return dropped; }
/* 返回 ADC 错误累计数。 */
uint32_t Acquisition_Errors(void) { return errors; }
