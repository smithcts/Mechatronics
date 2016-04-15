// Includes
#include "analog_in.h"
#include "stm32f4xx.h"

//*****************************************************************************
AnalogIn::AnalogIn(void)
{
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef       DMA_InitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;
    uint32_t              ComponentsUsed;

    // Enable clocks
    ComponentsUsed = RCC_AHB1Periph_DMA2  |
                     RCC_AHB1Periph_GPIOA |
                     RCC_AHB1Periph_GPIOC;
    RCC_AHB1PeriphClockCmd(ComponentsUsed, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // DMA2 Stream0 channel0 configuration
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adc_raw_values_;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 9;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    // Configure pins on GPIOs as analog input
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; // PA pins
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; // PC pins
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ADC Common Init
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;    // NOTE: all ADCs share this clock
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; // 72Mhz(APB2)/2=36Mhz
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADC1 Init
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising; // NA
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC3;         // NA
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                        // 12 bit right aligned
    ADC_InitStructure.ADC_NbrOfConversion = 9;                                    // 9 channels
    ADC_Init(ADC1, &ADC_InitStructure);

    // ADC1 regular channels configuration
    // Tconv = (480 +12)/36MHz, so 9 channels conversion scan at 8130 Hz
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 7, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 8, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 9, ADC_SampleTime_480Cycles);

    // Enable DMA request after last transfer (Single-ADC mode)
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    // Enable ADC1 DMA
    ADC_DMACmd(ADC1, ENABLE);

    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);

    //Start Conversion of regular channels
    ADC1->CR2 |= 0x40000000;

    //setupInterrupt();  // only for speed testing
}

//*****************************************************************************
void AnalogIn::getVoltages(float voltages[9])
{
    for (uint8_t i = 0; i < 9; i++)
    {
        voltages[i] = adc_raw_values_[i] * 3.3f / 0xFFF;
    }
}

//*****************************************************************************
void AnalogIn::setupInterrupt(void)
{
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
}

//*****************************************************************************
extern "C" void DMA2_Stream0_IRQHandler(void)
{
    static uint32_t count;

    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
        // Clear DMA Stream Transfer Complete interrupt pending bit
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

        count++;
        if (count > 81300)
        {
            count--;           // Set break point here to time 100 seconds
        }
    }
}
