/**
 * @file        stm32_adc_v4.cpp
 * @brief       Port implementation for the STM32 "v4" ADC found on the STM32H7
 *
 * @date February 25, 2021
 * @author Matthew Kennedy, (c) 2021
 */

#include "global.h"
#include "hal.h"
#include "mpu_util.h"
#include "adc_inputs.h"


#ifndef ADC_SLOW_DEVICE
#ifdef EFI_USE_ADC3_SLOW_DEVICE
#define ADC_SLOW_DEVICE ADCD3
#else
#define ADC_SLOW_DEVICE ADCD1
#endif /* EFI_USE_ADC3_SLOW_DEVICE */
#endif /* ADC_SLOW_DEVICE */

void portInitAdc() {
#ifndef EFI_USE_ONLY_FAST_ADC
	// Init slow ADC
	adcStart(&ADC_SLOW_DEVICE, NULL);
#endif /* EFI_USE_ONLY_FAST_ADC */

#if EFI_USE_FAST_ADC
	// Init fast ADC
	adcStart(&ADC_FAST_DEVICE, NULL);
#endif /* EFI_USE_FAST_ADC */

	// Connect the analog switches between {PA0_C, PA1_C, PC2_C, PC3_C} and their non-C counterparts
	// This lets us use normal (non-direct) analog on those channels
	SYSCFG->PMCR &= ~(SYSCFG_PMCR_PA0SO | SYSCFG_PMCR_PA1SO | SYSCFG_PMCR_PC2SO | SYSCFG_PMCR_PC3SO);
}

float getMcuTemperature() {
	// Ugh, internal temp sensor is wired to ADC3, which makes it nearly useless on the H7.
	return 0;
}

// ADC Clock is 25MHz
// 32.5 sampling + 8.5 conversion = 41 cycles per sample total
// 16 channels * 16x oversample = 256 samples per batch
// (41 * 256) / 25MHz -> 419 microseconds to sample all channels
#define ADC_SAMPLING_SLOW ADC_SMPR_SMP_32P5

// 1e6*(16.5+8.5)*(16*4)/25MHz = 64 usec
#define ADC_SAMPLING_FAST ADC_SMPR_SMP_16P5

// Sample the 16 channels that line up with the STM32F4/F7
#if EFI_USE_ADC3_SLOW_DEVICE
constexpr size_t slowChannelCount = 8;
#else
constexpr size_t slowChannelCount = 16;
#endif /* EFI_USE_ADC3_SLOW_DEVICE */

// Conversion group for slow channels
// This simply samples every channel in sequence
static constexpr ADCConversionGroup convGroupSlow = {
	.circular			= FALSE,
	.num_channels		= slowChannelCount,
	.end_cb				= nullptr,
	.error_cb			= nullptr,
	.cfgr				= 0,
	.cfgr2				= 	15 << ADC_CFGR2_OVSR_Pos |	// Oversample by 16x (register contains N-1)
							4 << ADC_CFGR2_OVSS_Pos |	// shift the result right 4 bits to make a 16 bit result
							ADC_CFGR2_ROVSE,			// Enable oversampling
	.ccr				= 0,
	.pcsel				= 0xFFFFFFFF, // enable analog switches on all channels
	// Thresholds aren't used
	.ltr1 = 0, .htr1 = 0, .ltr2 = 0, .htr2 = 0, .ltr3 = 0, .htr3 = 0,
//	.awd2cr				= 0,
//	.awd3cr				= 0,
	.smpr = {
		// Configure all channels to use ADC_SAMPLING_SLOW time
#if EFI_USE_ADC3_SLOW_DEVICE
		ADC_SMPR1_SMP_AN0(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN1(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN2(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN3(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN4(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN5(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN6(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN7(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN8(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN9(ADC_SAMPLING_SLOW),
		ADC_SMPR2_SMP_AN10(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN11(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN12(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN13(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN14(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN15(ADC_SAMPLING_SLOW)
#else
		ADC_SMPR1_SMP_AN0(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN1(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN2(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN3(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN4(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN5(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN6(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN7(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN8(ADC_SAMPLING_SLOW) |
		ADC_SMPR1_SMP_AN9(ADC_SAMPLING_SLOW),
		ADC_SMPR2_SMP_AN10(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN11(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN12(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN13(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN14(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN15(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN16(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN17(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN18(ADC_SAMPLING_SLOW) |
		ADC_SMPR2_SMP_AN19(ADC_SAMPLING_SLOW)
#endif /* EFI_USE_ADC3_SLOW_DEVICE */
	},
	.sqr = {
		// The seemingly insane values here exist to put the values
		// in the buffer in the same order as the ADCv2 (F4/F7) ADC
#if EFI_USE_ADC3_SLOW_DEVICE
		ADC_SQR1_SQ1_N(ADC_CHANNEL_IN2) |	// PF9
		ADC_SQR1_SQ2_N(ADC_CHANNEL_IN3) |	// PF7
		ADC_SQR1_SQ3_N(ADC_CHANNEL_IN4) |	// PF5
		ADC_SQR1_SQ4_N(ADC_CHANNEL_IN5),	// PF3
		ADC_SQR2_SQ5_N(ADC_CHANNEL_IN6) |	// PF10
		ADC_SQR2_SQ6_N(ADC_CHANNEL_IN7) |	// PF8
		ADC_SQR2_SQ7_N(ADC_CHANNEL_IN8) |	// PF6
		ADC_SQR2_SQ8_N(ADC_CHANNEL_IN9),	// PF4
		0,
		0
#else
		ADC_SQR1_SQ1_N(16) |	// PA0 (aka PA0_C)
		ADC_SQR1_SQ2_N(17) |	// PA1 (aka PA1_C)
		ADC_SQR1_SQ3_N(14) |	// PA2
		ADC_SQR1_SQ4_N(15),		// PA3
		ADC_SQR2_SQ5_N(18) |	// PA4
		ADC_SQR2_SQ6_N(19) |	// PA5
		ADC_SQR2_SQ7_N(3) |		// PA6
		ADC_SQR2_SQ8_N(7) |		// PA7
		ADC_SQR2_SQ9_N(9),		// PB0
		ADC_SQR3_SQ10_N(5) |	// PB1
		ADC_SQR3_SQ11_N(10) |	// PC0
		ADC_SQR3_SQ12_N(11) |	// PC1
		ADC_SQR3_SQ13_N(12) |	// PC2 (aka PC2_C)
		ADC_SQR3_SQ14_N(13),	// PC3 (aka PC3_C)
		ADC_SQR4_SQ15_N(4) |	// PC4
		ADC_SQR4_SQ16_N(8)		// PC5
#endif /* EFI_USE_ADC3_SLOW_DEVICE */
	},
};

bool readSlowAnalogInputs(adcsample_t* convertedSamples) {
#if EFI_USE_ONLY_FAST_ADC
	return true;
#endif /* EFI_USE_ONLY_FAST_ADC */
	// Oversampling and right-shift happen in hardware, so we can sample directly to the output buffer
	msg_t result = adcConvert(&ADC_SLOW_DEVICE, &convGroupSlow, convertedSamples, 1);

	// Return true if OK
	return result == MSG_OK;
}

#if EFI_USE_FAST_ADC
static ADCConversionGroup adcgrpcfgFast = {
	.circular			= FALSE,
	.num_channels		= 0,
	.end_cb				= nullptr,
	.error_cb			= nullptr,
	.cfgr				= 0,
	.cfgr2				= 3 << ADC_CFGR2_OVSR_Pos |	// Oversample by 4x (register contains N-1)
							2 << ADC_CFGR2_OVSS_Pos |	// shift the result right 2 bits to make a 16 bit result
							ADC_CFGR2_ROVSE,			// Enable oversampling
	.ccr				= 0,
	.pcsel				= 0xFFFFFFFF, // enable analog switches on all channels
	// Thresholds aren't used
	.ltr1 = 0, .htr1 = 0, .ltr2 = 0, .htr2 = 0, .ltr3 = 0, .htr3 = 0,
//	.awd2cr				= 0U,
//	.awd3cr				= 0U,
	.smpr = {
		// Configure all channels to use ADC_SAMPLING_FAST time
		ADC_SMPR1_SMP_AN0(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN1(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN2(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN3(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN4(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN5(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN6(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN7(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN8(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN9(ADC_SAMPLING_FAST),
		ADC_SMPR2_SMP_AN10(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN11(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN12(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN13(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN14(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN15(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN16(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN17(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN18(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN19(ADC_SAMPLING_FAST)
	},
	.sqr = {
#if EFI_USE_ONLY_FAST_ADC
		// The seemingly insane values here exist to put the values
		// in the buffer in the same order as the ADCv2 (F4/F7) ADC
		ADC_SQR1_SQ1_N(16) |	// PA0 (aka PA0_C)
		ADC_SQR1_SQ2_N(17) |	// PA1 (aka PA1_C)
		ADC_SQR1_SQ3_N(14) |	// PA2
		ADC_SQR1_SQ4_N(15),		// PA3
		ADC_SQR2_SQ5_N(18) |	// PA4
		ADC_SQR2_SQ6_N(19) |	// PA5
		ADC_SQR2_SQ7_N(3) |		// PA6
		ADC_SQR2_SQ8_N(7) |		// PA7
		ADC_SQR2_SQ9_N(9),		// PB0
		ADC_SQR3_SQ10_N(5) |	// PB1
		ADC_SQR3_SQ11_N(10) |	// PC0
		ADC_SQR3_SQ12_N(11) |	// PC1
		ADC_SQR3_SQ13_N(12) |	// PC2 (aka PC2_C)
		ADC_SQR3_SQ14_N(13),	// PC3 (aka PC3_C)
		ADC_SQR4_SQ15_N(4) |	// PC4
		ADC_SQR4_SQ16_N(8)		// PC5
#else
		0, 0, 0, 0
#endif /* EFI_USE_ONLY_FAST_ADC */
	},
};

void portInitFastAdc(adccallback_t endCallback) {
	adcgrpcfgFast.end_cb = endCallback;
}

ADCConversionGroup *getFastAdcConversionGroup() {
	return &adcgrpcfgFast;
}

void enableAdcChannel(ADCConversionGroup *hwConfig, adc_channel_e hwChannel, int logicChannel) {
	// we use the same order as the ADCv2 (F4/F7) ADC
	static const size_t channelMappingV2[EFI_ADC_LAST_CHANNEL] = { 16, 17, 14, 15, 18, 19, 3, 7, 9, 5, 10, 11, 12, 13, 4, 8 };

	/* TODO: following is correct for STM32 ADC1/2.
	 * ADC3 has another input to gpio mapping
	 * and should be handled separately */
	size_t channelAdcIndex = channelMappingV2[hwChannel - EFI_ADC_0];

	if (logicChannel < 4) {
		hwConfig->sqr[0] |= channelAdcIndex << (6 * (logicChannel + 1));
	} else if (logicChannel < 9) {
		hwConfig->sqr[1] |= channelAdcIndex << (6 * (logicChannel - 4));
	} else if (logicChannel < 14) {
		hwConfig->sqr[2] |= channelAdcIndex << (6 * (logicChannel - 9));
	} else if (logicChannel < 16) {
		hwConfig->sqr[3] |= channelAdcIndex << (6 * (logicChannel - 14));
	}
}

#endif /* EFI_USE_FAST_ADC */
