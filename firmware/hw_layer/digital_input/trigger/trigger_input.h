/**
 * @file	trigger_input.h
 * @brief	Position sensor hardware layer
 *
 * @date Dec 30, 2012
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "engine.h"
#include "pin_repository.h"
#include "trigger_structure.h"
#include "trigger_central.h"


#define TRIGGER_BAIL_IF_DISABLED          \
    if (!engine->hwTriggerInputEnabled) { \
		return;                           \
	}

#define TRIGGER_BAIL_IF_SELF_STIM                                 \
    if (engine->directSelfStimulation) {                          \
		/* sensor noise + self-stim = loss of trigger sync */     \
		return;                                                   \
	}


#define TRIGGER_SUPPORTED_CHANNELS 2

void turnOnTriggerInputPins(DECLARE_ENGINE_PARAMETER_SIGNATURE);
void applyNewTriggerInputPins(DECLARE_ENGINE_PARAMETER_SIGNATURE);
void startTriggerInputPins(DECLARE_ENGINE_PARAMETER_SIGNATURE);
void stopTriggerInputPins(DECLARE_ENGINE_PARAMETER_SIGNATURE);

void stopTriggerDebugPins(DECLARE_ENGINE_PARAMETER_SIGNATURE);
void startTriggerDebugPins(DECLARE_ENGINE_PARAMETER_SIGNATURE);

#if HAL_USE_ADC
typedef adcsample_t triggerAdcSample_t;
#endif /* HAL_USE_ADC */

// This detector has 2 modes for low-RPM (ADC) and fast-RPM (EXTI)
enum triggerAdcMode_t {
	TRIGGER_ADC_NONE = 0,
	TRIGGER_ADC_ADC,
	TRIGGER_ADC_EXTI,
};

adc_channel_e getAdcChannelForTrigger(void);
void addAdcChannelForTrigger(void);
void triggerAdcCallback(triggerAdcSample_t value);

void setTriggerAdcMode(triggerAdcMode_t adcMode);
triggerAdcMode_t getTriggerAdcMode(void);
int getTriggerAdcModeCnt(void);
float getTriggerAdcThreshold(void);
void onTriggerChanged(efitick_t stamp, bool isPrimary, bool isRising);
