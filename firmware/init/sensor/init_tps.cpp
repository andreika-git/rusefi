#include "adc_subscription.h"
#include "engine.h"
#include "error_handling.h"
#include "global.h"
#include "functional_sensor.h"
#include "redundant_sensor.h"
#include "proxy_sensor.h"
#include "linear_func.h"
#include "tps.h"

EXTERN_ENGINE;

LinearFunc tpsFunc1p(TPS_TS_CONVERSION);
LinearFunc tpsFunc1s(TPS_TS_CONVERSION);
LinearFunc tpsFunc2p(TPS_TS_CONVERSION);
LinearFunc tpsFunc2s(TPS_TS_CONVERSION);

FunctionalSensor tpsSens1p(SensorType::Tps1Primary, MS2NT(10));
FunctionalSensor tpsSens1s(SensorType::Tps1Secondary, MS2NT(10));
FunctionalSensor tpsSens2p(SensorType::Tps2Primary, MS2NT(10));
FunctionalSensor tpsSens2s(SensorType::Tps2Secondary, MS2NT(10));

RedundantSensor tps1(SensorType::Tps1, SensorType::Tps1Primary, SensorType::Tps1Secondary);
RedundantSensor tps2(SensorType::Tps2, SensorType::Tps2Primary, SensorType::Tps2Secondary);

LinearFunc pedalFuncPrimary;
LinearFunc pedalFuncSecondary;
FunctionalSensor pedalSensorPrimary(SensorType::AcceleratorPedalPrimary, MS2NT(10));
FunctionalSensor pedalSensorSecondary(SensorType::AcceleratorPedalSecondary, MS2NT(10));

RedundantSensor pedal(SensorType::AcceleratorPedal, SensorType::AcceleratorPedalPrimary, SensorType::AcceleratorPedalSecondary);

// This sensor indicates the driver's throttle intent - Pedal if we have one, TPS if not.
ProxySensor driverIntent(SensorType::DriverThrottleIntent);

static void configureTps(LinearFunc& func, float closed, float open, float min, float max) {
	func.configure(
		closed, 0,
		open, 100, 
		min, max
	);
}

static bool initTpsFunc(LinearFunc& func, FunctionalSensor& sensor, adc_channel_e channel, float closed, float open, float min, float max) {
	// Only register if we have a sensor
	if (channel == EFI_ADC_NONE) {
		return false;
	}

	configureTps(func, closed, open, min, max);

	sensor.setFunction(func);

	AdcSubscription::SubscribeSensor(sensor, channel);

	if (!sensor.Register()) {
		firmwareError(CUSTOM_INVALID_TPS_SETTING, "Duplicate registration for sensor \"%s\"", sensor.getSensorName());
		return false;
	}

	return true;
}

static void initTpsFuncAndRedund(RedundantSensor& redund, LinearFunc& func, FunctionalSensor& sensor, adc_channel_e channel, float closed, float open, float min, float max) {
	bool hasSecond = initTpsFunc(func, sensor, channel, closed, open, min, max);

	redund.configure(5.0f, !hasSecond);

	if (!redund.Register()) {
		firmwareError(CUSTOM_INVALID_TPS_SETTING, "Duplicate registration for sensor \"%s\"", redund.getSensorName());
	}
}

void initTps(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	float min = CONFIG(tpsErrorDetectionTooLow);
	float max = CONFIG(tpsErrorDetectionTooHigh);

	initTpsFunc(tpsFunc1p, tpsSens1p, CONFIG(tps1_1AdcChannel), CONFIG(tpsMin), CONFIG(tpsMax), min, max);
	initTpsFuncAndRedund(tps1, tpsFunc1s, tpsSens1s, CONFIG(tps1_2AdcChannel), CONFIG(tps1SecondaryMin), CONFIG(tps1SecondaryMax), min, max);
	initTpsFunc(tpsFunc2p, tpsSens2p, CONFIG(tps2_1AdcChannel), CONFIG(tps2Min), CONFIG(tps2Max), min, max);
	initTpsFuncAndRedund(tps2, tpsFunc2s, tpsSens2s, CONFIG(tps2_2AdcChannel), CONFIG(tps2SecondaryMin), CONFIG(tps2SecondaryMax), min, max);
	initTpsFunc(pedalFuncPrimary, pedalSensorPrimary, CONFIG(throttlePedalPositionAdcChannel), CONFIG(throttlePedalUpVoltage), CONFIG(throttlePedalWOTVoltage), min, max);
	initTpsFuncAndRedund(pedal, pedalFuncSecondary, pedalSensorSecondary, CONFIG(throttlePedalPositionSecondAdcChannel), CONFIG(throttlePedalSecondaryUpVoltage), CONFIG(throttlePedalSecondaryWOTVoltage), min, max);

	// Route the pedal or TPS to driverIntent as appropriate
	if (CONFIG(throttlePedalPositionAdcChannel) != EFI_ADC_NONE) {
		driverIntent.setProxiedSensor(SensorType::AcceleratorPedal);
	} else {
		driverIntent.setProxiedSensor(SensorType::Tps1);
	}

	if (!driverIntent.Register()) {
		firmwareError(CUSTOM_INVALID_TPS_SETTING, "Duplicate registration for driver acc intent sensor");
	}
}

void reconfigureTps(DECLARE_ENGINE_PARAMETER_SIGNATURE) {
	float min = CONFIG(tpsErrorDetectionTooLow);
	float max = CONFIG(tpsErrorDetectionTooHigh);

	configureTps(tpsFunc1p, CONFIG(tpsMin), CONFIG(tpsMax), min, max);
	configureTps(tpsFunc1s, CONFIG(tps1SecondaryMin), CONFIG(tps1SecondaryMax), min, max);
	configureTps(tpsFunc2p, CONFIG(tps2Min), CONFIG(tps2Max), min, max);
	configureTps(tpsFunc2s, CONFIG(tps2SecondaryMin), CONFIG(tps2SecondaryMax), min, max);

	configureTps(pedalFuncPrimary, CONFIG(throttlePedalUpVoltage), CONFIG(throttlePedalWOTVoltage), min, max);
	configureTps(pedalFuncSecondary, CONFIG(throttlePedalSecondaryUpVoltage), CONFIG(throttlePedalSecondaryWOTVoltage), min, max);
}
