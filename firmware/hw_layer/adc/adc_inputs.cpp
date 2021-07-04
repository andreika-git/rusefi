/**
 * @file	adc_inputs.cpp
 * @brief	Low level ADC code
 *
 * rusEfi uses two ADC devices on the same 16 pins at the moment. Two ADC devices are used in orde to distinguish between
 * fast and slow devices. The idea is that but only having few channels in 'fast' mode we can sample those faster?
 *
 * At the moment rusEfi does not allow to have more than 16 ADC channels combined. At the moment there is no flexibility to use
 * any ADC pins, only the hardcoded choice of 16 pins.
 *
 * Slow ADC group is used for IAT, CLT, AFR, VBATT etc - this one is currently sampled at 500Hz
 *
 * Fast ADC group is used for MAP, MAF HIP - this one is currently sampled at 10KHz
 *  We need frequent MAP for map_averaging.cpp
 *
 * 10KHz equals one measurement every 3.6 degrees at 6000 RPM
 *
 * @date Jan 14, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "global.h"

#if HAL_USE_ADC
#include "os_access.h"

#include "engine.h"
#include "adc_inputs.h"
#include "adc_subscription.h"
#include "AdcConfiguration.h"
#include "mpu_util.h"
#include "periodic_thread_controller.h"

#include "pin_repository.h"
#include "engine_math.h"
#include "engine_controller.h"
#include "maf.h"
#include "perf_trace.h"
#include "thread_priority.h"

/* Depth of the conversion buffer, channels are sampled X times each.*/
#ifndef ADC_BUF_DEPTH_FAST
#define ADC_BUF_DEPTH_FAST      4
#endif

static NO_CACHE adcsample_t slowAdcSamples[ADC_MAX_CHANNELS_COUNT];
static NO_CACHE adcsample_t fastAdcSampleBuf[ADC_BUF_DEPTH_FAST * ADC_MAX_CHANNELS_COUNT];

static adc_channel_mode_e adcHwChannelEnabled[HW_MAX_ADC_INDEX];

// Board voltage, with divider coefficient accounted for
float getVoltageDivided(const char *msg, adc_channel_e hwChannel DECLARE_ENGINE_PARAMETER_SUFFIX) {
	return getVoltage(msg, hwChannel PASS_ENGINE_PARAMETER_SUFFIX) * engineConfiguration->analogInputDividerCoefficient;
}

// voltage in MCU universe, from zero to VDD
float getVoltage(const char *msg, adc_channel_e hwChannel DECLARE_ENGINE_PARAMETER_SUFFIX) {
	return adcToVolts(getAdcValue(msg, hwChannel));
}

#if EFI_USE_FAST_ADC
AdcDevice::AdcDevice(ADCConversionGroup* hwConfig, adcsample_t *buf, size_t buf_len) {
	this->hwConfig = hwConfig;
	this->samples = buf;
	this->buf_len = buf_len;

	memset(hardwareIndexByIndernalAdcIndex, EFI_ADC_NONE, sizeof(hardwareIndexByIndernalAdcIndex));
	memset(internalAdcIndexByHardwareIndex, 0xFF, sizeof(internalAdcIndexByHardwareIndex));

#ifdef EFI_USE_ONLY_FAST_ADC
	for (int i = 0; i < ADC_MAX_CHANNELS_COUNT; i++) {
		internalAdcIndexByHardwareIndex[EFI_ADC_0 + i] = i;
		hardwareIndexByIndernalAdcIndex[i] = (adc_channel_e)(EFI_ADC_0 + i);
	}
#endif /* EFI_USE_ONLY_FAST_ADC */
}

#endif // EFI_USE_FAST_ADC

static uint32_t slowAdcCounter = 0;

// todo: move this flag to Engine god object
static int adcDebugReporting = false;

static adcsample_t getAvgAdcValue(int index, adcsample_t *samples, int bufDepth, int numChannels) {
	uint32_t result = 0;
	for (int i = 0; i < bufDepth; i++) {
		result += samples[index];
		index += numChannels;
	}

	// this truncation is guaranteed to not be lossy - the average can't be larger than adcsample_t
	return static_cast<adcsample_t>(result / bufDepth);
}


#if EFI_USE_FAST_ADC
void adc_callback_fast(ADCDriver *adcp);

AdcDevice fastAdc(getFastAdcConversionGroup(), fastAdcSampleBuf, ARRAY_SIZE(fastAdcSampleBuf));

//!!!!!!!!!!!!
static int fast_adc_bad_state = 0;
int fast_adc_callback_cnt = 0;

static void fast_adc_callback(GPTDriver*) {
#if EFI_INTERNAL_ADC
	/*
	 * Starts an asynchronous ADC conversion operation, the conversion
	 * will be executed in parallel to the current PWM cycle and will
	 * terminate before the next PWM cycle.
	 */
	chSysLockFromISR()
	;
	if (ADC_FAST_DEVICE.state != ADC_READY &&
	ADC_FAST_DEVICE.state != ADC_COMPLETE &&
	ADC_FAST_DEVICE.state != ADC_ERROR) {
		fast_adc_bad_state = ADC_FAST_DEVICE.state;
		fastAdc.errorsCount++;
		// todo: when? why? firmwareError(OBD_PCM_Processor_Fault, "ADC fast not ready?");
		chSysUnlockFromISR()
		;
		return;
	}

	adcStartConversionI(&ADC_FAST_DEVICE, getFastAdcConversionGroup(), fastAdc.samples, ADC_BUF_DEPTH_FAST);
	chSysUnlockFromISR()
	;
	fastAdc.conversionCount++;
#endif /* EFI_INTERNAL_ADC */
}
#endif // EFI_USE_FAST_ADC

static float mcuTemperature;

float getMCUInternalTemperature() {
	return mcuTemperature;
}

int getInternalAdcValue(const char *msg, adc_channel_e hwChannel) {
	if (!isAdcChannelValid(hwChannel)) {
		warning(CUSTOM_OBD_ANALOG_INPUT_NOT_CONFIGURED, "ADC: %s input is not configured", msg);
		return -1;
	}
#if EFI_ENABLE_MOCK_ADC
	if (engine->engineState.mockAdcState.hasMockAdc[hwChannel])
		return engine->engineState.mockAdcState.getMockAdcValue(hwChannel);

#endif /* EFI_ENABLE_MOCK_ADC */

#if EFI_USE_FAST_ADC
#ifndef EFI_USE_ONLY_FAST_ADC
	if (adcHwChannelEnabled[hwChannel] == ADC_FAST)
#endif /* EFI_USE_ONLY_FAST_ADC */
	{
		int internalIndex = fastAdc.internalAdcIndexByHardwareIndex[hwChannel];
#if (ADC_BUF_DEPTH_FAST == 1)
		return fastAdc.samples[internalIndex];
#else
		int value = getAvgAdcValue(internalIndex, fastAdc.samples, ADC_BUF_DEPTH_FAST, fastAdc.size());
		return value;
#endif /* ADC_BUF_DEPTH_FAST==1 */
	}
#endif // EFI_USE_FAST_ADC

	return slowAdcSamples[hwChannel - EFI_ADC_0];
}

#if EFI_USE_FAST_ADC
static GPTConfig fast_adc_config = {
	GPT_FREQ_FAST,
	fast_adc_callback,
	0, 0
};
#endif /* EFI_USE_FAST_ADC */

adc_channel_mode_e getAdcMode(adc_channel_e hwChannel) {
#if EFI_USE_FAST_ADC
	if (fastAdc.isHwUsed(hwChannel)) {
		return ADC_FAST;
	}
#endif // EFI_USE_FAST_ADC

	return ADC_SLOW;
}

#if EFI_USE_FAST_ADC

int AdcDevice::size() const {
#ifdef EFI_USE_ONLY_FAST_ADC
	return ADC_MAX_CHANNELS_COUNT;
#endif /* EFI_USE_ONLY_FAST_ADC */
	return channelCount;
}

int AdcDevice::getAdcValueByHwChannel(adc_channel_e hwChannel) const {
	int internalIndex = internalAdcIndexByHardwareIndex[hwChannel];
	return values.adc_data[internalIndex];
}

int AdcDevice::getAdcValueByIndex(int internalIndex) const {
	return values.adc_data[internalIndex];
}

void AdcDevice::init(void) {
	hwConfig->num_channels = size();
	/* driver does this internally */
	//hwConfig->sqr1 += ADC_SQR1_NUM_CH(size());
}

bool AdcDevice::isHwUsed(adc_channel_e hwChannelIndex) const {
	for (size_t i = 0; i < size(); i++) {
		if (hardwareIndexByIndernalAdcIndex[i] == hwChannelIndex) {
			return true;
		}
	}
	return false;
}

void AdcDevice::enableChannel(adc_channel_e hwChannel) {
	if ((channelCount + 1) >= ADC_MAX_CHANNELS_COUNT) {
		firmwareError(OBD_PCM_Processor_Fault, "Too many ADC channels configured");
		return;
	}

	int logicChannel = channelCount++;

	internalAdcIndexByHardwareIndex[hwChannel] = logicChannel;
	hardwareIndexByIndernalAdcIndex[logicChannel] = hwChannel;

	enableAdcChannel(hwConfig, hwChannel, logicChannel);
}

void AdcDevice::enableChannelAndPin(const char *msg, adc_channel_e hwChannel) {
	enableChannel(hwChannel);

	brain_pin_e pin = getAdcChannelBrainPin(msg, hwChannel);
	efiSetPadMode(msg, pin, PAL_MODE_INPUT_ANALOG);
}

adc_channel_e AdcDevice::getAdcHardwareIndexByInternalIndex(int index) const {
	return hardwareIndexByIndernalAdcIndex[index];
}

#endif // EFI_USE_FAST_ADC

static void printAdcValue(int channel) {
	int value = getAdcValue("print", (adc_channel_e)channel);
	float volts = adcToVoltsDivided(value);
	efiPrintf("adc voltage : %.2f", volts);
}

static uint32_t slowAdcConversionCount = 0;
static uint32_t slowAdcErrorsCount = 0;

void printFullAdcReport(void) {
#if EFI_USE_FAST_ADC
	efiPrintf("fast samples=%d errors=%d size=%d", fastAdc.conversionCount, fastAdc.errorsCount, fastAdc.size());
	//!!!!!!!!!!
	efiPrintf("fast state=%d callback_cnt=%d", fast_adc_bad_state, fast_adc_callback_cnt);

	for (int internalIndex = 0; internalIndex < fastAdc.size(); internalIndex++) {
		adc_channel_e hwIndex = fastAdc.getAdcHardwareIndexByInternalIndex(internalIndex);

		if (isAdcChannelValid(hwIndex)) {
			ioportid_t port = getAdcChannelPort("print", hwIndex);
			int pin = getAdcChannelPin(hwIndex);
			int adcValue = getAvgAdcValue(internalIndex, fastAdc.samples, ADC_BUF_DEPTH_FAST, fastAdc.size());
			float volts = adcToVolts(adcValue);
			/* Human index starts from 1 */
			efiPrintf(" F ch[%2d] @ %s%d ADC%d 12bit=%4d %.2fV",
				internalIndex, portname(port), pin, hwIndex - EFI_ADC_0 + 1, adcValue, volts);
		}
	}
#endif // EFI_USE_FAST_ADC
	efiPrintf("slow %d samples", slowAdcConversionCount);

	/* we assume that all slow ADC channels are enabled */
	for (int internalIndex = 0; internalIndex < ADC_MAX_CHANNELS_COUNT; internalIndex++) {
		adc_channel_e hwIndex = static_cast<adc_channel_e>(internalIndex + EFI_ADC_0);

		if (isAdcChannelValid(hwIndex)) {
			ioportid_t port = getAdcChannelPort("print", hwIndex);
			int pin = getAdcChannelPin(hwIndex);
			int adcValue = slowAdcSamples[internalIndex];
			float volts = adcToVolts(adcValue);
			/* Human index starts from 1 */
			efiPrintf(" S ch[%2d] @ %s%d ADC%d 12bit=%4d %.2fV",
				internalIndex, portname(port), pin, hwIndex - EFI_ADC_0 + 1, adcValue, volts);
		}
	}
}

static void setAdcDebugReporting(int value) {
	adcDebugReporting = value;
	efiPrintf("adcDebug=%d", adcDebugReporting);
}

void waitForSlowAdc(uint32_t lastAdcCounter) {
	// we use slowAdcCounter instead of slowAdc.conversionCount because we need ADC_COMPLETE state
	// todo: use sync.objects?
	while (slowAdcCounter <= lastAdcCounter) {
		chThdSleepMilliseconds(1);
	}
}

int getSlowAdcCounter() {
	return slowAdcCounter;
}


class SlowAdcController : public PeriodicController<256> {
public:
	SlowAdcController() 
		: PeriodicController("ADC", PRIO_ADC, SLOW_ADC_RATE)
	{
	}

	void PeriodicTask(efitick_t nowNt) override {
		{
			ScopePerf perf(PE::AdcConversionSlow);

			slowAdcConversionCount++;
			if (!readSlowAnalogInputs(slowAdcSamples)) {
				slowAdcErrorsCount++;
				return;
			}

#ifdef USE_ADC3_VBATT_HACK
			void proteusAdcHack();
			proteusAdcHack();
#endif

			// Ask the port to sample the MCU temperature
			mcuTemperature = getMcuTemperature();
		}

		{
			ScopePerf perf(PE::AdcProcessSlow);

			slowAdcCounter++;

			AdcSubscription::UpdateSubscribers(nowNt);
		}
	}
};

void addChannel(const char *name, adc_channel_e setting, adc_channel_mode_e mode) {
	if (!isAdcChannelValid(setting)) {
		return;
	}
	if (/*type-limited (int)setting < 0 || */(int)setting>=HW_MAX_ADC_INDEX) {
		firmwareError(CUSTOM_INVALID_ADC, "Invalid ADC setting %s", name);
		return;
	}

	adcHwChannelEnabled[setting] = mode;
#ifndef EFI_USE_ONLY_FAST_ADC
#if EFI_USE_FAST_ADC
	if (mode == ADC_FAST) {
		fastAdc.enableChannelAndPin(name, setting);
		return;
	}
#endif /* EFI_USE_FAST_ADC */
#endif /* EFI_USE_ONLY_FAST_ADC */

	// Slow ADC always samples all channels, simply set the input mode
	brain_pin_e pin = getAdcChannelBrainPin(name, setting);
	efiSetPadMode(name, pin, PAL_MODE_INPUT_ANALOG);
}

void removeChannel(const char *name, adc_channel_e setting) {
	(void)name;
	if (!isAdcChannelValid(setting)) {
		return;
	}
	adcHwChannelEnabled[setting] = ADC_OFF;
}

// Weak link a stub so that every board doesn't have to implement this function
__attribute__((weak)) void setAdcChannelOverrides() { }

static void configureInputs(void) {
	memset(adcHwChannelEnabled, 0, sizeof(adcHwChannelEnabled));

	/**
	 * order of analog channels here is totally random and has no meaning
	 * we also have some weird implementation with internal indices - that all has no meaning, it's just a random implementation
	 * which does not mean anything.
	 */

	addChannel("MAP", engineConfiguration->map.sensor.hwChannel, ADC_FAST);

	addChannel("HIP9011", engineConfiguration->hipOutputChannel, ADC_FAST);

	addChannel("Baro Press", engineConfiguration->baroSensor.hwChannel, ADC_SLOW);

	// not currently used	addChannel("Vref", engineConfiguration->vRefAdcChannel, ADC_SLOW);

	addChannel("AUXF#1", engineConfiguration->auxFastSensor1_adcChannel, ADC_FAST);

	addChannel("AFR", engineConfiguration->afr.hwChannel, ADC_SLOW);

	if (CONFIG(isCJ125Enabled)) {
		addChannel("CJ125 UR", engineConfiguration->cj125ur, ADC_SLOW);
		addChannel("CJ125 UA", engineConfiguration->cj125ua, ADC_SLOW);
	}

	setAdcChannelOverrides();
}

static SlowAdcController slowAdcController;

void initAdcInputs() {
	efiPrintf("initAdcInputs()");

	configureInputs();

	// migrate to 'enable adcdebug'
	addConsoleActionI("adcdebug", &setAdcDebugReporting);

#if EFI_INTERNAL_ADC
	portInitAdc();

	// Start the slow ADC thread
	slowAdcController.Start();

#if EFI_USE_FAST_ADC
	portInitFastAdc(adc_callback_fast);

	fastAdc.init();

	// todo: move GPT config to ports?
	gptStart(EFI_INTERNAL_FAST_ADC_GPT, &fast_adc_config);
	gptStartContinuous(EFI_INTERNAL_FAST_ADC_GPT, GPT_PERIOD_FAST);
#endif // EFI_USE_FAST_ADC

	addConsoleActionI("adc", (VoidInt) printAdcValue);
#else
	efiPrintf("ADC disabled");
#endif
}

void printFullAdcReportIfNeeded(void) {
	if (!adcDebugReporting)
		return;
	printFullAdcReport();
}

#else /* not HAL_USE_ADC */

__attribute__((weak)) float getVoltageDivided(const char*, adc_channel_e DECLARE_ENGINE_PARAMETER_SUFFIX) {
	return 0;
}

// voltage in MCU universe, from zero to VDD
__attribute__((weak)) float getVoltage(const char*, adc_channel_e DECLARE_ENGINE_PARAMETER_SUFFIX) {
	return 0;
}

#endif
