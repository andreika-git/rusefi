/**
 * @file	main.cpp
 * @brief C++ main entry point
 *
 * @date Nov 29, 2012
 * @author Andrey Belomutskiy, (c) 2012-2018
 *      http://rusefi.com/
 */

#include "global.h"
#include "rusefi.h"
#include "mpu_util.h"

#include "gpio/gpio_ext.h"
#include "gpio/tle6240.h"

#define BOARD_LED_PORT GPIOD
#define BOARD_LED_PIN 7

static THD_WORKING_AREA(waThreadBlinker, 128);
static THD_FUNCTION(ThreadBlinker, arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palSetPad(BOARD_LED_PORT, BOARD_LED_PIN);
    chThdSleepMilliseconds(500);
    palClearPad(BOARD_LED_PORT, BOARD_LED_PIN);
    chThdSleepMilliseconds(500);
  }
}

static char debugBuffer[256];
#define debugLog(fmt,...) { \
	chsnprintf(debugBuffer, sizeof(debugBuffer), fmt, ##__VA_ARGS__); \
	size_t transferred = strlen(debugBuffer); \
	uartSendTimeout(&UARTD2, &transferred, (const void *)debugBuffer, TIME_MS2I(1000)); \
}

/*
 * UART 1 configuration structure.
 */
const UARTConfig uartConf = {
  NULL,   /* UART transmission buffer callback.           */
  NULL,   /* UART physical end of transmission callback.  */
  NULL,   /* UART Receiver receiver filled callback.      */
  NULL,   /* UART caracter received callback.             */
  NULL,   /* UART received error callback.                */
  115200, /* UART baudrate.                               */
  0, 0, 0
};

int was_callback = 0;
static void adc_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
	was_callback++;
}

#define ADC_NUM_CHANNELS   16
#define ADC_BUF_DEPTH      1
#define ADC_SAMPLING_FAST ADC_SAMPLE_28
	static adcsample_t samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];
	static ADCConversionGroup adcgrpcfg = { FALSE, ADC_NUM_CHANNELS, adc_callback, NULL,
		ADC_TwoSamplingDelay_5Cycles, ADC_CR2_SWSTART, 
		ADC_SMPR1_SMP_AN10(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN11(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN12(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN13(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN14(ADC_SAMPLING_FAST) |
		ADC_SMPR1_SMP_AN15(ADC_SAMPLING_FAST)
		, // sample times for channels 10...18
		ADC_SMPR2_SMP_AN0(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN1(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN2(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN3(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN4(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN5(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN6(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN7(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN8(ADC_SAMPLING_FAST) |
		ADC_SMPR2_SMP_AN9(ADC_SAMPLING_FAST),
		0, 0, 0 };

int pwmCnt1 = 0, pwmCnt2 = 0;
static void pwmpcb_slow(PWMDriver *pwmp) {
	(void) pwmp;
	pwmCnt1++;

	chSysLockFromISR();
	if (ADCD1.state != ADC_READY && ADCD1.state != ADC_COMPLETE && ADCD1.state != ADC_ERROR) {
		chSysUnlockFromISR();
		return;
	}
	adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_BUF_DEPTH);
	pwmCnt2++;
	chSysUnlockFromISR();
}

// 168000000/64/125/1050 = 20Hz
#define PWM_FREQ_SLOW 21000   /* PWM clock frequency. I wonder what does this setting mean?  */
#define PWM_PERIOD_SLOW 1050  /* PWM period (in PWM ticks).    */

#if HAL_USE_PWM || defined(__DOXYGEN__)
static PWMConfig pwmcfg_slow = { PWM_FREQ_SLOW, PWM_PERIOD_SLOW, pwmpcb_slow, { {
PWM_OUTPUT_DISABLED, NULL }, { PWM_OUTPUT_DISABLED, NULL }, {
PWM_OUTPUT_DISABLED, NULL }, { PWM_OUTPUT_DISABLED, NULL } },
/* HW dependent part.*/
0, 0 };
#endif /* HAL_USE_PWM */


int main(void) {
	int ret;

	/*
	 * ChibiOS/RT initialization
	 */
    halInit();
	chSysInit();
	
	palSetPadMode(BOARD_LED_PORT, BOARD_LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(BOARD_LED_PORT, BOARD_LED_PIN);
	chThdCreateStatic(waThreadBlinker, sizeof(waThreadBlinker), NORMALPRIO, ThreadBlinker, NULL);
	
	uartInit();
	uartStart(&UARTD2, &uartConf);
	palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(2));
	palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(2));

	debugLog("Starting...\r\n");

	//////////////////////////////////////////////////////////

	adcStart(&ADCD1, NULL);

	debugLog("Starting ADC PWM...\r\n");

	pwmStart(EFI_INTERNAL_SLOW_ADC_PWM, &pwmcfg_slow);
	debugLog("pwmEnablePeriodicNotification()\r\n");
	pwmEnablePeriodicNotification(EFI_INTERNAL_SLOW_ADC_PWM);

	debugLog("Done!\r\n");

    while (1)
    {
    	uint32_t flags = ADC12_GetChannelStatusFlags(ADCD1.adc, 0);
		debugLog("pwmCnt1=%d pwmCnt2=%d was_callback=%d flags=%d\r\n", pwmCnt1, pwmCnt2, (int)was_callback, flags);
		if (was_callback > 0) {
			for (int i = 0; i < ADC_NUM_CHANNELS*ADC_BUF_DEPTH; i++) {
				debugLog("[%d] %.2fV\r\n", i, 5.0f * (float)samples[i] / 4095);
				samples[i] = 0;
			}
			was_callback = 0;
		}
		chThdSleepMilliseconds(1000);
    }
    return 0;
}
