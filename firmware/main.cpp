/**
 * @file	main.cpp
 * @brief C++ main entry point
 *
 * @date Nov 29, 2012
 * @author Andrey Belomutskiy, (c) 2012-2020
 *      http://rusefi.com/
 */

#include "global.h"
#include "os_access.h"
#include "rusefi.h"
#include "mpu_util.h"

#if 1
#define BOARD_LED_PORT GPIOJ
#define BOARD_LED_PIN 15

static THD_WORKING_AREA(waThreadBlinker, 128);
static THD_FUNCTION(ThreadBlinker, arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palSetPad(BOARD_LED_PORT, BOARD_LED_PIN);
    chThdSleepMilliseconds(100);
    palClearPad(BOARD_LED_PORT, BOARD_LED_PIN);
    chThdSleepMilliseconds(100);
  }
}
#endif

static LoggingWithStorage sharedLogger("main");

#include "usbconsole.h"

//extern "C" void debugLog(const char *fmt, ...);

extern "C" {
char __debugBuffer[200];
int __debugEnabled = 0;
}

#define BOARD_LED1_PORT GPIOJ	// LED1=RED
#define BOARD_LED1_PIN 15

#define BOARD_LED2_PORT GPIOJ	// LED2=GREEN
#define BOARD_LED2_PIN 0

#define BOARD_LED3_PORT GPIOJ	// LED3=BLUE
#define BOARD_LED3_PIN 12

#define BOARD_LED4_PORT GPIOA	// LED4=YELLOW
#define BOARD_LED4_PIN 0

#define BOARD_MOD1_PORT GPIOD
#define BOARD_MOD1_PIN 5

static ioportid_t ledPorts[] = { 0, BOARD_LED1_PORT, BOARD_LED2_PORT, BOARD_LED3_PORT, BOARD_LED4_PORT };
static iopadid_t ledPads[]   = { 0, BOARD_LED1_PIN,  BOARD_LED2_PIN,  BOARD_LED3_PIN,  BOARD_LED4_PIN };

extern "C" void toggleLed(int led, int mode);

void toggleLed(int led, int mode) {
#if 1
	static uint8_t st[5] = { 0 };
	if ((st[led] == 0 && mode == 0) || mode == 1) {
		palClearPad(ledPorts[led], ledPads[led]);
	}
	else if ((st[led] != 0 && mode == 0) || mode == -1) {
		palSetPad(ledPorts[led], ledPads[led]);
	}
   	st[led] = (st[led] + 1) % 2/*10*/; //!!!!!!!!!!!
#endif
}

/*
void debugLog(const char *fmt, ...) {
	static char buffer[256];
	
	va_list ap;
	va_start(ap, fmt);
	vsprintf(buffer, fmt, ap);
	va_end(ap);

   	size_t transferred = strlen(buffer);
	chnWriteTimeout(&SDU1, buffer, transferred, TIME_MS2I(1000));
	chThdSleepMilliseconds(20);
}
*/

//!!!!!!!!!!!
//#include "main_test_spi_8860.cpp"
//#include "main_test_gpt.cpp"
//#include "main_test_adc.cpp"
#include "main_test_can.cpp"

int main(void) {
	/*
	 * ChibiOS/RT initialization
	 */
	halInit();
	chSysInit();

	/**
	 * most basic MCU initialization - no configuration access, no external hardware access
	 */
	baseMCUInit();

#if 1
	palSetPadMode(BOARD_LED_PORT, BOARD_LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(BOARD_LED_PORT, BOARD_LED_PIN);
	
	chThdCreateStatic(waThreadBlinker, sizeof(waThreadBlinker), NORMALPRIO, ThreadBlinker, NULL);
#endif

#if 1
	palSetPadMode(BOARD_LED2_PORT, BOARD_LED2_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(BOARD_LED2_PORT, BOARD_LED2_PIN);

	palSetPadMode(BOARD_LED3_PORT, BOARD_LED3_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(BOARD_LED3_PORT, BOARD_LED3_PIN);

	palSetPadMode(BOARD_LED4_PORT, BOARD_LED4_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(BOARD_LED4_PORT, BOARD_LED4_PIN);

	//palSetPadMode(BOARD_MOD1_PORT, BOARD_MOD1_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	//palClearPad(BOARD_MOD1_PORT, BOARD_MOD1_PIN);

#endif

	usb_serial_start();
	__debugEnabled = 1;
	
#if 0
	//palSetPadMode(GPIOB, 0, PAL_MODE_INPUT);
	palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(PAL_MODE_ALTERNATIVE_EXTINT));
	
	//Gpio1pin_InitIn(GPIO1PIN_P10, Gpio1pin_InitPullup(1u));
	//bFM_GPIO_ADE_AN00 = 0;

	for (;;) {
		int v = palReadPad(GPIOB, 0);
		//int v = Gpio1pin_Get(GPIO1PIN_P10); //bFM4_GPIO_PDIR1_P0
		palWritePad(BOARD_MOD1_PORT, BOARD_MOD1_PIN, v);
		toggleLed();
		chThdSleepMilliseconds(100);
	}
#endif

	//toggleLed();
#if 1
	for (int i = 0; i < 10; i++) {
		debugLog("Test %d!!!!\r\n", i);
		chThdSleepMilliseconds(300);
		//toggleLed();
	}

	debugLog("SystemCoreClock=%d\r\n", SystemCoreClock);

	test();
#endif

#if 0
	
//	chThdSleepMilliseconds(3000);
	
	
	for (int i = 0; i < 10*12; i++) {
		print("Test!!!!\n");
		chThdSleepMilliseconds(100);
	}
#endif

	runRusEfi();
	return 0;
}

