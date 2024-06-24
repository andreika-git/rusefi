
#include "pch.h"
#include "usbconsole.h"
#include "hardware.h"

extern "C" {
	#include "boot.h"
	#include "flash.h"
	#include "shared_params.h"
}

#define RCC_AHB1ENR (*((volatile unsigned long *)0x40023830))
#define GPIOG_MODER (*((volatile unsigned long *)0x40021800))
#define GPIOG_ODR (*((volatile unsigned long *)0x40021814))

extern "C" {
void blink_led(void) {
    // Enable clock for GPIOG
    asm volatile (
        "ldr r0, =%[rcc]\n"
        "ldr r1, [r0]\n"
        "orr r1, r1, #(1 << 6)\n" // Enable GPIOG clock
        "str r1, [r0]\n"
        :: [rcc] "i" (0x40023830)
        : "r0", "r1"
    );

    // Configure PG1 as output
    asm volatile (
        "ldr r0, =%[moder]\n"
        "ldr r1, [r0]\n"
        "bic r1, r1, #(3 << (2 * 1))\n" // Clear bits for PG1
        "orr r1, r1, #(1 << (2 * 1))\n" // Set PG1 as output
        "str r1, [r0]\n"
        :: [moder] "i" (0x40021800)
        : "r0", "r1"
    );

    // Infinite loop to blink the LED on PG1
    asm volatile (
        "ldr r0, =%[odr]\n"
        "blink_loop:\n"
        "ldr r1, [r0]\n"
        "eor r1, r1, #(1 << 1)\n" // Toggle PG1
        "str r1, [r0]\n"

        // Delay loop
        "ldr r2, =%[delay_val]\n"
        "delay_loop:\n"
        "subs r2, r2, #1\n"
        "bne delay_loop\n"

        "b blink_loop\n"
        :: [odr] "i" (0x40021814),
           [delay_val] "i" (1000000)
        : "r0", "r1", "r2"
    );
}
}

static blt_bool waitedLongerThanTimeout = BLT_FALSE;

class BlinkyThread : public chibios_rt::BaseStaticThread<256> {
protected:
	void main(void) override {
		Gpio yellow = getWarningLedPin();
		Gpio blue = getCommsLedPin();
		Gpio green = getRunningLedPin();
		Gpio red = LED_CRITICAL_ERROR_BRAIN_PIN;

		efiSetPadMode("yellow", yellow, PAL_MODE_OUTPUT_PUSHPULL);
		efiSetPadMode("blue", blue, PAL_MODE_OUTPUT_PUSHPULL);
		efiSetPadMode("green", green, PAL_MODE_OUTPUT_PUSHPULL);
		efiSetPadMode("red", red, PAL_MODE_OUTPUT_PUSHPULL);

		auto yellowPort = getBrainPinPort(yellow);
		auto yellowPin = getBrainPinIndex(yellow);
		auto bluePort = getBrainPinPort(blue);
		auto bluePin = getBrainPinIndex(blue);
		auto greenPort = getBrainPinPort(green);
		auto greenPin = getBrainPinIndex(green);
		auto redPort = getBrainPinPort(red);
		auto redPin = getBrainPinIndex(red);

		if (yellowPort) {
			palSetPad(yellowPort, yellowPin);
		}
		if (bluePort) {
			palSetPad(bluePort, bluePin);
		}
		if (greenPort) {
			palSetPad(greenPort, greenPin);
		}
		if (redPort) {
			palSetPad(redPort, redPin);
		}

		while (true) {
			if (is2ndBootloader()) {
				if (yellowPort) {
					palTogglePad(yellowPort, yellowPin);
				}
				if (bluePort) {
					palTogglePad(bluePort, bluePin);
				}
			} else {
				if (greenPort) {
					palTogglePad(greenPort, greenPin);
				}
				if (redPort) {
					palTogglePad(redPort, redPin);
				}
			}
			// blink 3 times faster if Dual Bank is not enabled
			auto delay = isFlashDualBank() ? 125 : 40;
			// blink faster if not in the waiting mode
			chThdSleepMilliseconds(waitedLongerThanTimeout ? (delay * 2) : delay);
		}
	}
};

static BlinkyThread blinky;

static blt_bool checkIfRebootIntoOpenBltRequested(void) {
	uint8_t value = 0x00;
	if (SharedParamsReadByIndex(0, &value) && (value == 0x01)) {
		/* clear */
		SharedParamsWriteByIndex(0, 0x00);
		return BLT_TRUE;
	}
	return BLT_FALSE;
}

int main(void) {
	halInit();
	chSysInit();

	baseMCUInit();

	// start the blinky thread
	blinky.start(NORMALPRIO + 10);

	// Init openblt shared params
	SharedParamsInit();

	// Init openblt itself
	BootInit();

	blt_bool stayInBootloader = checkIfRebootIntoOpenBltRequested();
	blt_bool wasConnected = BLT_FALSE;

	while (true) {
		BootTask();

		// since BOOT_BACKDOOR_HOOKS_ENABLE==TRUE, BackDoorCheck() is not working
		// so we have to manually check if we need to jump to the main firmware
		if (ComIsConnected() == BLT_TRUE) {
			// remember we had connection attempt
			wasConnected = BLT_TRUE;
			continue;
		}
		if (stayInBootloader || wasConnected)
			continue;
#if (BOOT_BACKDOOR_ENTRY_TIMEOUT_MS > 0)
		blt_bool isTimeout = (TIME_I2MS(chVTGetSystemTime()) >= BOOT_BACKDOOR_ENTRY_TIMEOUT_MS);
#else
		blt_bool isTimeout = BLT_TRUE;
#endif // BOOT_BACKDOOR_ENTRY_TIMEOUT_MS
		if (isTimeout == BLT_TRUE) {
			waitedLongerThanTimeout = BLT_TRUE;
			if (!is2ndBootloader()) {
				CpuStartUserProgram();
			}
		}
	}
}

// very basic version, supports on chip pins only (really only used for USB)
void efiSetPadMode(const char* msg, brain_pin_e brainPin, iomode_t mode) {
	ioportid_t port = getHwPort(msg, brainPin);
	ioportmask_t pin = getHwPin(msg, brainPin);
	/* paranoid */
	if (port == GPIO_NULL) {
		return;
	}

	palSetPadMode(port, pin, mode);
}
