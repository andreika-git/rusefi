#include "pch.h"

extern "C" {
	#include "boot.h"
	#include "flash.h"
}

void CpuInit() { }
void CopInit() { }

void TimerInit() { }
void TimerReset() { }

void CopService() { }
void TimerUpdate() { }

#ifndef SRAM_END
#define SRAM_END (SRAM_BASE + 128*1024)
#endif

static volatile uint32_t vector_ram[CORTEX_NUM_VECTORS] __attribute__((aligned(1024)));
extern uint32_t _vectors[];

blt_bool is2ndBootloader(void) {
	uint32_t currentAddress;
	asm("mov %0, pc" : "=r"(currentAddress));
	// if we run inside the first 0x8000 bytes, then we're the first bootloader, 
	// otherwise we're the 2nd one
	return (currentAddress >= OFFSET_AFTER_BOOTLOADER);
}

#define COPY_TABLE(flash_base, start, end, offset) \
    asm volatile( \
        "ldr r0, =" # flash_base "\n\t"       /* r0 = address of flash_base */ \
        "ldr r1, =" # start "\n\t"       /* r1 = address of start */ \
        "ldr r2, =" # end "\n\t"       /* r2 = address of end */ \
        "add r0, r0, %0\n\t"    /* r0 = base_addr = flash_base + offset */ \
        "add r1, r1, %0\n\t"    /* r1 = start + offset */ \
        "add r2, r2, %0\n\t"    /* r2 = end + offset */ \
        "1:\n\t"                /* Loop start label */ \
        "ldr r3, [r0], #4\n\t" /* Load word from flash, increment r0 (load) */ \
		/* Check if r3 is within the first range (%1..%2) */ \
		"cmp r3, %1\n\t"       /* Compare r3 with lower bound */ \
	    "blt 2f\n\t"           /* If r3 < lower bound, skip addition */ \
	    "cmp r3, %2\n\t"       /* Compare r3 with upper bound */ \
	    "bgt 2f\n\t"           /* If r3 > upper bound, skip addition */ \
        "add r3, r3, %0\n\t"   /* Add offset to r3 */\
        "b 3f\n\t"             /* Branch to the end of range checks */  \
        /* Check if r3 is within the second range (%3..%4) */ \
        "2:\n\t" \
        "cmp r3, %3\n\t"          /* Compare r3 with lower bound of 2nd range */ \
        "blt 3f\n\t"              /* If r3 < lower bound, skip addition */ \
        "cmp r3, %4\n\t"          /* Compare r3 with upper bound of 2nd range */ \
        "bgt 3f\n\t"              /* If r3 > upper bound, skip addition */ \
        "add r3, r3, %0\n\t"      /* Add offset to r3 */ \
        "3:\n\t"               \
        "str r3, [r1], #4\n\t" /* Store word to RAM, increment r1 (start) */ \
        "cmp r1, r2\n\t"       /* Compare start with end */ \
        "blt 1b\n\t"           /* Branch to loop start if start < end */ \
        :                       /* No output operands */ \
        : "r" (offset), "r" (FLASH_BASE), "r" (FLASH_END), "r" (SRAM_BASE), "r" (SRAM_END) \
        : "r0", "r1", "r2", "r3", "cc" /* Clobbered registers */ \
    )

static void initGot(uint32_t offset) {
    // copy .got from flash to RAM and relocate
    COPY_TABLE(__got_flash_base__, __got_start__, __got_end__, offset);
}

static void initGotPlt(uint32_t offset) {
	// copy .got.plt from flash to RAM and relocate
	COPY_TABLE(__got_plt_flash_base__, __got_plt_start__, __got_plt_end__, offset);
}

static void initData(uint32_t offset) {
	// copy .data from flash to RAM and relocate
    COPY_TABLE(__textdata_base__, __data_base__, __data_end__, offset);
}

// relocate VTOR for the 2nd bootloader
static void initVectorsInRam(uint32_t offset) {
    __disable_irq();
    // copy vector table into RAM
    for (uint32_t i = 0; i < CORTEX_NUM_VECTORS; i++) {
        vector_ram[i] = _vectors[i];
		// relocate addresses for handlers in flash
        /*if (vector_ram[i] >= FLASH_BASE && vector_ram[i] <= FLASH_END)*/ {
        	vector_ram[i] += offset;
        }
	}
    // reassign the vectors
    SCB->VTOR = (uint32_t)vector_ram;
    // don't call __enable_irq() yet
}

static void initBss(uint32_t offset) {
	extern uint32_t __bss_base__;
	extern uint32_t __bss_end__;
    uint32_t *bss = (uint32_t *)((uint32_t)&__bss_base__ + offset);
    uint32_t *bss_end = (uint32_t *)((uint32_t)&__bss_end__ + offset);

    while (bss < bss_end) {
        *bss++ = 0;
    }
}

static void callConstructors(uint32_t offset) {
    extern uint32_t __init_array_base__;
	extern uint32_t __init_array_end__;
	typedef void (*init_func_t)(void);

    init_func_t *func_ptr = (init_func_t *)&__init_array_base__;
    init_func_t *func_end = (init_func_t *)&__init_array_end__;

    while (func_ptr < func_end) {
        // check if the constructor address is within the flash range
        init_func_t ctor = *func_ptr;
        /*if ((uint32_t)ctor >= FLASH_BASE && (uint32_t)ctor <= FLASH_END)*/ {
            ctor = (init_func_t)((uint32_t)ctor + offset); // apply offset
        }
        ctor(); // Call the constructor function
        func_ptr++;    // Move to the next constructor
    }
}

extern "C" void blink_led(void);

extern "C" void __core_init() {
	uint32_t offset = is2ndBootloader() ? BOOTLOADER_SIZE : 0;
	
	// must be called ASAP
    initGot(offset);
    initGotPlt(offset);
    // copy data (CRT0_INIT_DATA=FALSE)
	initData(offset);
	// init bss section (CRT0_INIT_BSS=FALSE)
	initBss(offset);
	// setup vectors (CRT0_VTOR_INIT=FALSE)
	initVectorsInRam(offset);
	// call ctors (CRT0_CALL_CONSTRUCTORS=FALSE)
	callConstructors(offset);

    // *** this is about specifically stm32f7 ***
	// This overrides the built-in __core_init() function
	// We do this to avoid enabling the D/I caches, which
	// we'll immediately have to turn back off when jumping
	// to the main firmware (which will then enable them itself)
}

blt_int32u TimerGet() {
	return 0;
}

void CpuMemCopy(blt_addr dest, blt_addr src, blt_int16u len)
{
	memcpy((void*)dest, (void*)src, len);
}

void CpuMemSet(blt_addr dest, blt_int8u value, blt_int16u len)
{
	memset((void*)dest, value, len);
}

/** \brief Pointer to the user program's reset vector. */
#define CPU_USER_PROGRAM_STARTADDR_PTR    ((blt_addr)(NvmGetUserProgBaseAddress() + 0x00000004))
/** \brief Pointer to the user program's vector table. */
#define CPU_USER_PROGRAM_VECTABLE_OFFSET  ((blt_int32u)NvmGetUserProgBaseAddress())

void CpuStartUserProgram(void)
{
  void (*pProgResetHandler)(void);

  /* check if a user program is present by verifying the checksum */
  if (NvmVerifyChecksum() == BLT_FALSE)
  {
#if (BOOT_COM_DEFERRED_INIT_ENABLE > 0) && (BOOT_COM_ENABLE > 0)
    /* bootloader will stay active so perform deferred initialization to make sure
     * the communication interface that were not yet initialized are now initialized.
     * this is needed to make sure firmware updates via these communication interfaces
     * will be possible.
     */
    ComDeferredInit();
#endif
    /* not a valid user program so it cannot be started */
    return;
  }
#if (BOOT_CPU_USER_PROGRAM_START_HOOK > 0)
  /* invoke callback */
  if (CpuUserProgramStartHook() == BLT_FALSE)
  {
  #if (BOOT_COM_DEFERRED_INIT_ENABLE > 0) && (BOOT_COM_ENABLE > 0)
    /* bootloader will stay active so perform deferred initialization to make sure
     * the communication interface that were not yet initialized are now initialized.
     * this is needed to make sure firmware updates via these communication interfaces
     * will be possible.
     */
    ComDeferredInit();
  #endif
    /* callback requests the user program to not be started */
    return;
  }
#endif
#if (BOOT_COM_ENABLE > 0)
  /* release the communication interface */
  ComFree();
#endif
  /* reset the HAL */
  chSysDisable();
  /* reset the timer */
  TimerReset();
  /* remap user program's vector table */
  SCB->VTOR = CPU_USER_PROGRAM_VECTABLE_OFFSET & (blt_int32u)0x1FFFFF80;
  /* set the address where the bootloader needs to jump to. this is the address of
   * the 2nd entry in the user program's vector table. this address points to the
   * user program's reset handler.
   */
  pProgResetHandler = (void(*)(void))(*((blt_addr *)CPU_USER_PROGRAM_STARTADDR_PTR));
  /* The Cortex-M4 core has interrupts enabled out of reset. the bootloader
   * explicitly disables these for security reasons. Enable them here again, so it does
   * not have to be done by the user program.
   */
  /* start the user program by activating its reset interrupt service routine */
  pProgResetHandler();
#if (BOOT_COM_DEFERRED_INIT_ENABLE > 0) && (BOOT_COM_ENABLE > 0)
  /* theoretically, the code never gets here because the user program should now be
   * running and the previous function call should not return. In case it did return
   * for whatever reason, make sure all communication interfaces are initialized so that
   * firmware updates can be started.
   */
  ComDeferredInit();
#endif
} /*** end of CpuStartUserProgram ***/


