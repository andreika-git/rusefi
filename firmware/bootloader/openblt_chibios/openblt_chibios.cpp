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

//static uint32_t _vector_ram[CORTEX_NUM_VECTORS] __attribute__ ((section (".vectors_ram")));
static __attribute__((aligned(256))) uint32_t vector_ram[CORTEX_NUM_VECTORS];
extern uint32_t _vectors[];

blt_bool is2ndBootloader(void) {
	uint32_t currentAddress;
	asm("mov %0, pc" : "=r"(currentAddress));
	// if we run inside the first 0x8000 bytes, then we're the first bootloader, 
	// otherwise we're the 2nd one
	return (currentAddress >= OFFSET_AFTER_BOOTLOADER);
}

// relocate VTOR for the 2nd bootloader
static void relVtorFor2ndBootloader(void) {
	/*if (is2ndBootloader())*/ {
	    // copy vector table into RAM
	    for (uint32_t i = 0; i < CORTEX_NUM_VECTORS; i++) {
	        vector_ram[i] = _vectors[i];
			// relocate addresses for handlers in flash
	        if (vector_ram[i] >= FLASH_BASE) {
	        	vector_ram[i] += BOOTLOADER_SIZE;
	        }
	    }
	    // reassign the vectors
	    SCB->VTOR = (uint32_t)vector_ram;
	}
}

static void initGot(void) {
	// Unfortunately we need asm to setup GOT because we cannot use global variables without GOT
	asm volatile (
        // Load current PC into r7
        "mov r7, pc\n"

        // Load constants into registers
        "ldr r0, =%0\n"                // r0 = BOOTLOADER_SIZE
        "ldr r1, =%1\n"                // r1 = OFFSET_AFTER_BOOTLOADER
        "ldr r2, =%2\n"                // r2 = FLASH_BASE
        "ldr r3, =__got_flash_base__\n"// r3 = Original GOT flash base

        // Determine the GOT flash base address based on PC position
        "cmp r7, r1\n"                 // Compare PC (r7) with OFFSET_AFTER_BOOTLOADER
        "it hs\n"                      // If PC >= OFFSET_AFTER_BOOTLOADER
        "addhs r3, r3, r0\n"           // Adjust r3 by adding BOOTLOADER_SIZE if PC >= OFFSET_AFTER_BOOTLOADER

        // Determine the flash base value
        "mov r6, #0xffffffff\n"        // Default value of flash_base (0xffffffff)
        "cmp r7, r1\n"                 // Compare PC (r7) with OFFSET_AFTER_BOOTLOADER
        "it hs\n"                       // If PC >= OFFSET_AFTER_BOOTLOADER
        "movhs r6, r2\n"               // Set r6 = FLASH_BASE if PC >= OFFSET_AFTER_BOOTLOADER

        // Load the start and end addresses of the GOT section in RAM
        "ldr r4, =__got_start__\n"     // r4 = Start of GOT in RAM
        "ldr r5, =__got_end__\n"       // r5 = End of GOT in RAM

        // Copy loop: from flash (r3) to RAM (r4)
        "1:\n"
        "cmp r4, r5\n"                 // Compare r4 with r5
        "bge 2f\n"                     // If r4 >= r5, exit the loop
        "ldr r8, [r3], #4\n"           // Load 4 bytes from flash (r3) into r8, increment r3
        "cmp r8, r6\n"                 // Compare loaded GOT entry (r8) with flash_base
        "it hs\n"                      // If r8 >= flash_base
        "addhs r8, r8, r0\n"           // Adjust GOT entry by adding BOOTLOADER_SIZE if r8 >= flash_base
        "str r8, [r4], #4\n"           // Store 4 bytes from r8 into RAM (r4), increment r4
        "b 1b\n"                       // Repeat the loop
        "2:\n"
                        
        // Set global pointer to the start of the GOT 
        "ldr r9, =__got_start__\n"
        :
        : "i" (BOOTLOADER_SIZE), "i" (OFFSET_AFTER_BOOTLOADER), "i" (FLASH_BASE)
        : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "memory"
    );
}

void threadInitHook(void *tp) {
	// set GOT table for all thread contexts
	asm volatile (
        "ldr r9, =__got_start__\n"
    );
	asm("mov %0, r9" : "=r"(((thread_t *)tp)->ctx.sp->r9));
}

static void callConstructors(void) {
	extern void (*__init_array_base__[])(void);
	extern void (*__init_array_end__[])(void);

	asm volatile (
        // Load the base and end addresses of the constructors array
        "ldr r4, %[init_array_base]\n"
        "ldr r5, %[init_array_end]\n"

        // Loop to call each constructor
        "initloop:\n"
        "cmp r4, r5\n"
        "bge endinitloop\n"
        "ldr r1, [r4], #4\n"

        // Add bootloader size if address is greater than or equal to OFFSET_AFTER_BOOTLOADER
        "ldr r2, =%[offset_after_bootloader]\n"
        "cmp r1, r2\n"
        "blt skip_bootloader_adjustment\n"
        "add r1, r1, %[bootloader_size]\n"

        "skip_bootloader_adjustment:\n"
        "blx r1\n"
        "b initloop\n"
        
        // End of the loop
        "endinitloop:\n"
        :
        : [init_array_base] "m" (__init_array_base__),
          [init_array_end] "m" (__init_array_end__),
          [offset_after_bootloader] "i" (OFFSET_AFTER_BOOTLOADER),
          [bootloader_size] "i" (BOOTLOADER_SIZE)
        : "r1", "r2", "r4", "r5", "cc", "memory"
    );
}

static void initData(void) {
	// copy .data from flash to RAM and relocate
    extern uint32_t __textdata_base__, __data_base__, __data_end__;
    uint32_t textdata_base_addr = (uint32_t)&__textdata_base__;
	uint32_t *data_load = (uint32_t *)(textdata_base_addr + (is2ndBootloader() ? BOOTLOADER_SIZE : 0));
    uint32_t *data_start = &__data_base__;
    uint32_t *data_end = &__data_end__;
    while (data_start < data_end) {
        *data_start++ = *data_load++;
    }
}

extern void blink_led(void);

extern "C" void __core_init() {
	// must be called ASAP
	initGot();
	// setup vectors
	relVtorFor2ndBootloader();

    // *** this is about specifically stm32f7 ***
	// This overrides the built-in __core_init() function
	// We do this to avoid enabling the D/I caches, which
	// we'll immediately have to turn back off when jumping
	// to the main firmware (which will then enable them itself)
}

extern "C" void __late_init_custom(void) {
    initData();
	callConstructors();
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


