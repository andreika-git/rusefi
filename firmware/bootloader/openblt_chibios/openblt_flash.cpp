#include "pch.h"
#include "flash_int.h"

extern "C" {
	#include "boot.h"
	#include "flash.h"
}

void FlashInit() {
	// Flash already init by ChibiOS
}

blt_addr FlashGetUserProgBaseAddress() {
	if (is2ndBootloader())
		return FLASH_BASE;
	return OFFSET_AFTER_BOOTLOADER;
}

static blt_bool isInsideBootloader(blt_addr addr) {
	if (is2ndBootloader())
		return (addr >= OFFSET_AFTER_BOOTLOADER);
	return (addr < OFFSET_AFTER_BOOTLOADER);
}

blt_bool FlashWrite(blt_addr addr, blt_int32u len, blt_int8u *data) {
	// don't allow overwriting the bootloader
	if (isInsideBootloader(addr)) {
		return BLT_FALSE;
	}

	return (FLASH_RETURN_SUCCESS == intFlashWrite(addr, (const char*)data, len)) ? BLT_TRUE : BLT_FALSE;

	return BLT_TRUE;
}

blt_bool FlashErase(blt_addr addr, blt_int32u len) {
	// don't allow erasing the bootloader
	if (isInsideBootloader(addr)) {
		return BLT_FALSE;
	}

	if (!intFlashIsErased(addr, len)) {
		return (FLASH_RETURN_SUCCESS == intFlashErase(addr, len)) ? BLT_TRUE : BLT_FALSE;
	}

	return BLT_TRUE;
}

blt_bool FlashDone() {
	return BLT_TRUE;
}

blt_bool FlashWriteChecksum() {
	return BLT_TRUE;
}

blt_bool FlashVerifyChecksum() {
	// Naive check: if the first block is blank, there's no code there
	return intFlashIsErased(FlashGetUserProgBaseAddress(), 4) ? BLT_FALSE : BLT_TRUE;
}

blt_bool isFlashDualBank(void) {
#ifdef STM32F7XX
	// cleared bit indicates dual bank
	return (FLASH->OPTCR & FLASH_OPTCR_nDBANK) == 0 ? BLT_TRUE : BLT_FALSE;
#else
	return BLT_TRUE;
#endif
}
