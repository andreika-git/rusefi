#include "pch.h"

#include "hal.h"

#include "can_hw.h"
#include "can_common.h"

extern "C" {
	#include "boot.h"
}

// CAN1 PB8+PB9 and CAN2 PB5+PB6 pins are commonly used by Hellen.
// CAN2 PB5+PB13 pins can be used for ST-bootloader compatibility.
// 
// Other STM32 CAN pin combinations:
// CAN1_RX: { PI9, PA11, PH14, PD0, PB8 }, CAN1_TX: { PA12, PH13, PD1, PB9 }
// CAN2_RX: { PB5, PB12 }, CAN2_TX: { PB6, PB13 }

#ifndef BOOT_COM_CAN_CHANNEL_INDEX
  #error BOOT_COM_CAN_CHANNEL_INDEX is not defined.
#elif (BOOT_COM_CAN_CHANNEL_INDEX == 0)
  #ifndef STM32_CAN_USE_CAN1
  #error STM32_CAN_USE_CAN1 is not enabled for CAN index 0
  #endif
  #undef OPENBLT_CAND
  #define OPENBLT_CAND CAND1
#elif (BOOT_COM_CAN_CHANNEL_INDEX == 1)
  #ifndef STM32_CAN_USE_CAN2
  #error STM32_CAN_USE_CAN2 is not enabled for CAN index 1
  #endif
  #undef OPENBLT_CAND
  #define OPENBLT_CAND CAND2
#else
  #error Unknown BOOT_COM_CAN_CHANNEL_INDEX.
#endif

#if !defined(OPENBLT_CAN_RX_PIN) || !defined(OPENBLT_CAN_RX_PORT) || !defined(OPENBLT_CAN_TX_PIN) || !defined(OPENBLT_CAN_TX_PORT)
#if (BOOT_COM_CAN_CHANNEL_INDEX == 0)
  // default pins for CAN1 (compatible with Hellen)
  #define OPENBLT_CAN_RX_PORT GPIOB
  #define OPENBLT_CAN_RX_PIN 8
  #define OPENBLT_CAN_TX_PORT GPIOB
  #define OPENBLT_CAN_TX_PIN 9
#elif (BOOT_COM_CAN_CHANNEL_INDEX == 1)
  // default pins for CAN2 (compatible with ST-bootloader)
  #define OPENBLT_CAN_RX_PORT GPIOB
  #define OPENBLT_CAN_RX_PIN 5
  #define OPENBLT_CAN_TX_PORT GPIOB
  #define OPENBLT_CAN_TX_PIN 13
#endif
#endif

extern const CANConfig *findCanConfig(can_baudrate_e rate);

/************************************************************************************//**
** \brief     Initializes the CAN controller and synchronizes it to the CAN bus.
** \return    none.
**
****************************************************************************************/
extern "C" void CanInit(void) {
	// init pins
	palSetPadMode(OPENBLT_CAN_TX_PORT, OPENBLT_CAN_TX_PIN, PAL_MODE_ALTERNATE(EFI_CAN_TX_AF));
	palSetPadMode(OPENBLT_CAN_RX_PORT, OPENBLT_CAN_RX_PIN, PAL_MODE_ALTERNATE(EFI_CAN_RX_AF));

	auto cfg = findCanConfig(B500KBPS);
	canStart(&OPENBLT_CAND, cfg);
}


/************************************************************************************//**
** \brief     Transmits a packet formatted for the communication interface.
** \param     data Pointer to byte array with data that it to be transmitted.
** \param     len  Number of bytes that are to be transmitted.
** \return    none.
**
****************************************************************************************/
extern "C" void CanTransmitPacket(blt_int8u *data, blt_int8u len)
{
	blt_int32u txMsgId = BOOT_COM_CAN_TX_MSG_ID;
	CANTxFrame frame;

	if ((txMsgId & 0x80000000) == 0)
	{
		/* set the 11-bit CAN identifier. */
		frame.SID = txMsgId;
		frame.IDE = CAN_IDE_STD;
	}
	else
	{
		txMsgId &= ~0x80000000;
		/* set the 29-bit CAN identifier. */
		frame.EID = txMsgId;
		frame.IDE = CAN_IDE_EXT;
	}

	// Copy data/DLC
	frame.DLC = len;
	memcpy(frame.data8, data, len);

	canTransmitTimeout(&OPENBLT_CAND, CAN_ANY_MAILBOX, &frame, TIME_MS2I(100));
}

static blt_bool CanCheckPacketId(const CANRxFrame & frame, blt_int32u rxMsgId) {
	// Check that the ID type matches this frame (std vs ext)
	bool configuredAsExt = (rxMsgId & 0x80000000) != 0;
	if (configuredAsExt != frame.IDE) {
		// Wrong frame type
		return BLT_FALSE;
	}

	// Check that the frame's ID matches
	if (frame.IDE) {
		if (frame.EID != (rxMsgId & ~0x80000000)) {
			// Wrong ID
			return BLT_FALSE;
		}
	} else {
		if (frame.SID != rxMsgId) {
			// Wrong ID
			return BLT_FALSE;
		}
	}
	return BLT_TRUE;
}

static bool wasFwWipeout = false;

static blt_bool CanRequestFwWipeoutPacket(const CANRxFrame & frame, blt_int8u *data, blt_int8u *len) {
	static const blt_int8u wipeoutDataMagic[8] = { 0x01, 0x23, 0xDE, 0x88, 0x00, 0x00, 0x00, 0x00 };
	if (wasFwWipeout)
		return BLT_FALSE;
	if (frame.DLC != sizeof(wipeoutDataMagic)) {
		return BLT_FALSE;
	}
	if (memcmp(frame.data8, wipeoutDataMagic, sizeof(wipeoutDataMagic)) != 0) {
		return BLT_FALSE;
	}

	// start erasing
	blt_addr   eraseAddr = NvmGetUserProgBaseAddress();
	blt_int32u bootloaderSize = eraseAddr - FLASH_BASE;
	blt_int32u eraseLen = 512 * 1024 - bootloaderSize;	// erase the first 512k of the fw
	if (NvmErase(eraseAddr, eraseLen) == BLT_FALSE) {
		/*XcpSetCtoError(XCP_ERR_GENERIC);*/
		return BLT_FALSE;
	}

	wasFwWipeout = true;

	// disable the green LED to indicate the successful erase
	efiSetPadMode("green", getRunningLedPin(), PAL_MODE_INPUT);

	// send the 'connect' packet to stay in the bootloader mode
	data[0] = XCP_CMD_CONNECT;
	data[1] = 0; // connectMode
	*len = 2;
	return BLT_TRUE;
}

/************************************************************************************//**
** \brief     Receives a communication interface packet if one is present.
** \param     data Pointer to byte array where the data is to be stored.
** \param     len Pointer where the length of the packet is to be stored.
** \return    BLT_TRUE is a packet was received, BLT_FALSE otherwise.
**
****************************************************************************************/
extern "C" blt_bool CanReceivePacket(blt_int8u *data, blt_int8u *len)
{
	CANRxFrame frame;

	if (MSG_OK != canReceiveTimeout(&OPENBLT_CAND, CAN_ANY_MAILBOX, &frame, TIME_IMMEDIATE)) {
		// no message was waiting
		return BLT_FALSE;
	}

	// todo: here we assume that all bench_test_packet_ids_e are extended IDs, but it's not guaranteed
	const blt_int32u canIdWipeout = (blt_int32u)(bench_test_packet_ids_e::FW_WIPE_OUT) | 0x80000000;
	if (CanCheckPacketId(frame, canIdWipeout)) {
		return CanRequestFwWipeoutPacket(frame, data, len);
	}

	if (CanCheckPacketId(frame, BOOT_COM_CAN_RX_MSG_ID)) {
		// Copy data and length out
		*len = frame.DLC;
		memcpy(data, frame.data8, frame.DLC);

		return BLT_TRUE;
	}
	return BLT_TRUE;
}
