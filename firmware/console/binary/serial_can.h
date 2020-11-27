/**
 * @file	serial_can.h
 *
 * @date Aug 1, 2020
 * @author andreika <prometheus.pcb@gmail.com>
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "fifo_buffer.h"

#if !EFI_UNIT_TEST
#define can_msg_t msg_t
#define can_sysinterval_t sysinterval_t
#define CAN_MSG_OK MSG_OK
#define CAN_MSG_TIMEOUT MSG_TIMEOUT
#else
#include "can_mocks.h"
#endif /* EFI_UNIT_TEST */


#define CAN_TIME_IMMEDIATE ((can_sysinterval_t)0)

#define CAN_FIFO_BUF_SIZE 64

#define CAN_TX_ID 0x102

enum IsoTpFrameType {
	ISO_TP_FRAME_SINGLE = 0,
	ISO_TP_FRAME_FIRST = 1,
	ISO_TP_FRAME_CONSECUTIVE = 2,
	ISO_TP_FRAME_FLOW_CONTROL = 3,
};

class IsoTpFrameHeader {
public:
	IsoTpFrameType frameType;

	// used for 'single' or 'first' frames
	int numBytes;
	// used for 'consecutive' frames
	int index;
	// used for 'flow control' frames
	int fcFlag;
	int blockSize;
	int separationTime;
};

// We need an abstraction layer for unit-testing
class ICanStreamer {
public:
	virtual can_msg_t transmit(canmbx_t mailbox, const CANTxFrame *ctfp, can_sysinterval_t timeout) = 0;
	virtual can_msg_t receive(canmbx_t mailbox, CANRxFrame *crfp, can_sysinterval_t timeout) = 0;
};

class CanStreamerState {
public:
	fifo_buffer<uint8_t, CAN_FIFO_BUF_SIZE> rxFifoBuf;
	fifo_buffer<uint8_t, CAN_FIFO_BUF_SIZE> txFifoBuf;

#if defined(TS_CAN_DEVICE_SHORT_PACKETS_IN_ONE_FRAME)
	// used to restore the original packet with CRC
    uint8_t tmpRxBuf[13];
#endif

	// used for multi-frame ISO-TP packets
	int waitingForNumBytes = 0;
	int waitingForFrameIndex = 0;

	ICanStreamer *streamer;
	
public:
	CanStreamerState(ICanStreamer *s) : streamer(s) {}

	int sendFrame(const IsoTpFrameHeader & header, const uint8_t *data, int num, can_sysinterval_t timeout);
	int receiveFrame(CANRxFrame *rxmsg, uint8_t *buf, int num, can_sysinterval_t timeout);
	int getDataFromFifo(uint8_t *rxbuf, size_t &numBytes);
	// returns the number of bytes sent
	int sendDataTimeout(const uint8_t *txbuf, int numBytes, can_sysinterval_t timeout);

	// streaming support for TS I/O (see tunerstudio_io.cpp)
	can_msg_t streamAddToTxTimeout(size_t *np, const uint8_t *txbuf, can_sysinterval_t timeout);
	can_msg_t streamFlushTx(can_sysinterval_t timeout);
	can_msg_t streamReceiveTimeout(size_t *np, uint8_t *rxbuf, can_sysinterval_t timeout);
};


#if HAL_USE_CAN
// The implementation using the ChibiOS CAN driver
class CanStreamer : public ICanStreamer {
public:
	void init(CANDriver *c);

	virtual can_msg_t transmit(canmbx_t mailbox, const CANTxFrame *ctfp, can_sysinterval_t timeout) override;
	virtual can_msg_t receive(canmbx_t mailbox, CANRxFrame *crfp, can_sysinterval_t timeout) override;

protected:
	CANDriver *canp;
	event_listener_t el;
};

void canStreamInit(CANDriver *canp);

// we don't have canStreamSendTimeout() because we need to "bufferize" the stream and send it in fixed-length packets
msg_t canStreamAddToTxTimeout(size_t *np, const uint8_t *txbuf, sysinterval_t timeout);
msg_t canStreamFlushTx(sysinterval_t timeout);

msg_t canStreamReceiveTimeout(size_t *np, uint8_t *rxbuf, sysinterval_t timeout);
#endif /* HAL_USE_CAN */
