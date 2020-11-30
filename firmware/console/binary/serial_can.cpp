/**
 * @file	serial_can.cpp
 *
 * This code is a bridge between a serial streaming used by TS and a packet-frame CAN-bus, using the ISO-TP protocol.
 * ISO 15765-2, or ISO-TP (Transport Layer), which is an international standard for sending data packets over a CAN-Bus.
 * https://en.wikipedia.org/wiki/ISO_15765-2
 *
 * @date Aug 1, 2020
 * @author andreika <prometheus.pcb@gmail.com>
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "global.h"
#include "os_access.h"
#include "crc.h"
#include "serial_can.h"

#if HAL_USE_CAN
static CanStreamer streamer;
static CanStreamerState state(&streamer);
#endif /* HAL_USE_CAN */

int CanStreamerState::sendFrame(const IsoTpFrameHeader & header, const uint8_t *data, int num, can_sysinterval_t timeout) {
	CANTxFrame txmsg;
	memset(&txmsg, 0, sizeof(txmsg));
	txmsg.IDE = CAN_IDE_STD;
	txmsg.EID = CAN_TX_ID;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = 8;	// 8 bytes
	
	// fill the frame data according to the CAN-TP protocol (ISO 15765-2)
	txmsg.data8[0] = (uint8_t)((header.frameType & 0xf) << 4);
	int offset, maxNumBytes;
	switch (header.frameType) {
	case ISO_TP_FRAME_SINGLE:
		offset = 1;
		maxNumBytes = minI(header.numBytes, txmsg.DLC - offset);
		txmsg.data8[0] |= maxNumBytes;
		break;
	case ISO_TP_FRAME_FIRST:
		txmsg.data8[0] |= (header.numBytes >> 8) & 0xf;
		txmsg.data8[1] = (uint8_t)(header.numBytes & 0xff);
		offset = 2;
		maxNumBytes = minI(header.numBytes, txmsg.DLC - offset);
		break;
	case ISO_TP_FRAME_CONSECUTIVE:
		txmsg.data8[0] |= header.index & 0xf;
		offset = 1;
		maxNumBytes = txmsg.DLC - offset;
		break;
	case ISO_TP_FRAME_FLOW_CONTROL:
		txmsg.data8[0] |= header.fcFlag & 0xf;
		txmsg.data8[1] = (uint8_t)(header.blockSize);
		txmsg.data8[2] = (uint8_t)(header.separationTime);
		offset = 3;
		maxNumBytes = 0;	// no data is sent with 'flow control' frame
		break;
	}
	
	int numBytes = minI(maxNumBytes, num);
	// copy the contents
	if (data != nullptr) {
		for (int i = 0; i < numBytes; i++) {
			txmsg.data8[i + offset] = data[i];
		}
	}
	
	// send the frame!
	if (streamer->transmit(CAN_ANY_MAILBOX, &txmsg, timeout) == CAN_MSG_OK)
		return numBytes;
	return 0;
}

// returns the number of copied bytes
int CanStreamerState::receiveFrame(CANRxFrame *rxmsg, uint8_t *buf, int num, can_sysinterval_t timeout) {
	if (rxmsg == nullptr || rxmsg->DLC < 1)
		return 0;
	int frameType = (rxmsg->data8[0] >> 4) & 0xf;
	int numBytesAvailable, frameIdx;
	uint8_t *srcBuf = rxmsg->data8;
	switch (frameType) {
	case ISO_TP_FRAME_SINGLE:
		numBytesAvailable = rxmsg->data8[0] & 0xf;
		srcBuf = rxmsg->data8 + 1;
		this->waitingForNumBytes = -1;
		break;
	case ISO_TP_FRAME_FIRST:
		this->waitingForNumBytes = ((rxmsg->data8[0] & 0xf) << 8) | rxmsg->data8[1];
		this->waitingForFrameIndex = 1;
		numBytesAvailable = minI(this->waitingForNumBytes, 6);
		srcBuf = rxmsg->data8 + 2;
		break;
	case ISO_TP_FRAME_CONSECUTIVE:
		frameIdx = rxmsg->data8[0] & 0xf;
		if (this->waitingForNumBytes < 0 || this->waitingForFrameIndex != frameIdx) {
			// todo: that's an abnormal situation, and we probably should react?
			return 0;
		}
		numBytesAvailable = minI(this->waitingForNumBytes, 7);
		srcBuf = rxmsg->data8 + 1;
		this->waitingForFrameIndex = (this->waitingForFrameIndex + 1) & 0xf;
		break;
	case ISO_TP_FRAME_FLOW_CONTROL:
		// todo: currently we just ignore the FC frame
		return 0;
	}

#if defined(TS_CAN_DEVICE_SHORT_PACKETS_IN_ONE_FRAME)
	if (frameType == ISO_TP_FRAME_SINGLE) {
		srcBuf = tmpRxBuf;
		// restore the CRC on the whole packet
		uint32_t crc = crc32((void *) (rxmsg->data8 + 1), numBytesAvailable);
		// we need a separate buffer for crc because srcBuf may not be word-aligned for direct copy
		uint8_t crcBuffer[sizeof(uint32_t)];
		*(uint32_t *) (crcBuffer) = SWAP_UINT32(crc);

		// now set the packet size (including the command byte)
		*(uint16_t *) srcBuf = SWAP_UINT16(numBytesAvailable);
		// copy the data
		if (numBytesAvailable > 0)
			memcpy(srcBuf + 2, rxmsg->data8 + 1, numBytesAvailable);
		// copy the crc
		memcpy(srcBuf + 2 + numBytesAvailable, crcBuffer, sizeof(crcBuffer));
		numBytesAvailable += 1 + sizeof(crcBuffer);	// added command & crc bytes
	}
#endif /* TS_CAN_DEVICE_SHORT_PACKETS_IN_ONE_FRAME */

	int numBytesToCopy = minI(num, numBytesAvailable);
	if (buf != nullptr) {
		memcpy(buf, srcBuf, numBytesToCopy);
	}
	srcBuf += numBytesToCopy;
	numBytesAvailable -= numBytesToCopy;
	waitingForNumBytes -= numBytesToCopy;
	// if there are some more bytes left, we save them for the next time
	for (int i = 0; i < numBytesAvailable; i++) {
		rxFifoBuf.put(srcBuf[i]);
	}

	// according to the specs, we need to acknowledge the received multi-frame start frame
	if (frameType == ISO_TP_FRAME_FIRST) {
		IsoTpFrameHeader header;
		header.frameType = ISO_TP_FRAME_FLOW_CONTROL;
		header.fcFlag = 0;			// = "continue to send"
		header.blockSize = 0;		// = the remaining "frames" to be sent without flow control or delay
		header.separationTime = 0;	// = wait 0 milliseconds, send immediately
		sendFrame(header, nullptr, 0, timeout);
	}

	return numBytesToCopy;
}

int CanStreamerState::sendDataTimeout(const uint8_t *txbuf, int numBytes, can_sysinterval_t timeout) {

	int offset = 0;
	can_msg_t ret;

	if (numBytes < 1)
		return 0;

	// 1 frame
	if (numBytes <= 7) {
		IsoTpFrameHeader header;
		header.frameType = ISO_TP_FRAME_SINGLE;
		header.numBytes = numBytes;
		return sendFrame(header, txbuf, numBytes, timeout);
	}

	// multiple frames

	// send the first header frame
	IsoTpFrameHeader header;
	header.frameType = ISO_TP_FRAME_FIRST;
	header.numBytes = numBytes;
	int numSent = sendFrame(header, txbuf + offset, numBytes, timeout);
	offset += numSent;
	numBytes -= numSent;
	int totalNumSent = numSent;

	// get a flow control frame
	CANRxFrame rxmsg;
	if (streamer->receive(CAN_ANY_MAILBOX, &rxmsg, timeout) == CAN_MSG_OK) {
		receiveFrame(&rxmsg, nullptr, 0, timeout);
	}

	// send the rest of the data
	int idx = 1;
	while (numBytes > 0) {
		int len = minI(numBytes, 7);
		// send the consecutive frames
		IsoTpFrameHeader header;
		header.frameType = ISO_TP_FRAME_CONSECUTIVE;
		header.index = ((idx++) & 0x0f);
		header.numBytes = numBytes;
		int numSent = sendFrame(header, txbuf + offset, numBytes, timeout);
		if (numSent < 1)
			break;
		totalNumSent += numSent;
		offset += numSent;
		numBytes -= numSent;
	}
	return totalNumSent;
}

int CanStreamerState::getDataFromFifo(uint8_t *rxbuf, size_t &numBytes) {
	if (rxFifoBuf.isEmpty())
		return 0;
	int numReadFromFifo = minI(numBytes, rxFifoBuf.getCount());
	// move bytes from the FIFO buffer
	int i;
	for (i = 0; !rxFifoBuf.isEmpty() && i < numReadFromFifo; i++) {
		rxbuf[i] = rxFifoBuf.get();
		numBytes--;
	}
	return i;
}

can_msg_t CanStreamerState::streamAddToTxTimeout(size_t *np, const uint8_t *txbuf, can_sysinterval_t timeout) {
	int numBytes = *np;
	int offset = 0;
	int minNumBytesRequiredToSend = 7 - txFifoBuf.getCount();
	while (numBytes >= minNumBytesRequiredToSend) {
		txFifoBuf.put(txbuf + offset, minNumBytesRequiredToSend);
		int numSent = sendDataTimeout((const uint8_t *)txFifoBuf.getElements(), txFifoBuf.getCount(), timeout);
		if (numSent < 1)
			break;
		txFifoBuf.clear();
		offset += minNumBytesRequiredToSend;
		numBytes -= minNumBytesRequiredToSend;
		minNumBytesRequiredToSend = 7;
	}
	
	// now we put the rest on hold
	txFifoBuf.put(txbuf + offset, numBytes);

	return CAN_MSG_OK;
}

can_msg_t CanStreamerState::streamFlushTx(can_sysinterval_t timeout) {
	int numSent = sendDataTimeout((const uint8_t *)txFifoBuf.getElements(), txFifoBuf.getCount(), timeout);
	txFifoBuf.clear();
	
	return CAN_MSG_OK;
}

can_msg_t CanStreamerState::streamReceiveTimeout(size_t *np, uint8_t *rxbuf, can_sysinterval_t timeout) {
	int i = 0;
	size_t numBytes = *np;
	
	// first, fill the data from the stored buffer (saved from the previous CAN frame)
	i = getDataFromFifo(rxbuf, numBytes);

	// if even more data is needed, then we receive more CAN frames
	while (numBytes > 0) {
#if HAL_USE_CAN
		if (chEvtWaitAnyTimeout(ALL_EVENTS, timeout) == 0)
			return CAN_MSG_TIMEOUT;
#endif /* HAL_USE_CAN */

		CANRxFrame rxmsg;
		if (streamer->receive(CAN_ANY_MAILBOX, &rxmsg, CAN_TIME_IMMEDIATE) == CAN_MSG_OK) {
			int numReceived = receiveFrame(&rxmsg, rxbuf + i, numBytes, timeout);
			if (numReceived < 1)
				break;
			numBytes -= numReceived;
			i += numReceived;
		}
	}
	//*np -= numBytes;
	return CAN_MSG_OK;
}

#if HAL_USE_CAN

void CanStreamer::init(CANDriver *c) {
	canp = c;
	chEvtRegister(&canp->rxfull_event, &el, 0);
}

can_msg_t CanStreamer::transmit(canmbx_t mailbox, const CANTxFrame *ctfp, can_sysinterval_t timeout) {
	return (can_msg_t)canTransmit(canp, mailbox, ctfp, (sysinterval_t)timeout);
}

can_msg_t CanStreamer::receive(canmbx_t mailbox, CANRxFrame *crfp, can_sysinterval_t timeout) {
	return (can_msg_t)canReceive(canp, mailbox, crfp, (sysinterval_t)timeout);
}


void canStreamInit(CANDriver *canp) {
	streamer.init(canp);
}

msg_t canStreamAddToTxTimeout(size_t *np, const uint8_t *txbuf, sysinterval_t timeout) {
	return state.streamAddToTxTimeout(np, txbuf, timeout);
}

msg_t canStreamFlushTx(sysinterval_t timeout) {
	return state.streamFlushTx(timeout);
}

msg_t canStreamReceiveTimeout(size_t *np, uint8_t *rxbuf, sysinterval_t timeout) {
	return state.streamReceiveTimeout(np, rxbuf, timeout);
}

#endif /* HAL_USE_CAN */
