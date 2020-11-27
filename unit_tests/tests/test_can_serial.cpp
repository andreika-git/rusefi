/*
 * @file test_can_serial.cpp
 *
 *  Created on: Nov 26, 2020
 * @author andreika <prometheus.pcb@gmail.com>
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include <list>
#include <string>

#include "engine_test_helper.h"
#include "serial_can.h"


using namespace std::string_literals;

class TestCanStreamer : public ICanStreamer {
public:
	virtual can_msg_t transmit(canmbx_t mailbox, const CANTxFrame *ctfp, can_sysinterval_t timeout) override {
		ctfList.emplace_back(*ctfp);
		return CAN_MSG_OK;
	}

	virtual can_msg_t receive(canmbx_t mailbox, CANRxFrame *crfp, can_sysinterval_t timeout) override {
		*crfp = *crfList.begin();
		crfList.pop_front();
		return CAN_MSG_OK;
	}

	template<typename T>
	void checkFrame(const T & frame, const std::string & bytes) {
		EXPECT_EQ(bytes.size(), frame.DLC);
		for (size_t i = 0; i < bytes.size(); i++) {
  			EXPECT_EQ(bytes[i], frame.data8[i]) << "Frame byte #" << i << " differs!";
		}
	}

public:
	std::list<CANTxFrame> ctfList;
	std::list<CANRxFrame> crfList;
};

class TestCanStreamerState : public CanStreamerState {
public:
	TestCanStreamerState() : CanStreamerState(&streamer) {}

	void test(const std::vector<std::string> & dataList, const std::vector<std::string> & frames, int fifoLeftoverSize) {
		for (auto data : dataList) {
			size_t np = data.size();
			streamAddToTxTimeout(&np, (uint8_t *)data.c_str(), 0);
		}
		
		// check the FIFO buf size
		EXPECT_EQ(fifoLeftoverSize, txFifoBuf.getCount());

		// send the rest
		streamFlushTx(0);

		// check if correct the TX frames were sent
		EXPECT_EQ(frames.size(), streamer.ctfList.size());
		
		auto it1 = streamer.ctfList.begin();
		auto it2 = frames.begin();
		for (; it1 != streamer.ctfList.end() && it2 != frames.end(); it1++, it2++) {
			streamer.checkFrame(*it1, *it2);
		}
	
		//state.streamReceiveTimeout(np, rxbuf, timeout);
	}

protected:
	TestCanStreamer streamer;
};

TEST(testCanSerial, test1Frame) {
	{
		TestCanStreamerState state;
		state.test({ "1" }, { "\x01"s "1\0\0\0\0\0\0"s }, 1); // 1 byte -> 1 frame, 1 byte in FIFO
	}
	{
		TestCanStreamerState state;
		state.test({ "0123456" }, { "\x07"s "0123456"s }, 0); // 7 bytes -> 1 8-byte frame
	}
	{
		TestCanStreamerState state;
		state.test({ "0", "1", "2", "3", "4", "5", "6" }, { "\x07"s "0123456"s }, 0); // 7 bytes separately -> 1 8-byte frame
	}
}

TEST(testCanSerial, test2Frames) {
	{
		TestCanStreamerState state;
		state.test({ "01234567" }, { "\x07"s "0123456"s, "\x01"s "7\0\0\0\0\0\0"s }, 1); // 8 bytes -> 2 8-byte frames, 1 byte in FIFO
	}
	{
		TestCanStreamerState state;
		state.test({ "0123456ABCDEFG" }, { "\x07"s "0123456"s, "\x07"s "ABCDEFG"s }, 0); // 14 bytes -> 2 8-byte frames, empty FIFO
	}
}

TEST(testCanSerial, testIrregularSplits) {
	{
		TestCanStreamerState state;
		state.test({ "012", "3456ABCDEFG" }, { "\x07"s "0123456"s, "\x07"s "ABCDEFG"s }, 0); // 14 bytes -> 2 8-byte frames, empty FIFO
	}
	{
		TestCanStreamerState state;
		state.test({ "0123456ABC", "DEFG" }, { "\x07"s "0123456"s, "\x07"s "ABCDEFG"s }, 0); // 14 bytes -> 2 8-byte frames, empty FIFO
	}
}

TEST(testCanSerial, testLongMessage) {
	{
		TestCanStreamerState state;
		state.test({ "abcdefghijklmnopqrstuvwxyz" }, {
			"\x07"s "abcdefg"s,
			"\x07"s "hijklmn"s,
			"\x07"s "opqrstu"s,
			"\x05"s "vwxyz\0\0"s }, 5); // 26 bytes -> 4 8-byte frames, 5 bytes left in FIFO
	}
}
