/*
 * @file test_executor.h
 * implementation of ExecutorInterface used by unit tests
 *
 *  Created on: Jan 9, 2019
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "scheduler.h"
#include "event_queue.h"

class TestExecutor : public ExecutorInterface {
public:
	~TestExecutor();

	void scheduleByTimestampNt(const char *msg, scheduling_s *scheduling, efitick_t timeNt, action_s action) override;
	void cancel(scheduling_s* scheduling) override;

	void clear();
	int executeAll(efitimeus_t nowUs);
	int size();
	scheduling_s * getHead();
	scheduling_s * getForUnitTest(int index);

	void setMockExecutor(ExecutorInterface* exec);
private:
	EventQueue schedulingQueue;
	ExecutorInterface* m_mockExecutor = nullptr;
};
