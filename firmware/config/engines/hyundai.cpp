/*
 * @file hyundai.cpp
 *
 * @date Oct 11, 2021
 * @author Andrey Belomutskiy, (c) 2012-2021
 */

#include "pch.h"

#include "hyundai.h"
#include "proteus_meta.h"
#include "defaults.h"

static void set201xHyundai() {
	setPPSCalibration(0.73, 4.0, 0.34, 1.86);

    // note how these numbers are very flipped m111 defaults?
    setTPS1Calibration(98, 926, 891, 69);
}

void setHyundaiPb() {
	engineConfiguration->cylindersCount = 4;
	engineConfiguration->firingOrder = FO_1_3_4_2;
	engineConfiguration->displacement = 1.6;
	strcpy(engineConfiguration->engineMake, ENGINE_MAKE_Hyundai);
	strcpy(engineConfiguration->engineCode, "Gamma");

	engineConfiguration->trigger.type = trigger_type_e::TT_TOOTHED_WHEEL_60_2;
	engineConfiguration->globalTriggerAngleOffset = 90;

	set201xHyundai();
	// Injectors flow 1214 cc/min at 100 bar pressure
	engineConfiguration->injector.flow = 1214;
	setGDIFueling();
	engineConfiguration->injectionMode = IM_SEQUENTIAL;
   	engineConfiguration->crankingInjectionMode = IM_SEQUENTIAL;

	engineConfiguration->map.sensor.type = MT_CUSTOM;
	engineConfiguration->map.sensor.lowValue = 20;
	engineConfiguration->mapLowValueVoltage = 0.79;
	engineConfiguration->map.sensor.highValue = 101.3;
	engineConfiguration->mapHighValueVoltage = 4;

	engineConfiguration->vvtMode[0] = VVT_SECOND_HALF;
	engineConfiguration->vvtMode[1] = VVT_SECOND_HALF;

    engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;

    engineConfiguration->hpfpCamLobes = 4;

   	engineConfiguration->highPressureFuel.v1 = 0.5; /* volts */;
   	engineConfiguration->highPressureFuel.value1 = 0;
   	engineConfiguration->highPressureFuel.v2 = 4.5; /* volts */;
   	// page 98, Fuel System > Engine Control System > Rail Pressure Sensor (RPS) > Specifications
   	engineConfiguration->highPressureFuel.value2 = 20'000;
	engineConfiguration->highPressureFuel.hwChannel = PROTEUS_IN_ANALOG_VOLT_4;

#if HW_PROTEUS
	gppwm_channel *coolantControl = &engineConfiguration->gppwm[0];
	coolantControl->pin = PROTEUS_LS_5;
    engineConfiguration->tachOutputPin = PROTEUS_IGN_12;
    engineConfiguration->hpfpValvePin = PROTEUS_LS_6;
#endif // HW_PROTEUS
}

void setProteusHyundaiPb() {
    setHyundaiPb();
	engineConfiguration->triggerInputPins[0] = PROTEUS_VR_1;
	engineConfiguration->camInputs[0] = PROTEUS_DIGITAL_1;
	engineConfiguration->camInputs[1] = PROTEUS_DIGITAL_2;

	engineConfiguration->tps1_2AdcChannel = PROTEUS_IN_TPS1_2;
	setPPSInputs(PROTEUS_IN_PPS, PROTEUS_IN_PPS2);


// something something input levels are not happy for digital input pin?
	engineConfiguration->starterControlPin = PROTEUS_LS_14;
	engineConfiguration->startStopButtonPin = PROTEUS_IN_AV_6_DIGITAL;
	engineConfiguration->startStopButtonMode = PI_DEFAULT;
}

static void commonGenesisCoupe() {
	set201xHyundai();

	strncpy(config->luaScript, R"(

setTickRate(100)
t = Timer.new()
t : reset()

local data_0x329 = { 0x2C, 0x96, 0x80, 0x0E, 0x11, 0x2E, 0x00, 0x14 }
local slowCounter = 0
local slowRoll = 0

local slowRollTable = { 0x0C, 0x4F, 0x80, 0xE3 }

-- wakeup CAN messages
local data_0x382 = { 0x00, 0x3A, 0X44, 0x24, 0x00, 0x00, 0x00, 0x00 }
local data_0x0a0 = { 0x00, 0x68, 0x00, 0x00, 0x00, 0xFF, 0x01, 0x00 }
local data_0x0a1 = { 0x80, 0x80, 0x00, 0x00, 0xEC, 0x00, 0x00, 0x00 }
local data_0x18f = { 0xFA , 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
local data_0x545 = { 0xEC, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00 }
local data_0xA1 = { 0x80, 0x80, 0x00, 0x00, 0xEC, 0x00, 0x00, 0x00 }
local data_0xA0 = { 0x00, 0x68, 0x00, 0x00, 0x00, 0xFF, 0x01, 0x00 }
local data_0x260 = { 0x01, 0x1A, 0x1A, 0x10, 0x00, 0xAB, 0x93, 0x00 }
local data_0x2A0 = { 0x00, 0x00, 0x04, 0x00, 0x78, 0x05, 0x0D, 0x01 }

function onTick()

	-- keep alives
	txCan(1, 0x18f, 0, data_0x18f)
	txCan(1, 0x545, 0, data_0x545)
	-- more random can sends
	txCan(1, 0x545, 0, data_0x545)
	txCan(1, 0x382, 0, data_0x382)
	txCan(1, 0xA0, 0, data_0xA0)
	txCan(1, 0xA1, 0, data_0xA1)
	txCan(1, 0x260, 0, data_0x260)
	txCan(1, 0x2A0, 0, data_0x2A0)

	local RPMread = math.floor(getSensor("RPM") + 0.5) * 4

	local RPMhi = RPMread >> 8
	local RPMlo = RPMread & 0xff

	local CLTread = 50
	if getSensor("CLT") then
		CLTread = math.floor(getSensor("CLT") + 0.5)
	else
		CLTread = 50
	end

	local CLThi = CLTread
	local CLTlo = CLTread * 256

	canCLTpayloadNo = { 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
	canCLTpayloadLo = { 0x00, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
	canCLTpayloadHi = { 0x00, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
	canCLTpayloadHi2 = { 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

	if slowCounter == 0 then
		-- cycle through 0, 40, 80, c0
		slowRoll = (slowRoll + 1) & 3
		data_0x329[1] = slowRollTable[slowRoll + 1]

		txCan(1, 0x329, 0, data_0x329)
		slowCounter = 30
	end
	slowCounter = slowCounter -1

	-- rpm fun stuff
	if t : getElapsedSeconds() < 1.5 then
		canRPMpayload = { 0x05, 0x1C, 0xFC, 0x7B, 0x1C, 0x2F, 0x00, 0x80 }
	else
		canRPMpayload = { 0x05, RPMlo, RPMhi, RPMhi, RPMlo, 0x2F, 0x00, 0x80 }
	end

	txCan(1, 0x316, 0, canRPMpayload)
	if CLTread < 60 then
		txCan(1, 0x608, 0, canCLTpayloadNo)
	elseif CLTread >= 60 and CLTread < 85 then
		txCan(1, 0x608, 0, canCLTpayloadLo)
	elseif CLTread >= 85 and CLTread < 110 then
		txCan(1, 0x608, 0, canCLTpayloadHi)
	else
		txCan(1, 0x608, 0, canCLTpayloadHi2)
	end
end


)", efi::size(config->luaScript));
}

void setGenesisCoupeBK1() {
	commonGenesisCoupe();
	// 2010-2012 315cc at 43.5psi
	// fuel system is fixed pressure 55psi
	engineConfiguration->injector.flow = 354.19; // https://www.google.com/search?q=315*sqrt%2855%2F43.5%29
	engineConfiguration->fuelReferencePressure = PSI2KPA(55);

}

void setGenesisCoupeBK2() {
	commonGenesisCoupe();
	// 2013+ 450cc at 43.5
	// fuel system is fixed pressure 85psi
	// flow rate P2 = flow rate P1 * sqrt(P2/P1)
	engineConfiguration->injector.flow = 629.03; // https://www.google.com/search?q=450*sqrt%2885%2F43.5%29
	engineConfiguration->fuelReferencePressure = PSI2KPA(85);

}
