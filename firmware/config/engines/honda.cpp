#include "pch.h"
#include "custom_engine.h"

// set engine_type 57
void setHondaCivicBcm() {
    setBodyControlUnit();
    engineConfiguration->totalGearsCount = 6;
    engineConfiguration->gearRatio[0] = 8;
    engineConfiguration->gearRatio[1] = 5.2;
    engineConfiguration->gearRatio[2] = 3.9;
    engineConfiguration->gearRatio[3] = 2.8;
    engineConfiguration->gearRatio[4] = 2.2;
    engineConfiguration->gearRatio[5] = 1.8;

#if HW_SMALL_CAN_BOARD
strncpy(config->luaScript, R"(
-- this controls onCanRx rate as well!
setTickRate(300)

timeout = 3000

rpmSensor = Sensor.new("rpm")
rpmSensor : setTimeout(timeout)

ppsSensor = Sensor.new("AcceleratorPedal")
ppsSensor : setTimeout(timeout)

speedSensor = Sensor.new("VehicleSpeed")
speedSensor : setTimeout(timeout)

hexstr = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, "A", "B", "C", "D", "E", "F" }

function getTwoBytesMSB(data, offset, factor)
	return (data[offset + 1] * 256 + data[offset + 2]) * factor
end

function getTwoBytesLSB(data, offset, factor)
	return (data[offset + 2] * 256 + data[offset + 1]) * factor
end

function onPOWERTRAIN_DATA(bus, id, dlc, data)
    ppsValue = data[1] * 100.0 / 255
    ppsSensor : set(ppsValue)

    rpmValue = getTwoBytesMSB(data, 2, 1)
    rpmSensor : set(rpmValue)
--     print('onPOWERTRAIN_DATA ' .. rpmValue .. ' pedal ' .. ppsValue)
end

function onCAR_SPEED(bus, id, dlc, data)
    speedKph = getTwoBytesLSB(data, 1, 0.01)
    print('onCAR_SPEED ' .. speedKph)
    speedSensor : set(speedKph)

    print('onPOWERTRAIN_DATA speed' .. speedKph .. ' ratio ' .. (speedKph / rpmValue))

end

canRxAdd(1, 0x17C, onPOWERTRAIN_DATA)
canRxAdd(1, 0x309, onCAR_SPEED)
)", efi::size(config->luaScript));
#endif // HW_SMALL_CAN_BOARD
}
