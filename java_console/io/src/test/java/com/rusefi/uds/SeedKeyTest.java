package com.rusefi.uds;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SeedKeyTest {
    @Test
    public void testAssert() {
        assertEquals(0xC42F15AE, SeedKeyCalculator.Uds_Security_CalcKey(SeedKeyCalculator.BOOTLOADER_SECRET, 0x5D8A2010, 0xF0));
        assertEquals(0x001B6F78, SeedKeyCalculator.Uds_Security_CalcKey(SeedKeyCalculator.SECRET, 0x5DA0B808, 0xA4));
    }
}
