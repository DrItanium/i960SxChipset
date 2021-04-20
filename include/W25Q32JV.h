//
// Created by jwscoggins on 4/11/21.
//

#ifndef RAMBLOCK_W25Q32JV_H
#define RAMBLOCK_W25Q32JV_H
#include "RAM.h"
#include <SPI.h>
template<i960Pinout enable>
class W25Q32JV : public RAM {
public:
    enum class Opcodes : uint8_t {
        WriteEnable = 0x06,
        WriteEnableForVolatileStatusRegister = 0x50,
        WriteDisable = 0x04,
        ReadStatusRegister1 = 0x05,
        ReadStatusRegister2 = 0x35,
        ReadStatusRegister3 = 0x15,
        WriteStatusRegister1 = 0x01,
        WriteStatusRegister2 = 0x31,
        WriteStatusRegister3 = 0x11,
        Read = 0x03,
        FastRead = 0x0b,
        FastReadDualOutput = 0x3b,
        FastReadQuadOutput = 0x6b,
        FastReadDualIO = 0xBB,
        FastReadQuadIO = 0xEB,
        SetBurstWithWrap = 0x77,
        PageProgram = 0x02,
        QuadInputPageProgram = 0x32,
        SectorErase = 0x20,
        BlockErase32KB = 0x52,
        BlockErase64KB = 0xD8,
        ChipErase0 = 0xC7,
        ChipErase1 = 0x60,
        Erase_ProgramSuspend = 0x75,
        Erase_ProgramResume = 0x7A,
        PowerDown = 0xB9,
        ReleasePowerDown_DeviceID = 0xAB,
        ReadManufacturer_DeviceID = 0x90,
        ReadManufacturer_DeviceIDDualIO = 0x92,
        ReadManufacturer_DeviceIDQuadIO = 0x94,
        ReadUniqueIDNumber = 0x4B,
        ReadJEDECID = 0x9F,
        ReadSFDPRegister = 0x5A,
        EraseSecurityRegisters = 0x44,
        ProgramSecurityRegisters = 0x42,
        ReadSecurityRegisters = 0x48,
        IndividualBlockSectorLock = 0x36,
        IndividualBlockSectorUnlock = 0x39,
        ReadBlockSectorLock = 0x3D,
        GlobalBlock_SectorLock = 0x7E,
        GlobalBlock_SectorUnlock = 0x98,
        EnableReset = 0x66,
        ResetDevice = 0x99,
    };
    static constexpr uint32_t Size = 4 * static_cast<uint32_t>(1024) * static_cast<uint32_t>(1024);
public:
    constexpr W25Q32JV(uint32_t startAddress) : RAM(startAddress, Size) { }
    ~W25Q32JV() override = default;
    void begin() override {
        SPI.begin();
        pinMode(enable, OUTPUT);
        digitalWrite(enable, HIGH);
    }
protected:
    uint8_t read8(uint32_t address) override {
        byte a = static_cast<byte>(address >> 16);
        byte b = static_cast<byte>(address >> 8);
        byte c = static_cast<byte>(address);
        HoldPinLow <enable> transaction;
        SPI.transfer(static_cast<byte>(Opcodes::Read));
        SPI.transfer(a);
        SPI.transfer(b);
        SPI.transfer(c);
        return SPI.transfer(0x00);
    }
    uint16_t read16(uint32_t address) override {
        byte a = static_cast<byte>(address >> 16);
        byte b = static_cast<byte>(address >> 8);
        byte c = static_cast<byte>(address);
        HoldPinLow <enable> transaction;
        SPI.transfer(static_cast<byte>(Opcodes::Read));
        SPI.transfer(a);
        SPI.transfer(b);
        SPI.transfer(c);
        return SPI.transfer16(0x00);
    }
    void write8(uint32_t address, uint8_t value) override {
        // Do not allow writing to this flash module at runtime, it must be programmed offline
        // thus do nothing!
    }
    void write16(uint32_t address, uint16_t value) override {
        // Do not allow writing to this flash module at runtime, it must be programmed offline
        // thus do nothing!
    }
};

#endif //RAMBLOCK_W25Q32JV_H
