//
// Created by jwscoggins on 4/11/21.
//

#ifndef RAMBLOCK_W25Q32JV_H
#define RAMBLOCK_W25Q32JV_H
#include "RAM.h"
#include "SPIBus.h"
class W25Q32JV : public RAM {
public:
    enum class Opcodes : uint8_t {
        Read = 0x03,
        FastRead = 0x0B,
        FastReadQuad = 0xEB,
        Write = 0x02,
        QuadWrite = 0x38,
        EnterQuadMode = 0x35,
        ExitQuadMode = 0xF5,
        ResetEnable = 0x66,
        Reset = 0x99,
        SetBurstLength = 0xC0,
        ReadID = 0x9F,
    };
    static constexpr uint32_t Size = 4 * static_cast<uint32_t>(1024) * static_cast<uint32_t>(1024);
public:
    constexpr W25Q32JV(uint32_t startAddress, SPIBusDevice id) : RAM(startAddress, Size), busId_(id) { }
    ~W25Q32JV() override = default;
    constexpr auto getBusID() const noexcept { return busId_; }
protected:
    uint8_t read8(uint32_t address) override;
    uint16_t read16(uint32_t address) override;
    void write8(uint32_t address, uint8_t value) override;
    void write16(uint32_t address, uint16_t value) override;
private:
    SPIBusDevice busId_;
};

#endif //RAMBLOCK_W25Q32JV_H
