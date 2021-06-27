/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef I960SXCHIPSET_PSRAMBLOCK_H
#define I960SXCHIPSET_PSRAMBLOCK_H
#include <SPI.h>
#include "MCUPlatform.h"
#include "MemoryThing.h"
#include "ProcessorSerializer.h"
#include "Pinout.h"
/**
 * @brief Represents a block of 64 Megabytes of PSRAM chips spread across eight separate devices, it is described using an 8-bit address on the SPI Bus.
 * The lowest three bits are always consumed by the eight devices, the upper five bits are used to select it. You can have up to 32 blocks on
 * the spi bus (although that would preclude adding any other devices to the bus). This is separate to how it is mapped into the i960 memory space
 */
template<byte blockAddress, i960Pinout enable = i960Pinout::SPI_BUS_EN>
class PSRAMBlock : public MemoryThing {
public:
    static const SPISettings& getSPISettings() noexcept {
        static SPISettings tmp(8_MHz, MSBFIRST, SPI_MODE0);
        return tmp;
    }
    enum class Opcode : byte {
        ResetEnable = 0x66,
        Reset = 0x99,
        Write = 0b0000'0010,
        Read = 0b0000'0011,
    };
    static_assert (blockAddress < 32, "Can only have up to 32 different PSRAMBlocks on the SPI Bus");
    static constexpr auto EnablePin = enable;
    using EnablePinManager = PinAsserter<EnablePin>;
    static constexpr byte BlockAddressID = blockAddress;
    static constexpr byte BlockBaseAddress = blockAddress << 3;
    static constexpr Address TotalCapacity = 64_MB;
    static constexpr Address NumberOfChipsPerBlock = 8;
    static constexpr Address CapacityPerChip = TotalCapacity / NumberOfChipsPerBlock;
    static constexpr Address BlockAddressMask = TotalCapacity - 1;
    static constexpr Address PerChipAddressMask = CapacityPerChip - 1;
    static_assert(CapacityPerChip == 8_MB, "CAPACITY PER DEVICE SANITY CHECK FAILED");
    static_assert(CapacityPerChip == 0x80'0000, "EQUIVALENCE FOR CAPACITY SANITY CHECK FAILED");
    using Parent = MemoryThing;
    explicit PSRAMBlock(Address baseAddress) noexcept : Parent(baseAddress, baseAddress + TotalCapacity) { }
    ~PSRAMBlock() override = default;
    void begin() noexcept override {
        /// @todo align to 16-byte addresses
        for (int i = 0; i < 8; ++i) {
            setCurrentPSRAMBlock(id);
            sendResetEnable();
            sendReset();
        }
    }
    // all of these overrides are written from the perspective that the mcu will only ever be interfacing with a single psram chip at a time
    // thus all of the internal routines are written assuming that the work has already been done to setup the spi bus correctly
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        // divide up the writes into writes of 8 bytes to prevent data loss
        return 0;
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
        // divide up the reads into reads of 8 bytes to prevent data loss
        return 0;
    }
    uint8_t read8(Address address) noexcept override {
        // setup the command stream
        byte commandStream[] = {
           static_cast<byte>(Opcode::Read),
           static_cast<byte>(address >> 16),
           static_cast<byte>(address >> 8),
           static_cast<byte>(address),
            0, // storage cell
        };
        performSPITransaction(commandStream, 5);
        return commandStream[4];
    }
    uint16_t read16(Address address) noexcept override {
        return MemoryThing::read16(address);
    }
    void write8(Address address, uint8_t value) noexcept override {
        MemoryThing::write8(address, value);
    }
    void write16(Address address, uint16_t value) noexcept override {
        MemoryThing::write16(address, value);
    }
    /**
     * @brief Make the address relative to a single block of memory and setup the spi bus device
     * @param input The address to modify
     * @return The masked address relative to a single device
     */
    [[nodiscard]] Address makeAddressRelative(Address input) const noexcept override {
        // first, set the appropriate device id
        setCurrentPSRAMBlockFromAddress(input);
        // then return the address into the block we have selected
        return getOffsetAddress(input);
    }
private:
    void performSPITransaction(byte* command, size_t length) const noexcept {
        SPI.beginTransaction(getSPISettings());
        EnablePinManager trigger;
        SPI.transfer(command, length);
        SPI.endTransaction();
    }
    static constexpr byte getDeviceId(Address targetAddress) noexcept {
        return (targetAddress >> 23) & 0b111;
    }
    static constexpr Address getOffsetAddress(Address targetAddress) noexcept {
        return (targetAddress & PerChipAddressMask);
    }
    static constexpr Address getInternalBlockAddress(Address targetAddress) noexcept {
        return targetAddress & BlockAddressMask;
    }
    // sanity checks that getDeviceId works as intended
    static_assert(getDeviceId(PerChipAddressMask*1) == 0);
    static_assert(getDeviceId(PerChipAddressMask*2) == 1);
    static_assert(getDeviceId(PerChipAddressMask*3) == 2);
    static_assert(getDeviceId(PerChipAddressMask*4) == 3);
    static_assert(getDeviceId(PerChipAddressMask*5) == 4);
    static_assert(getDeviceId(PerChipAddressMask*6) == 5);
    static_assert(getDeviceId(PerChipAddressMask*7) == 6);
    static_assert(getDeviceId(PerChipAddressMask*8) == 7);
    static void setCurrentPSRAMBlock(byte id) noexcept {
        ProcessorInterface::getInterface().setSPIBusIndex(BlockBaseAddress + (id & 0b111));
    }
    static void setCurrentPSRAMBlockFromAddress(Address targetAddress) noexcept {
        setCurrentPSRAMBlock(getDeviceId(targetAddress));
    }
    void sendResetEnable() const noexcept {
        SPI.beginTransaction(getSPISettings());
        EnablePinManager trigger;
        SPI.transfer(static_cast<byte>(Opcode::ResetEnable));
        SPI.endTransaction();
    }
    void sendReset() const noexcept {
        SPI.beginTransaction(getSPISettings());
        EnablePinManager trigger;
        SPI.transfer(static_cast<byte>(Opcode::Reset));
        SPI.endTransaction();
    }

};

#endif //I960SXCHIPSET_PSRAMBLOCK_H
