//
// Created by jwscoggins on 7/31/21.
//

#ifndef I960SXCHIPSET_PSRAMCHIP_H
#define I960SXCHIPSET_PSRAMCHIP_H
#include <Arduino.h>
#include <SPI.h>
#include "MCUPlatform.h"
#include "Pinout.h"
#include "MemoryThing.h"
#include "TaggedCacheAddress.h"
/**
 * @brief Interface to the memory connected to the chipset
 * @tparam enablePin The pin that is used to signal chip usage
 * @tparam sel0 The lowest pin used to select the target device
 * @tparam sel1 The middle pin used to select the target device
 * @tparam sel2 The upper pin used to select the target device
 */
template<i960Pinout enablePin>
class MemoryBlock {
public:
    static constexpr auto EnablePin = enablePin;
    static constexpr auto Select0 = i960Pinout::SPI_OFFSET0;
    static constexpr auto Select1 = i960Pinout::SPI_OFFSET1;
    static constexpr auto Select2 = i960Pinout::SPI_OFFSET2;
    static constexpr auto NumChips = 8;
    static_assert ((EnablePin != Select0) && (EnablePin != Select1) && (EnablePin != Select2), "The enable pin must be different from all select pins");
    static_assert ((Select0 != Select1) && (Select0 != Select2) && (Select1 != Select2), "All three select pins must point to a different physical pin");
public:
    MemoryBlock() = delete;
    ~MemoryBlock() = delete;
    union PSRAMBlockAddress {
        constexpr explicit PSRAMBlockAddress(Address value = 0) : base(value) { }
        constexpr auto getAddress() const noexcept { return base; }
        constexpr auto getOffset() const noexcept { return offset; }
        constexpr auto getIndex() const noexcept { return index; }
        Address base;
        struct {
            Address offset : 23;
            byte index : 3;
        };
        byte bytes_[4];
    };
private:
    enum class OperationKind {
        Write,
        Read,
        Generic,
    };

    template<byte opcode, OperationKind kind = OperationKind::Generic>
    inline static size_t genericReadWriteOperation(uint32_t address, byte* buf, size_t capacity) noexcept {
        PSRAMBlockAddress curr(address);
        setChipId(curr.getIndex());
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        SPDR = opcode;
        asm volatile("nop");
        PSRAMBlockAddress end(address + capacity);
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = curr.bytes_[2];
        asm volatile("nop");
        auto numBytesToSecondChip = end.getOffset();
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = curr.bytes_[1];
        asm volatile("nop");
        auto localToASingleChip = curr.getIndex() == end.getIndex();
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = curr.bytes_[0];
        asm volatile("nop");
        auto numBytesToFirstChip = localToASingleChip ? capacity : (capacity - numBytesToSecondChip);
        while (!(SPSR & _BV(SPIF))) ; // wait
        if constexpr (kind == OperationKind::Write) {
            auto count = numBytesToFirstChip;
            for (decltype(count) i = 0; i < count; ++i) {
                SPDR = buf[i];
                asm volatile("nop");
                while (!(SPSR & _BV(SPIF)));
            }
        } else if constexpr (kind == OperationKind::Read) {
            auto count = numBytesToFirstChip;
            for (decltype(count) i = 0; i < count; ++i) {
                SPDR = 0;
                asm volatile("nop");
                while (!(SPSR & _BV(SPIF)));
                buf[i] = SPDR;
            }
        } else {
            SPI.transfer(buf, numBytesToFirstChip);
        }
        digitalWrite<EnablePin, HIGH>();
        if (!localToASingleChip && (numBytesToSecondChip > 0)) {
            // since size_t is 16-bits on AVR we can safely reduce the largest buffer size 64k, thus we can only ever span two psram chips at a time
            // thus we can actually convert this work into two separate spi transactions
            // start writing at the start of the next chip the remaining number of bytes
            setChipId(end.getIndex());
            // we start at address zero on this new chip always
            digitalWrite<EnablePin, LOW>();
            SPDR = opcode;
            asm volatile("nop");
            while (!(SPSR & _BV(SPIF))) ; // wait
            SPDR = 0;
            asm volatile("nop");
            while (!(SPSR & _BV(SPIF))) ; // wait
            SPDR = 0;
            asm volatile("nop");
            while (!(SPSR & _BV(SPIF))) ; // wait
            SPDR = 0;
            asm volatile("nop");
            while (!(SPSR & _BV(SPIF))) ; // wait
            if constexpr (kind == OperationKind::Write) {
                auto count = numBytesToSecondChip;
                auto actualBuf = buf + numBytesToFirstChip;
                for (decltype(count) i = 0; i < count; ++i) {
                    SPDR = actualBuf[i];
                    asm volatile("nop");
                    while (!(SPSR & _BV(SPIF)));
                }
            } else if (kind == OperationKind::Read) {
                auto count = numBytesToSecondChip;
                auto actualBuf = buf + numBytesToFirstChip;
                for (decltype(count) i = 0; i < count; ++i) {
                    SPDR = 0;
                    asm volatile("nop");
                    while (!(SPSR & _BV(SPIF)));
                    actualBuf[i] = SPDR;
                }
            } else {
                SPI.transfer(buf + numBytesToFirstChip, numBytesToSecondChip);
            }
            digitalWrite<EnablePin, HIGH>();
        }
        SPI.endTransaction();
        return capacity;
    }
public:
    static void writeCacheLine(TaggedAddress address, const byte* buf) noexcept {
        //return genericCacheLineReadWriteOperation<0x02, OperationKind::Write>(address, buf);
        // unlike a generic read/write operation, tagged addresses will never actually span multiple devices so there is no
        // need to do the offset calculation
        setChipId(address.getPSRAMChipId());
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        SPDR = 0x02;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = address.getPSRAMAddress_High();
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = address.getPSRAMAddress_Middle();
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = address.getPSRAMAddress_Low();
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        for (int i = 0; i < 16; ++i) {
            SPDR = buf[i];
            asm volatile("nop");
            while (!(SPSR & _BV(SPIF)));
        }
        digitalWrite<EnablePin, HIGH>();
        SPI.endTransaction();
    }
    static void readCacheLine(TaggedAddress address, byte* buf) noexcept {
        // unlike a generic read/write operation, tagged addresses will never actually span multiple devices so there is no
        // need to do the offset calculation
        setChipId(address.getPSRAMChipId());
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        SPDR = 0x03;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = address.getPSRAMAddress_High();
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = address.getPSRAMAddress_Middle();
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = address.getPSRAMAddress_Low();
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        for (int i = 0; i < 16; ++i) {
            SPDR = 0;
            asm volatile("nop");
            while (!(SPSR & _BV(SPIF)));
            buf[i] = SPDR;
        }
        digitalWrite<EnablePin, HIGH>();
        SPI.endTransaction();
    }
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x02, OperationKind::Write>(address, buf, capacity);
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x03, OperationKind::Read>(address, buf, capacity);
    }
private:
    static void setChipId(byte index) noexcept {
        if (index != currentIndex_) {
            digitalWrite<Select0>((index & (1 << 0)) ? HIGH : LOW);
            digitalWrite<Select1>((index & (1 << 1)) ? HIGH : LOW);
            digitalWrite<Select2>((index & (1 << 2)) ? HIGH : LOW);
            currentIndex_ = index;
        }
    }
public:
    static void begin() noexcept {
        static bool initialized_ = false;
        if (!initialized_) {
            initialized_ = true;
            setChipId(0);
            SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
            for (int i = 0; i < NumChips; ++i) {
                setChipId(i);
                delayMicroseconds(200); // give the psram enough time to come up regardless of where you call begin
                digitalWrite<enablePin, LOW>();
                SPI.transfer(0x66);
                digitalWrite<enablePin, HIGH>();
                asm volatile ("nop");
                asm volatile ("nop");
                digitalWrite<enablePin, LOW>();
                SPI.transfer(0x99);
                digitalWrite<enablePin, HIGH>();
            }
            SPI.endTransaction();
        }
    }
private:
    static inline byte currentIndex_ = 0xFF;
};

using OnboardPSRAMBlock = MemoryBlock<i960Pinout::PSRAM_EN>;
#endif //I960SXCHIPSET_PSRAMCHIP_H
