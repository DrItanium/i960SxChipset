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
/**
 * @brief Represents access to a single PSRAM chip
 */
template<i960Pinout enablePin, bool sanityCheckOnBegin = true, bool clearOnBegin = true>
class PSRAMChip : public MemoryThing {
public:
    static SPISettings& getSettings() noexcept {
        // I am using 74HC series circuits in a 3.3v domain, because of that the maximum swtiching speed is around 150ns or so
        // so for now I must run the psram at 5 mhz or so

        static SPISettings psramSettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0);
        return psramSettings;
    }
    static constexpr uint32_t Size = 8_MB;
    static constexpr uint32_t Mask = Size - 1;
    explicit PSRAMChip(Address start) : MemoryThing(start, start + Size) { }
    ~PSRAMChip() override = default;
    uint8_t read8(Address address) noexcept override {
        SplitWord32 translated(address);
        byte theInstruction[5]{
                0x03,
                translated.bytes[2],
                translated.bytes[1],
                translated.bytes[0],
                0
        };
        doSPI(theInstruction, 5);
        return theInstruction[4];
    }
    uint16_t read16(Address address) noexcept override {
        SplitWord32 translated(address);
        byte theInstruction[6]{
                0x03,
                translated.bytes[2],
                translated.bytes[1],
                translated.bytes[0],
                0, 0
        };
        doSPI(theInstruction, 6);
        return SplitWord16(theInstruction[4], theInstruction[5]).getWholeValue();
    }
    void write8(Address address, uint8_t value) noexcept override {
        SplitWord32 theAddress(address);
        byte theInstruction[5] {
                0x02,
                theAddress.bytes[2],
                theAddress.bytes[1],
                theAddress.bytes[0],
                static_cast<byte>(value),
        };
        doSPI(theInstruction, 5);
    }
    void write16(Address address, uint16_t value) noexcept override {
        SplitWord32 theAddress(address);
        SplitWord16 theValue(value);
        byte theInstruction[6] {
                0x02,
                theAddress.bytes[2],
                theAddress.bytes[1],
                theAddress.bytes[0],
                theValue.bytes[0],
                theValue.bytes[1],
        };
        doSPI(theInstruction, 6);
    }
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        SPI.beginTransaction(getSettings());
        SplitWord32 theAddress(address);
        digitalWrite<enablePin, LOW>();
        SPI.transfer(0x02);
        SPI.transfer(theAddress.bytes[2]);
        SPI.transfer(theAddress.bytes[1]);
        SPI.transfer(theAddress.bytes[0]);
        SPI.transfer(buf, capacity);
        digitalWrite<enablePin, HIGH>();
        SPI.endTransaction();
        return capacity;
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
        SPI.beginTransaction(getSettings());
        SplitWord32 theAddress(address);
        digitalWrite<enablePin, LOW>();
        SPI.transfer(0x03);
        SPI.transfer(theAddress.bytes[2]);
        SPI.transfer(theAddress.bytes[1]);
        SPI.transfer(theAddress.bytes[0]);
        SPI.transfer(buf, capacity);
        digitalWrite<enablePin, HIGH>();
        SPI.endTransaction();
        return capacity;
    }
    void begin() noexcept override {
        delayMicroseconds(200); // give the psram enough time to come up regardless of where you call begin
        SPI.beginTransaction(getSettings());
        digitalWrite<enablePin, LOW>();
        SPI.transfer(0x66);
        digitalWrite<enablePin, HIGH>();
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        digitalWrite<enablePin, LOW>();
        SPI.transfer(0x99);
        digitalWrite<enablePin, HIGH>();
        if constexpr (clearOnBegin && sanityCheckOnBegin) {
            Serial.println(F("TESTING PSRAM!"));
            constexpr auto ActualSize = 64;
            for (uint32_t addr = 0; addr < Size; addr +=ActualSize) {
                SplitWord32 translated(addr);
                byte theInstruction[ActualSize + 4]{
                        0x02,
                        translated.bytes[2],
                        translated.bytes[1],
                        translated.bytes[0],
                        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                };
                doSPI<false>(theInstruction, ActualSize + 4);
                byte theInstruction2[ActualSize + 4]{
                        0x03,
                        translated.bytes[2],
                        translated.bytes[1],
                        translated.bytes[0],
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                };
                // rest of the values do not matter!
                doSPI<false>(theInstruction2, ActualSize + 4);
                byte* ptr = theInstruction2 + 4;
                for (int i = 4; i < (ActualSize + 4); ++i) {
                    if (theInstruction2[i] != 0x55) {
                        Serial.print(F("MISMATCH @ ADDRESS 0x"));
                        Serial.print(translated.wholeValue_, HEX);
                        Serial.print(F(" GOT 0x"));
                        Serial.print(ptr[i], HEX);
                        Serial.println(F(" EXPECTED 0x55"));
                        available_ = false;
                        break;
                    }
                }
                if (!available_) {
                    break;
                }
                // then clear the memory area
                byte theInstruction3[ActualSize + 4]{
                        0x02,
                        translated.bytes[2],
                        translated.bytes[1],
                        translated.bytes[0],
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                };
                doSPI<false>(theInstruction3, ActualSize + 4);
            }
            if (available_) {
                Serial.println(F("DONE STARTING UP PSRAM!"));
            } else {
                Serial.println(F("PSRAM ERROR ON STARTUP, DISABLING!"));
            }
        } else if (clearOnBegin && !sanityCheckOnBegin) {
            Serial.println(F("CLEARING PSRAM"));
            constexpr auto ActualSize = 1024;
            for (uint32_t addr = 0; addr < Size; addr += ActualSize) {
                SplitWord32 translated(addr);
                digitalWrite<enablePin, LOW>();
                SPI.transfer(0x02);
                SPI.transfer(translated.bytes[2]);
                SPI.transfer(translated.bytes[1]);
                SPI.transfer(translated.bytes[0]);
                for (size_t i = 0; i < ActualSize ; ++i) {
                    SPI.transfer(0);
                }
                digitalWrite<enablePin, HIGH>();
            }
            Serial.println(F("DONE CLEARING PSRAM!"));
        }
        SPI.endTransaction();
    }
    [[nodiscard]] bool respondsTo(Address address) const noexcept override {
        if constexpr (sanityCheckOnBegin) {
            return available_ && MemoryThing::respondsTo(address);
        } else {
            return MemoryThing::respondsTo(address);
        }
    }
private:
    template<bool independentTransaction = true>
    void doSPI(byte* command, size_t length) {
        if constexpr (independentTransaction) {
            SPI.beginTransaction(getSettings());
        }
        digitalWrite<enablePin, LOW>();
        SPI.transfer(command, length);
        digitalWrite<enablePin, HIGH>();
        if constexpr (independentTransaction) {
            SPI.endTransaction();
        }
        // make extra sure that the psram has enough time to do its refresh in between operations
    }
public:
    [[nodiscard]] constexpr auto isAvailable() const noexcept { return available_; }
private:
    bool available_ = true;
};
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
    using SingleChip = PSRAMChip<EnablePin, false, false>;
    static constexpr auto NumChips = 8;
    static constexpr auto Size = NumChips * SingleChip::Size;
    static constexpr auto Mask = Size - 1;
    static_assert ((EnablePin != Select0) && (EnablePin != Select1) && (EnablePin != Select2), "The enable pin must be different from all select pins");
    static_assert ((Select0 != Select1) && (Select0 != Select2) && (Select1 != Select2), "All three select pins must point to a different physical pin");
public:
    //explicit MemoryBlock() : currentIndex_(0xFF) { }
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
    template<byte opcode>
    inline static size_t genericReadWriteOperation(uint32_t address, byte* buf, size_t capacity) noexcept {
        PSRAMBlockAddress curr(address);
        PSRAMBlockAddress end(address + capacity);
        //SplitWord32 theAddress(curr.getOffset());
        auto numBytesToSecondChip = end.getOffset();
        auto localToASingleChip = curr.getIndex() == end.getIndex();
        auto numBytesToFirstChip = localToASingleChip ? capacity : (capacity - numBytesToSecondChip);
        setChipId(curr.getIndex());
        SPI.beginTransaction(SingleChip::getSettings());
        digitalWrite<enablePin, LOW>();
        SPI.transfer(opcode);
        SPI.transfer(curr.bytes_[2]);
        SPI.transfer(curr.bytes_[1]);
        SPI.transfer(curr.bytes_[0]);
        SPI.transfer(buf, numBytesToFirstChip);
        digitalWrite<enablePin, HIGH>();
        if ((!localToASingleChip) && (numBytesToSecondChip > 0)) {
            // since size_t is 16-bits on AVR we can safely reduce the largest buffer size 64k, thus we can only ever span two psram chips at a time
            // thus we can actually convert this work into two separate spi transactions
            // start writing at the start of the next chip the remaining number of bytes
            setChipId(end.getIndex());
            // we start at address zero on this new chip always
            digitalWrite<enablePin, LOW>();
            SPI.transfer(opcode);
            SPI.transfer(0);
            SPI.transfer(0);
            SPI.transfer(0);
            SPI.transfer(buf + numBytesToFirstChip, numBytesToSecondChip);
            digitalWrite<enablePin, HIGH>();
        }
        SPI.endTransaction();
        return capacity;
    }
public:
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x02>(address, buf, capacity);
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x03>(address, buf, capacity);
    }
private:
    union Decomposition {
        constexpr explicit Decomposition(byte value = 0) : index(value) { }
        constexpr auto getIndex() const noexcept { return index; }
        byte index;
        struct {
            bool s0 : 1;
            bool s1 : 1;
            bool s2 : 1;
        };
    };
    static void setChipId(byte index) noexcept {
        if (Decomposition dec(index); dec.getIndex() != currentIndex_.getIndex()) {
            digitalWrite<Select0>(dec.s0 ? HIGH : LOW);
            digitalWrite<Select1>(dec.s1 ? HIGH : LOW);
            digitalWrite<Select2>(dec.s2 ? HIGH : LOW);
            currentIndex_ = dec;
        }
    }
public:
    static void begin() noexcept {
        static bool initialized_ = false;
        if (!initialized_) {
            Serial.println(F("BRINGING UP PSRAM MEMORY BLOCK"));
            initialized_ = true;
            currentIndex_.index = 0;
            setChipId(0);
            for (int i = 0; i < 8; ++i) {
                setChipId(i);
                delayMicroseconds(200); // give the psram enough time to come up regardless of where you call begin
                SPI.beginTransaction(SingleChip::getSettings());
                digitalWrite<enablePin, LOW>();
                SPI.transfer(0x66);
                digitalWrite<enablePin, HIGH>();
                asm volatile ("nop");
                asm volatile ("nop");
                asm volatile ("nop");
                asm volatile ("nop");
                digitalWrite<enablePin, LOW>();
                SPI.transfer(0x99);
                digitalWrite<enablePin, HIGH>();
            }
            Serial.println(F("Done bringing up psram memory blocks!"));
        }
    }
private:
    static inline Decomposition currentIndex_ { 0xFF };
};

template<bool clearOnBegin, bool performSanityCheck>
using OnboardPSRAM = PSRAMChip<i960Pinout::PSRAM_EN, performSanityCheck, clearOnBegin>;
using OnboardPSRAMBlock = MemoryBlock<i960Pinout::PSRAM_EN>;
#endif //I960SXCHIPSET_PSRAMCHIP_H
