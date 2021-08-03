//
// Created by jwscoggins on 7/31/21.
//

#ifndef I960SXCHIPSET_SRAMCHIP_H
#define I960SXCHIPSET_SRAMCHIP_H
#include <Arduino.h>
#include <SPI.h>
#include "Pinout.h"
#include "MemoryThing.h"
#include "../.pio/libdeps/1284p/SdFat/src/common/FmtNumber.h"
#include "../.pio/libdeps/1284p/SdFat/src/FsLib/FsNew.h"

/**
 * @brief Represents access to a single PSRAM chip
 */
template<i960Pinout enablePin>
class SRAMChip : public MemoryThing {
public:
    static constexpr uint32_t Size = 128_KB;
    static constexpr uint32_t Mask = Size - 1;
    explicit SRAMChip(Address start) : MemoryThing(start, start + Size) { }
    ~SRAMChip() override = default;

    static SPISettings& getSettings() noexcept {
        static SPISettings psramSettings(8_MHz, MSBFIRST, SPI_MODE0);
        return psramSettings;
    }
    bool respondsTo(Address address) const noexcept override {
        return MemoryThing::respondsTo(address);
    }
    uint8_t read8(Address address) noexcept override {
        return readOneByte(address);
    }
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        // do not copy the buf but just use it as a transfer medium instead
        SplitWord32 trackedAddress(address);
        byte header[4]{
                0x02,
                trackedAddress.bytes[2],
                trackedAddress.bytes[1],
                trackedAddress.bytes[0],
        };
        SPI.beginTransaction(getSettings());
        digitalWrite<enablePin, LOW>();
        SPI.transfer(header, 4);
        SPI.transfer(buf, capacity);
        digitalWrite<enablePin, HIGH>();
        SPI.endTransaction();
        return capacity;
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
        SplitWord32 trackedAddress(address);
        byte header[4]{
                0x03,
                trackedAddress.bytes[2],
                trackedAddress.bytes[1],
                trackedAddress.bytes[0],
        };
        SPI.beginTransaction(getSettings());
        digitalWrite<enablePin, LOW>();
        SPI.transfer(header, 4);
        SPI.transfer(buf, capacity);
        digitalWrite<enablePin, HIGH>();
        SPI.endTransaction();
        return capacity;
    }
    uint16_t read16(Address address) noexcept override {
            return readTwoBytes(address);
    }
    void write8(Address address, uint8_t value) noexcept override {
            writeOneByte(address, value);
    }
    void write16(Address address, uint16_t value) noexcept override {
            writeTwoBytes(address, value);
    }
    void begin() noexcept override {
        SPI.beginTransaction(getSettings());
            digitalWrite(i960Pinout::CACHE_EN_, LOW);
            SPI.transfer(0xFF);
            digitalWrite(i960Pinout::CACHE_EN_, HIGH);
            constexpr uint32_t max = 128_KB; // only one SRAM chip on board
            Serial.println(F("CHECKING SRAM..."));
            for (uint32_t i = 0; i < max; i += 32) {
                SplitWord32 translation(i);
                byte pagePurgeInstruction[36]{
                        0x02,
                        translation.bytes[2],
                        translation.bytes[1],
                        translation.bytes[0],
                        1, 2, 3, 4, 5, 6, 7, 8,
                        9, 10, 11, 12, 13, 14, 15, 16,
                        17, 18, 19, 20, 21, 22, 23, 24,
                        25, 26, 27, 28, 29, 30, 31, 32,
                };
                byte pageReadInstruction[36]{
                        0x03,
                        translation.bytes[2],
                        translation.bytes[1],
                        translation.bytes[0],
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                };
                digitalWrite<i960Pinout::CACHE_EN_, LOW>();
                SPI.transfer(pagePurgeInstruction, 36);
                digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
                digitalWrite<i960Pinout::CACHE_EN_, LOW>();
                SPI.transfer(pageReadInstruction, 36);
                digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
                for (int x = 4; x < 36; ++x) {
                    auto a = pageReadInstruction[x];
                    auto index = x - 3;
                    if (a != index) {
                        Serial.print(F("MISMATCH 0x"));
                        Serial.print(index, HEX);
                        Serial.print(F(" => 0x"));
                        Serial.println(a, HEX);
                        signalHaltState(F("SRAM CHECK FAILURE! HALTING!!"));
                    }
                }
            }
            Serial.println(F("SUCCESSFULLY CHECKED SRAM CACHE!"));
            Serial.println(F("PURGING SRAM!"));
            for (uint32_t i = 0; i < max; i+= 32) {
                SplitWord32 translation(i);
                byte pagePurgeInstruction[36] {
                        0x02,
                        translation.bytes[2],
                        translation.bytes[1],
                        translation.bytes[0],
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                };
                byte pageReadInstruction[36]{
                        0x03,
                        translation.bytes[2],
                        translation.bytes[1],
                        translation.bytes[0],
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                };
                digitalWrite<i960Pinout::CACHE_EN_, LOW>();
                SPI.transfer(pagePurgeInstruction, 36);
                digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
                digitalWrite<i960Pinout::CACHE_EN_, LOW>();
                SPI.transfer(pageReadInstruction, 36);
                digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
                for (int x = 4; x < 36; ++x) {
                    if (pageReadInstruction[x] != 0) {
                        Serial.print(F("CHECK FAILURE!!!"));
                        signalHaltState(F("SRAM PURGE FAILURE! HALTING!!"));
                    }
                }
            }
            Serial.println(F("DONE PURGING SRAM CACHE!"));
            SPI.endTransaction();
    }
private:
    void doSPI(byte* command, size_t length) {
        SPI.beginTransaction(getSettings());
        digitalWrite<enablePin, LOW>();
        SPI.transfer(command, length);
        digitalWrite<enablePin, HIGH>();
        SPI.endTransaction();
        // make extra sure that the psram has enough time to do its refresh in between operations
    }
    uint16_t readTwoBytes(Address addr) noexcept {
        SplitWord32 addrTmp(addr);
        byte theInstruction[6]{
                0x03,
                addrTmp.bytes[2],
                addrTmp.bytes[1],
                addrTmp.bytes[0],
                0, 0
        };
        doSPI(theInstruction, 6);
        return SplitWord16(theInstruction[4], theInstruction[5]).wholeValue_;
    }
    uint8_t readOneByte(Address addr) noexcept {
        SplitWord32 addrTmp(addr);
        byte theInstruction[5]{
                0x03,
                addrTmp.bytes[2],
                addrTmp.bytes[1],
                addrTmp.bytes[0],
                0
        };
        doSPI(theInstruction, 5);
        return theInstruction[4];
    }
    void writeTwoBytes(Address addr, uint16_t value) noexcept {
        SplitWord32 addrTmp(addr);
        SplitWord16 tmp(value);
        byte theInstruction[6] {
                0x02,
                addrTmp.bytes[2],
                addrTmp.bytes[1],
                addrTmp.bytes[0],
                tmp.bytes[0],
                tmp.bytes[1],
        };
        doSPI(theInstruction, 6);
    }
    void writeOneByte(Address addr, uint8_t value) noexcept {
        SplitWord32 addrTmp(addr);
        byte theInstruction[5] {
                0x02,
                addrTmp.bytes[2],
                addrTmp.bytes[1],
                addrTmp.bytes[0],
                static_cast<byte>(value),
        };
        doSPI(theInstruction, 5);
    }
private:
    bool available_ = false;
};
/**
 * @brief Wraps eight different SPI sram chips into a single object
 * @tparam enablePin The pin that is used to signal chip usage
 * @tparam sel0 The lowest pin used to select the target device
 * @tparam sel1 The middle pin used to select the target device
 * @tparam sel2 The upper pin used to select the target device
 */
template<i960Pinout enablePin,
        i960Pinout sel0 = i960Pinout::SPI_OFFSET0,
        i960Pinout sel1 = i960Pinout::SPI_OFFSET1,
        i960Pinout sel2 = i960Pinout::SPI_OFFSET2>
class SRAMBlock8 : public MemoryThing {
public:
    static constexpr auto EnablePin = enablePin;
    static constexpr auto Select0 = sel0;
    static constexpr auto Select1 = sel1;
    static constexpr auto Select2 = sel2;
    using SingleSRAMChip = SRAMChip<EnablePin>;
    static constexpr auto NumSRAMChips = 8;
    static constexpr auto Size = NumSRAMChips * SingleSRAMChip::Size;
    static constexpr auto Mask = Size - 1;
    static constexpr auto SingleChipSize = SingleSRAMChip::Size;
    static_assert ((EnablePin != Select0) && (EnablePin != Select1) && (EnablePin != Select2), "The enable pin must be different from all select pins");
    static_assert ((Select0 != Select1) && (Select0 != Select2) && (Select1 != Select2), "All three select pins must point to a different physical pin");
    static_assert(Size == 1_MB, "SRAMBlock8 needs to be 1 megabyte in size");
    static_assert(Mask == 0x0F'FFFF, "SRAMBlock8 mask is wrong!");
public:
    explicit SRAMBlock8(Address base) : MemoryThing(base, base + Size),
    backingChips{
        SingleSRAMChip (base + (0 * SingleChipSize)),
        SingleSRAMChip (base + (1 * SingleChipSize)),
        SingleSRAMChip (base + (2 * SingleChipSize)),
        SingleSRAMChip (base + (3 * SingleChipSize)),
        SingleSRAMChip (base + (4 * SingleChipSize)),
        SingleSRAMChip (base + (5 * SingleChipSize)),
        SingleSRAMChip (base + (6 * SingleChipSize)),
        SingleSRAMChip (base + (7 * SingleChipSize)),
    } {

    }
    ~SRAMBlock8() override = default;
    union Address20 {
        constexpr explicit Address20(Address value = 0) : base(value) { }
        constexpr auto getAddress() const noexcept { return base; }
        constexpr auto getOffset() const noexcept { return offset; }
        constexpr auto getIndex() const noexcept { return index; }
        Address base : 20; // 1 megabyte always
        struct {
            Address offset : 17;
            Address index : 3;
        };
    };

    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        // okay figure out which device to start the write into, since size_t is 16 bits on AVR we can exploit this for our purposes.
        // There is no way to go beyond a 64k buffer (which to be honest would be crazy anyway since this chip doesn't have that much sram)
        // because of that we can just do a write to the underlying device and find out how much is left over to continue from that point
        Address20 curr(address);
        Address20 end(address + capacity);
        size_t bytesRemaining = capacity;
        size_t bytesWritten = 0;
        auto currentAddress = address;
        auto* theBuf = buf;
        // just keep spanning the whole area as we go along
        for (byte i = curr.getIndex(); i <= end.getIndex(); ++i) {
            setChipId(i);
            auto numWritten = backingChips[i].write(currentAddress, theBuf, bytesRemaining);
            bytesWritten += numWritten;
            bytesRemaining -= numWritten;
            theBuf += numWritten;
            currentAddress += numWritten;
        }
        return bytesWritten;
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
        // just like blockWrite, we can span multiple devices and thus we just need to keep populating the buffer as we go along
        Address20 curr(address);
        Address20 end(address + capacity);
        size_t bytesRemaining = capacity;
        size_t bytesRead = 0;
        auto* theBuf = buf;
        auto currentAddress = address;
        // just keep spanning the whole area as we go along
        for (byte i = curr.getIndex(); i <= end.getIndex(); ++i) {
            setChipId(i);
            auto numRead = backingChips[i].read(currentAddress, theBuf, bytesRemaining);
            bytesRead += numRead;
            bytesRemaining -= numRead;
            theBuf += numRead;
            currentAddress += numRead;
        }
        return bytesRead;
    }
    uint8_t read8(Address address) noexcept override {
        Address20 curr(address);
        setChipId(curr);
        // it will always be lower8 because the read method which called this method already added 1. Thus we can just write a byte with the address we currently have
        return static_cast<uint8_t>(backingChips[curr.getIndex()].read(address, LoadStoreStyle::Lower8));
    }
    uint16_t read16(Address address) noexcept override {
        Address20 curr(address);
        setChipId(curr);
        return backingChips[curr.getIndex()].read(address, LoadStoreStyle::Full16);
    }
    void write8(Address address, uint8_t value) noexcept override {
        Address20 curr(address);
        setChipId(curr);
        backingChips[curr.getIndex()].write(curr.getOffset(), value, LoadStoreStyle::Lower8);
    }
    void write16(Address address, uint16_t value) noexcept override {
        Address20 curr(address);
        setChipId(curr);
        backingChips[curr.getIndex()].write(curr.getOffset(), value, LoadStoreStyle::Full16);
        MemoryThing::write16(address, value);
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
    inline void setChipId(const Address20& address) noexcept {
        setChipId(address.getIndex());
    }
    void setChipId(byte index) noexcept {
        if (Decomposition dec(index); dec.getIndex() != currentIndex_.getIndex()) {
            digitalWrite<Select0>(dec.s0 ? HIGH : LOW);
            digitalWrite<Select1>(dec.s1 ? HIGH : LOW);
            digitalWrite<Select2>(dec.s2 ? HIGH : LOW);
            currentIndex_ = dec;
        }
    }
public:
    void begin() noexcept override  {
        if (!initialized_) {
            Serial.println(F("BRINGING UP SRAM MEMORY BLOCK"));
            initialized_ = true;
            currentIndex_.index = 0;
            setChipId(0);
            for (int i = 0; i < 8; ++i) {
                setChipId(i);
                backingChips[i].begin();
            }
            Serial.println(F("Done bringing up sram memory blocks!"));
        }
    }
private:
    bool initialized_ = false;
    Decomposition currentIndex_;
    SingleSRAMChip backingChips[8];
};
using OnboardSRAMBlock = SRAMBlock8<i960Pinout::PSRAM_EN,
                                    i960Pinout::SPI_OFFSET0,
                                    i960Pinout::SPI_OFFSET1,
                                    i960Pinout::SPI_OFFSET2>;
#endif //I960SXCHIPSET_SRAMCHIP_H
