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
template<i960Pinout enablePin, bool sanityCheckOnBegin = true>
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
        if constexpr (sanityCheckOnBegin) {
            Serial.println(F("TESTING PSRAM!"));
        } else {
            Serial.println(F("CLEARING PSRAM ONLY!"));
        }
        constexpr auto ActualSize = 64;
        for (uint32_t addr = 0; addr < Size; addr +=ActualSize) {
            SplitWord32 translated(addr);
            if constexpr (sanityCheckOnBegin) {
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
        SPI.endTransaction();
        if constexpr (sanityCheckOnBegin) {
            if (available_) {
                Serial.println(F("DONE STARTING UP PSRAM!"));
            } else {
                Serial.println(F("PSRAM ERROR ON STARTUP, DISABLING!"));
            }
        } else {
            Serial.println(F("DONE CLEARING PSRAM!"));
        }
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
 * @brief Wraps eight different SPI sram chips into a single object
 * @tparam enablePin The pin that is used to signal chip usage
 * @tparam sel0 The lowest pin used to select the target device
 * @tparam sel1 The middle pin used to select the target device
 * @tparam sel2 The upper pin used to select the target device
 */
template<i960Pinout enablePin,
        bool performSanityCheck = true,
        i960Pinout sel0 = i960Pinout::SPI_OFFSET0,
        i960Pinout sel1 = i960Pinout::SPI_OFFSET1,
        i960Pinout sel2 = i960Pinout::SPI_OFFSET2>
class PSRAMBlock8 : public MemoryThing {
public:
    static constexpr auto EnablePin = enablePin;
    static constexpr auto Select0 = sel0;
    static constexpr auto Select1 = sel1;
    static constexpr auto Select2 = sel2;
    using SingleChip = PSRAMChip<EnablePin, performSanityCheck>;
    static constexpr auto NumChips = 8;
    static constexpr auto Size = NumChips * SingleChip::Size;
    static constexpr auto Mask = Size - 1;
    static constexpr auto SingleChipSize = SingleChip::Size;
    static_assert ((EnablePin != Select0) && (EnablePin != Select1) && (EnablePin != Select2), "The enable pin must be different from all select pins");
    static_assert ((Select0 != Select1) && (Select0 != Select2) && (Select1 != Select2), "All three select pins must point to a different physical pin");
    static_assert(Size == 64_MB, "PSRAMBlock8 needs to be 1 megabyte in size");
    static_assert(Mask == 0x03FF'FFFF, "PSRAMBlock8 mask is wrong!");
public:
    explicit PSRAMBlock8(Address base) : MemoryThing(base, base + Size),
    backingChips{
                SingleChip (base + (0 * SingleChipSize)),
                SingleChip (base + (1 * SingleChipSize)),
                SingleChip (base + (2 * SingleChipSize)),
                SingleChip (base + (3 * SingleChipSize)),
                SingleChip (base + (4 * SingleChipSize)),
                SingleChip (base + (5 * SingleChipSize)),
                SingleChip (base + (6 * SingleChipSize)),
                SingleChip (base + (7 * SingleChipSize)),
    } {

    }
    ~PSRAMBlock8() override = default;
    union Address26 {
        constexpr explicit Address26(Address value = 0) : base(value) { }
        constexpr auto getAddress() const noexcept { return base; }
        constexpr auto getOffset() const noexcept { return offset; }
        constexpr auto getIndex() const noexcept { return index; }
        Address base : 26; // 1 megabyte always
        struct {
            Address offset : 23;
            Address index : 3;
        };
    };

    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        // okay figure out which device to start the write into, since size_t is 16 bits on AVR we can exploit this for our purposes.
        // There is no way to go beyond a 64k buffer (which to be honest would be crazy anyway since this chip doesn't have that much sram)
        // because of that we can just do a write to the underlying device and find out how much is left over to continue from that point
        Address26 curr(address);
        Address26 end(address + capacity);
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
        Address26 curr(address);
        Address26 end(address + capacity);
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
        Address26 curr(address);
        setChipId(curr);
        // it will always be lower8 because the read method which called this method already added 1. Thus we can just write a byte with the address we currently have
        return static_cast<uint8_t>(backingChips[curr.getIndex()].read(address, LoadStoreStyle::Lower8));
    }
    uint16_t read16(Address address) noexcept override {
        Address26 curr(address);
        setChipId(curr);
        return backingChips[curr.getIndex()].read(address, LoadStoreStyle::Full16);
    }
    void write8(Address address, uint8_t value) noexcept override {
        Address26 curr(address);
        setChipId(curr);
        backingChips[curr.getIndex()].write(curr.getOffset(), value, LoadStoreStyle::Lower8);
    }
    void write16(Address address, uint16_t value) noexcept override {
        Address26 curr(address);
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
    inline void setChipId(const Address26& address) noexcept {
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
            Serial.println(F("BRINGING UP PSRAM MEMORY BLOCK"));
            initialized_ = true;
            currentIndex_.index = 0;
            setChipId(0);
            for (int i = 0; i < 8; ++i) {
                setChipId(i);
                backingChips[i].begin();
                if (!backingChips[i].isAvailable()) {
                    available_ = false;
                    break;
                }
            }
            if (available_) {
                Serial.println(F("Done bringing up psram memory blocks!"));
            } else {
                Serial.println(F("DISABLING ONBOARD PSRAM ACCESS"));
            }
        }
    }
    bool respondsTo(Address address) const noexcept override {
        return available_ && MemoryThing::respondsTo(address);
    }
private:
    bool available_ = true;
    bool initialized_ = false;
    Decomposition currentIndex_;
    SingleChip backingChips[NumChips];
};

template<bool performSanityCheck>
using OnboardPSRAM = PSRAMChip<i960Pinout::PSRAM_EN, performSanityCheck>;
template<bool performSanityCheck>
using OnboardPSRAMBlock = PSRAMBlock8<i960Pinout::PSRAM_EN, performSanityCheck>;
#endif //I960SXCHIPSET_PSRAMCHIP_H
