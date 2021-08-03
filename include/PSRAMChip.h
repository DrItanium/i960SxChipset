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
template<i960Pinout enablePin>
class PSRAMChip : public MemoryThing {
public:
    static SPISettings& getSettings() noexcept {
        static SPISettings psramSettings(8_MHz, MSBFIRST, SPI_MODE0);
        return psramSettings;
    }
    static constexpr uint32_t Size = 8_MB;
    static constexpr uint32_t Mask = Size - 1;
    explicit PSRAMChip(Address start) : MemoryThing(start, start + Size) { }
    ~PSRAMChip() override = default;
    uint8_t read8(Address address) noexcept override {
        return readOneByte(address);
    }
    union BlockCapacityInfo {
        constexpr explicit BlockCapacityInfo(size_t value = 0) : value_(value) { }
        size_t value_;
        struct {
            // avr specific at this point
            size_t slop : 5;
            size_t times : 11;
        };
    };
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
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
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
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
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
        SPI.endTransaction();
        Serial.println(F("TESTING PSRAM!"));
        for (uint32_t addr = 0; addr < Size; addr +=32) {
            SplitWord32 translated(addr);
            byte theInstruction[36]{
                    0x02,
                    translated.bytes[2],
                    translated.bytes[1],
                    translated.bytes[0],
                    1, 2, 3, 4, 5, 6, 7, 8,
                    9, 10, 11, 12, 13, 14, 15, 16,
                    17, 18, 19, 20, 21, 22, 23, 24,
                    25, 26, 27, 28, 29, 30, 31, 32,
            };
            byte theInstruction2[36]{
                    0x03,
                    translated.bytes[2],
                    translated.bytes[1],
                    translated.bytes[0],
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
            };
            doSPI(theInstruction, 36);
            // rest of the values do not matter!
            doSPI(theInstruction2, 36);
            byte* ptr = theInstruction2 + 4;
            for (int i = 0; i < 32; ++i) {
                if (ptr[i] != (i+1)) {
                    Serial.print(F("MISMATCH @ ADDRESS 0x"));
                    Serial.print(translated.wholeValue_, HEX);
                    Serial.print(F(" GOT 0x"));
                    Serial.print(ptr[i], HEX);
                    Serial.print(F(" EXPECTED 0x"));
                    Serial.println((i + 1), HEX);
                    available_ = false;
                    break;
                }
            }
            if (!available_) {
                break;
            }
        }
        if (available_) {
            Serial.println(F("CLEARING PSRAM!"));
            for (uint32_t addr = 0; addr < Size; addr +=32) {
                SplitWord32 translated(addr);
                byte theInstruction[36]{
                        0x02,
                        translated.bytes[2],
                        translated.bytes[1],
                        translated.bytes[0],
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                };
                byte theInstruction2[36]{
                        0x03,
                        translated.bytes[2],
                        translated.bytes[1],
                        translated.bytes[0],
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                };
                doSPI(theInstruction, 36);
                // rest of the values do not matter!
                doSPI(theInstruction2, 36);
            }
        }
        SPI.endTransaction();
        if (available_) {
            Serial.println(F("DONE STARTING UP PSRAM!"));
        } else {
            Serial.println(F("PSRAM ERROR ON STARTUP, DISABLING!"));
        }
    }
    [[nodiscard]] bool respondsTo(Address address) const noexcept override {
        return available_ && MemoryThing::respondsTo(address);
    }
private:
    void doSPI(byte* command, size_t length) {
        SPI.beginTransaction(getSettings());
        digitalWrite<enablePin, LOW>();
        //asm volatile ("nop"); // 100 ns
        SPI.transfer(command, length);
        //asm volatile ("nop"); // 100 ns
        digitalWrite<enablePin, HIGH>();
        SPI.endTransaction();
        // make extra sure that the psram has enough time to do its refresh in between operations
        asm volatile ("nop"); // 100 ns
        asm volatile ("nop"); // 100 ns
        asm volatile ("nop"); // 100 ns
        asm volatile ("nop"); // 100 ns
    }
    uint16_t readTwoBytes(Address addr) noexcept {
        SplitWord32 translated(addr);
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
    uint8_t readOneByte(Address addr) noexcept {
        SplitWord32 translated(addr);
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
    void writeTwoBytes(Address addr, uint16_t value) noexcept {
        SplitWord32 theAddress(addr);
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
    void writeOneByte(Address addr, uint8_t value) noexcept {
        SplitWord32 theAddress(addr);
        byte theInstruction[5] {
                0x02,
                theAddress.bytes[2],
                theAddress.bytes[1],
                theAddress.bytes[0],
                static_cast<byte>(value),
        };
        doSPI(theInstruction, 5);
    }
private:
    bool available_ = true;
};

using OnboardPSRAM = PSRAMChip<i960Pinout::PSRAM_EN>;
#endif //I960SXCHIPSET_PSRAMCHIP_H
