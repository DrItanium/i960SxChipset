//
// Created by jwscoggins on 7/31/21.
//

#ifndef I960SXCHIPSET_PSRAMCHIP_H
#define I960SXCHIPSET_PSRAMCHIP_H
#include <Arduino.h>
#include <SPI.h>
#include "Pinout.h"
#include "MemoryThing.h"
/**
 * @brief Represents access to a single PSRAM chip
 */
template<i960Pinout enablePin>
class PSRAMChip : public MemoryThing {
public:
    static SPISettings psramSettings(8_MHz, MSBFIRST, SPI_MODE0);
    static constexpr uint32_t Size = 8_MB;
    static constexpr uint32_t Mask = Size - 1;
    explicit PSRAMChip(Address start) : MemoryThing(start, start + Size) { }
    ~PSRAMChip() override = default;
    uint8_t read8(Address address) noexcept override {
            Serial.print(F("PSRAM: READ8 FROM 0x"));
            Serial.println(address, HEX);
            return readOneByte(address);
    }
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
            // do not copy the buf but just use it as a transfer medium instead
            SPI.beginTransaction(psramSettings);
            auto times = capacity / 32;
            auto slop = capacity % 32;
            auto* theBuf = buf;
            for (size_t i = 0; i < times; ++i, theBuf += 32) {
                byte header[4]{
                        0x02,
                        static_cast<byte>(address >> 16),
                        static_cast<byte>(address >> 8),
                        static_cast<byte>(address),
                };
                digitalWrite<enablePin, LOW>();
                SPI.transfer(header, 4);
                SPI.transfer(theBuf, 32);
                digitalWrite<enablePin, HIGH>();
            }
            // there should be some slop left over
            if (slop > 0) {
                byte header[36]{
                        0x02,
                        static_cast<byte>(address >> 16),
                        static_cast<byte>(address >> 8),
                        static_cast<byte>(address),
                };

                digitalWrite<enablePin, LOW>();
                SPI.transfer(header, 4);
                SPI.transfer(theBuf, slop);
                digitalWrite<enablePin, HIGH>();
            }
            SPI.endTransaction();
            return capacity;
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
            SPI.beginTransaction(psramSettings);
            auto times = capacity / 32;
            auto slop = capacity % 32;
            auto* theBuf = buf;
            for (size_t i = 0; i < times; ++i, theBuf += 32) {
                byte header[4]{
                        0x03,
                        static_cast<byte>(address >> 16),
                        static_cast<byte>(address >> 8),
                        static_cast<byte>(address),
                };
                digitalWrite<enablePin, LOW>();
                SPI.transfer(header, 4);
                SPI.transfer(theBuf, 32);
                digitalWrite<enablePin, HIGH>();
            }
            // there should be some slop left over
            if (slop > 0) {
                byte header[4]{
                        0x03,
                        static_cast<byte>(address >> 16),
                        static_cast<byte>(address >> 8),
                        static_cast<byte>(address),
                };
                digitalWrite<enablePin, LOW>();
                SPI.transfer(header, 4);
                SPI.transfer(theBuf, 32);
                digitalWrite<enablePin, HIGH>();
            }
            SPI.endTransaction();
            return capacity;
    }
    uint16_t read16(Address address) noexcept override {
            Serial.print(F("PSRAM: READ16 FROM 0x"));
            Serial.println(address, HEX);
            return readTwoBytes(address);
    }
    void write8(Address address, uint8_t value) noexcept override {
            Serial.print(F("PSRAM: WRITE8 0x"));
            Serial.print(value, HEX);
            Serial.print(F(" to 0x"));
            Serial.println(address, HEX);
            writeOneByte(address, value);
    }
    void write16(Address address, uint16_t value) noexcept override {
            Serial.print(F("PSRAM: WRITE16 0x"));
            Serial.print(value, HEX);
            Serial.print(F(" to 0x"));
            Serial.println(address, HEX);
            writeTwoBytes(address, value);
    }
    void begin() noexcept override {
            delayMicroseconds(200); // give the psram enough time to come up regardless of where you call begin
            SPI.beginTransaction(psramSettings);
            digitalWrite<enablePin, LOW>();
            SPI.transfer(0x66);
            digitalWrite<enablePin, HIGH>();
            asm("nop");
            digitalWrite<enablePin, LOW>();
            SPI.transfer(0x99);
            digitalWrite<enablePin, HIGH>();
            SPI.endTransaction();
            Serial.println(F("CLEARING PSRAM!"));
            for (uint32_t addr = 0; addr < Size; addr +=32) {
                byte theInstruction[36]{
                        0x02,
                        static_cast<byte>(addr >> 16),
                        static_cast<byte>(addr >> 8),
                        static_cast<byte>(addr),
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0,
                };
                doSPI(theInstruction, 36);
                theInstruction[0]  = 0x03;
                theInstruction[1] =  static_cast<byte>(addr >> 16);
                theInstruction[2] =  static_cast<byte>(addr >> 8);
                theInstruction[3] = static_cast<byte>(addr);
                // rest of the values do not matter!
                doSPI(theInstruction, 36);
                byte* ptr = theInstruction + 4;
                for (int i = 0; i < 32; ++i) {
                    if (ptr[i] != 0) {
                        Serial.print(F("MISMATCH GOT 0x"));
                        Serial.print(ptr[i], HEX);
                        Serial.println(F(" EXPECTED 0x0"));
                        available_ = false;
                        break;
                    }
                }
                if (!available_) {
                    break;
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
        SPI.beginTransaction(psramSettings);
        digitalWrite<enablePin, LOW>();
        SPI.transfer(command, length);
        digitalWrite<enablePin, HIGH>();
        SPI.endTransaction();
        // make extra sure that the psram has enough time to do its refresh in between operations
        asm("nop"); // 100 ns
        asm("nop"); // 100 ns
        asm("nop"); // 100 ns
        asm("nop"); // 100 ns
    }
    uint16_t readTwoBytes(Address addr) noexcept {
        byte theInstruction[6]{
                0x03,
                static_cast<byte>(addr >> 16),
                static_cast<byte>(addr >> 8),
                static_cast<byte>(addr),
                0, 0
        };
        doSPI(theInstruction, 6);
        auto lower = static_cast<uint16_t>(theInstruction[4]);
        auto upper = static_cast<uint16_t>(theInstruction[5]) << 8;
        return lower | upper;
    }
    uint8_t readOneByte(Address addr) noexcept {
        byte theInstruction[5]{
                0x03,
                static_cast<byte>(addr >> 16),
                static_cast<byte>(addr >> 8),
                static_cast<byte>(addr),
                0
        };
        doSPI(theInstruction, 5);
        return theInstruction[4];
    }
    void writeTwoBytes(Address addr, uint16_t value) noexcept {
        byte theInstruction[6] {
                0x02,
                static_cast<byte>(addr >> 16),
                static_cast<byte>(addr >> 8),
                static_cast<byte>(addr),
                static_cast<byte>(value),
                static_cast<byte>(value >> 8)
        };
        doSPI(theInstruction, 6);
    }
    void writeOneByte(Address addr, uint8_t value) noexcept {
        byte theInstruction[5] {
                0x02,
                static_cast<byte>(addr >> 16),
                static_cast<byte>(addr >> 8),
                static_cast<byte>(addr),
                static_cast<byte>(value),
        };
        doSPI(theInstruction, 5);
    }
private:
    bool available_ = true;
};

using OnboardPSRAM = PSRAMChip<i960Pinout::PSRAM_EN>;
#endif //I960SXCHIPSET_PSRAMCHIP_H
