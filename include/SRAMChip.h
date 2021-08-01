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
class SRAMChip : public MemoryThing {
public:
    static constexpr uint32_t Size = 128_KB;
    static constexpr uint32_t Mask = Size - 1;
    explicit SRAMChip(Address start) : MemoryThing(start, start + Size) { }
    ~SRAMChip() override = default;
    uint8_t read8(Address address) noexcept override {
            return readOneByte(address);
    }
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
            // do not copy the buf but just use it as a transfer medium instead
            auto times = capacity / 32;
            auto slop = capacity % 32;
            auto* theBuf = buf;
            SplitWord32 trackedAddress(address);
            for (size_t i = 0; i < times; ++i, theBuf += 32, trackedAddress.wholeValue_ += 32) {
                byte header[4]{
                        0x02,
                        trackedAddress.bytes[2],
                        trackedAddress.bytes[1],
                        trackedAddress.bytes[0],
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
                        trackedAddress.bytes[2],
                        trackedAddress.bytes[1],
                        trackedAddress.bytes[0],
                };

                digitalWrite<enablePin, LOW>();
                SPI.transfer(header, 4);
                SPI.transfer(theBuf, slop);
                digitalWrite<enablePin, HIGH>();
            }
            return capacity;
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
            auto times = capacity / 32;
            auto slop = capacity % 32;
            auto* theBuf = buf;
            SplitWord32 trackedAddress(address);
            for (size_t i = 0; i < times; ++i, theBuf += 32, trackedAddress.wholeValue_ += 32) {
                byte header[4]{
                        0x03,
                        trackedAddress.bytes[2],
                        trackedAddress.bytes[1],
                        trackedAddress.bytes[0],
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
                        trackedAddress.bytes[2],
                        trackedAddress.bytes[1],
                        trackedAddress.bytes[0],
                };
                digitalWrite<enablePin, LOW>();
                SPI.transfer(header, 4);
                SPI.transfer(theBuf, slop);
                digitalWrite<enablePin, HIGH>();
            }
            return capacity;
    }
    uint16_t read16(Address address) noexcept override {
            Serial.print(F("SRAM: READ16 FROM 0x"));
            Serial.println(address, HEX);
            return readTwoBytes(address);
    }
    void write8(Address address, uint8_t value) noexcept override {
            Serial.print(F("SRAM: WRITE8 0x"));
            Serial.print(value, HEX);
            Serial.print(F(" to 0x"));
            Serial.println(address, HEX);
            writeOneByte(address, value);
    }
    void write16(Address address, uint16_t value) noexcept override {
            Serial.print(F("SRAM: WRITE16 0x"));
            Serial.print(value, HEX);
            Serial.print(F(" to 0x"));
            Serial.println(address, HEX);
            writeTwoBytes(address, value);
    }
    void begin() noexcept override {
            digitalWrite(i960Pinout::CACHE_EN_, LOW);
            SPI.transfer(0xFF);
            digitalWrite(i960Pinout::CACHE_EN_, HIGH);
            constexpr uint32_t max = 128_KB; // only one SRAM chip on board
            Serial.println(F("CHECKING SRAM IS PROPERLY WRITABLE"));
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
                    }
                }
            }
            Serial.println(F("SUCCESSFULLY CHECKED SRAM CACHE!"));
            Serial.println(F("PURGING SRAM CACHE!"));
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
                    }
                }
            }
            Serial.println(F("DONE PURGING SRAM CACHE!"));
    }
private:
    void doSPI(byte* command, size_t length) {
        digitalWrite<enablePin, LOW>();
        SPI.transfer(command, length);
        digitalWrite<enablePin, HIGH>();
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
};

using OnBoardSRAM = SRAMChip<i960Pinout::CACHE_EN_>;
#endif //I960SXCHIPSET_PSRAMCHIP_H
