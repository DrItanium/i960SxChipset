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

/// i960Sx chipset mcu, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include <SPI.h>
#include <libbonuspin.h>
#include <Fsm.h>
#include <SD.h>

#include <Adafruit_TFTShield18.h>
#include <Adafruit_ST7735.h>
#include "Pinout.h"


#include "ProcessorSerializer.h"
#include "MemoryThing.h"
/// Set to false to prevent the console from displaying every single read and write
constexpr bool displayMemoryReadsAndWrites = false;
constexpr bool displayCacheLineUpdates = false;

union MemoryElement {
    explicit MemoryElement(uint16_t value = 0) noexcept : wordValue(value) { }
    uint16_t wordValue;
    uint8_t bytes[sizeof(uint16_t)];
};
/**
 * @brief Describes a single cache line which associates an address with 16 bytes of storage
 */
template<uint32_t size = 16>
struct CacheLine {
public:
    [[nodiscard]] constexpr bool respondsTo(uint32_t targetAddress) const noexcept {
        return valid_ && (address_ <= targetAddress) && (targetAddress < (address_ + CacheLineSize));
    }
    [[nodiscard]] constexpr uint8_t getByte(uint32_t targetAddress) const noexcept {
        auto base = computeCacheByteOffset(targetAddress);
        auto componentId = base >> 1;
        auto offsetId = base & 1;
        return components_[componentId].bytes[offsetId];
    }
    [[nodiscard]] constexpr auto getBaseAddress() const noexcept { return address_; }
    [[nodiscard]] constexpr auto cacheDirty() const noexcept { return dirty_; }
    void setByte(uint32_t address, uint8_t value) noexcept {
        dirty_ = true;
        auto base = computeCacheByteOffset(address);
        auto componentId = computeCacheWordOffset(address);
        auto offsetId = base & 1;
        components_[componentId].bytes[offsetId] = value;
    }
    [[nodiscard]] constexpr uint16_t getWord(uint32_t targetAddress) const noexcept {
        return components_[computeCacheWordOffset(targetAddress)].wordValue;
    }
    void setWord(uint32_t targetAddress, uint16_t value) noexcept {
        dirty_ = true;
        components_[computeCacheWordOffset(targetAddress)].wordValue = value;
    }
    [[nodiscard]] constexpr auto getCacheLineSize() const noexcept { return CacheLineSize; }
    static constexpr auto CacheLineSize = size;
    static constexpr auto ComponentSize = CacheLineSize / sizeof(MemoryElement);
    static constexpr auto CacheByteMask = CacheLineSize - 1;
    static_assert((CacheLineSize == 16 ||
                   CacheLineSize == 32 ||
                   CacheLineSize == 64 ||
                   CacheLineSize == 128 ||
                   CacheLineSize == 256 ||
                   CacheLineSize == 512 ||
                   CacheLineSize == 1024 ||
                   CacheLineSize == 2048 ||
                   CacheLineSize == 4096 ||
                   CacheLineSize == 8192), "CacheLineSize must be 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, or 8192");
    static constexpr uint32_t computeCacheByteOffset(uint32_t targetAddress) noexcept {
        return targetAddress & CacheByteMask;
    }
    static constexpr uint32_t computeCacheWordOffset(uint32_t targetAddress) noexcept {
        return computeCacheByteOffset(targetAddress) >> 1;
    }
    static constexpr uint32_t computeAlignedOffset(uint32_t targetAddress) noexcept {
        return targetAddress & ~CacheByteMask;
    }
    [[nodiscard]] MemoryElement* getMemoryBlock() noexcept { return components_; }
    void reset(uint32_t address, MemoryThing* thing) noexcept {
        byte* buf = reinterpret_cast<byte*>(components_);
        if (valid_ && dirty_) {
            thing->write(address_, buf, CacheLineSize);
        }
        dirty_ = false;
        valid_ = true;
        address_ = address;
        thing->read(address_, buf, CacheLineSize);
    }
    [[nodiscard]] constexpr auto isValid() const noexcept { return valid_; }
private:
    /**
     * @brief The base address of the cache line
     */
    uint32_t address_;
    /**
     * @brief The cache line contents itself
     */
    MemoryElement components_[ComponentSize];
    bool dirty_ = false;
    bool valid_ = false;
    static_assert(sizeof(components_) == CacheLineSize );
};
static_assert(CacheLine<16>::computeAlignedOffset(0xFFFF'FFFF) == 0xFFFF'FFF0);
template<uint32_t numLines = 16, uint32_t cacheLineSize = 32>
class DataCache {
public:
    using ASingleCacheLine = CacheLine<cacheLineSize>;
    static constexpr auto NumberOfCacheLines = numLines;
    static constexpr auto NumberOfCacheLinesMask = numLines - 1;
    static constexpr auto CacheLineSize = cacheLineSize;
    static constexpr auto DataCacheSize = CacheLineSize * NumberOfCacheLines;
    static_assert(DataCacheSize <= 4096, "Overall cache size must be less than or equal to 4k of sram");
    static_assert(NumberOfCacheLines == 16 ||
                  NumberOfCacheLines == 32 ||
                  NumberOfCacheLines == 64 ||
                  NumberOfCacheLines == 128 ||
                  NumberOfCacheLines == 256);
    explicit DataCache(MemoryThing* backingStore) : thing_(backingStore) { }
    [[nodiscard]] uint8_t getByte(uint32_t targetAddress) noexcept {
        if constexpr (displayCacheLineUpdates) {
            Serial.print(F("GetByte: 0x"));
            Serial.println(targetAddress, HEX);
        }
        for (const ASingleCacheLine & line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                auto result = line.getByte(targetAddress);
                if constexpr (displayCacheLineUpdates) {
                    Serial.print(F("\tResult: 0x"));
                    Serial.println(result, HEX);
                }
                return result;

            }
        }
        // cache miss
        // need to replace a cache line
        auto hit = cacheMiss(targetAddress).getByte(targetAddress);
        if constexpr (displayCacheLineUpdates) {
            Serial.print(F("\tHIT: 0x"));
            Serial.println(hit, HEX);
        }
        return hit;

    }
    [[nodiscard]] uint16_t getWord(uint32_t targetAddress) noexcept {
        if constexpr (displayCacheLineUpdates) {
            Serial.print(F("GetWord: 0x"));
            Serial.print(targetAddress, HEX);
        }
        for (const ASingleCacheLine& line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                auto result = line.getWord(targetAddress);
                if constexpr (displayCacheLineUpdates) {
                    Serial.print(F("  Result: 0x"));
                    Serial.println(result, HEX);
                }
                return result;
            }
        }
        // cache miss
        // need to replace a cache line
        auto hit = cacheMiss(targetAddress).getWord(targetAddress);
        if constexpr (displayCacheLineUpdates) {
            Serial.print(F("  HIT: 0x"));
            Serial.println(hit, HEX);
        }
        return hit;
    }
    void setByte(uint32_t targetAddress, uint8_t value) noexcept {
        for (ASingleCacheLine & line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                line.setByte(targetAddress, value);
                return;
            }
        }
        // cache miss
        // need to replace a cache line
        cacheMiss(targetAddress).setByte(targetAddress, value);
    }
    void setWord(uint32_t targetAddress, uint16_t value) noexcept {
        for (ASingleCacheLine& line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                line.setWord(targetAddress, value);
                return;
            }
        }
        // cache miss
        // need to replace a cache line
        cacheMiss(targetAddress).setWord(targetAddress, value);
    }
private:
    /**
     * @brief Looks through the given lines and does a random replacement (like arm cortex R)
     * @param targetAddress
     * @return The line that was updated
     */
    ASingleCacheLine& cacheMiss(uint32_t targetAddress) noexcept {
        auto alignedAddress = ASingleCacheLine::computeAlignedOffset(targetAddress);
        if constexpr (displayCacheLineUpdates) {
            Serial.println(F("Cache Miss: handling cache miss"));
            Serial.print(F("Target Address: 0x"));
            Serial.println(targetAddress, HEX);
            Serial.print(F("Aligned Address: 0x"));
            Serial.println(alignedAddress, HEX);
        }
        // use random number generation to do this
        for (ASingleCacheLine& line : lines_) {
            if (!line.isValid()) {
                line.reset(alignedAddress, thing_);
                return line;
            }
        }
        // we had no free elements so choose one to replace
        auto targetCacheLine = rand() & NumberOfCacheLinesMask;
        ASingleCacheLine& replacementLine = lines_[targetCacheLine];
        if constexpr (displayCacheLineUpdates) {
            Serial.print(F("Number of cache lines: 0x"));
            Serial.println(NumberOfCacheLines, HEX);
            Serial.print(F("Cache lines mask: 0b"));
            Serial.println(NumberOfCacheLinesMask, BIN);
            Serial.print(F("Cache Miss: target cache line: "));
            Serial.println(targetCacheLine, DEC);
            Serial.print(F("Cache Miss: raw address: 0x"));
            Serial.println(targetAddress, HEX);
            Serial.print(F("Cache Miss: aligned address: 0x"));
            Serial.println(alignedAddress, HEX);
        }
        // generate an aligned address
        replacementLine.reset(alignedAddress, thing_);
        return replacementLine;
    }
private:
    MemoryThing* thing_ = nullptr;
    ASingleCacheLine lines_[NumberOfCacheLines];
};
ProcessorInterface& processorInterface = ProcessorInterface::getInterface();
constexpr auto FlashStartingAddress = 0x0000'0000;
constexpr Address OneMemorySpace = 0x0100'0000; // 16 megabytes
// the upper 2G is for non-program related stuff, according to my memory map information, we support a maximum of 512 Megs of RAM
// so the "ram" file is 512 megs in size. If the range is between 0x8000'0000 and
constexpr Address OneMemorySpaceMask = OneMemorySpace - 1;
constexpr Address MaxRamSize = 32 * OneMemorySpace; // 32 Memory Spaces or 512 Megabytes
constexpr auto RamMask = MaxRamSize - 1;
constexpr Address RamStartingAddress = 0x8000'0000;
constexpr auto RamEndingAddress = RamStartingAddress + MaxRamSize;
constexpr Address BaseMCUPeripheralsBaseAddress = 0;
constexpr Address BuiltinLedOffsetBaseAddress = BaseMCUPeripheralsBaseAddress;
constexpr Address BuiltinPortZBaseAddress = BaseMCUPeripheralsBaseAddress + 0x10;
template<typename T>
[[noreturn]] void signalHaltState(T haltMsg);
// ----------------------------------------------------------------
// Load/Store routines
// ----------------------------------------------------------------
class BuiltinLedThing : public IOSpaceThing {
public:
    explicit BuiltinLedThing(Address offsetFromIOBase = 0) : IOSpaceThing(offsetFromIOBase) { }
    ~BuiltinLedThing() override = default;
    [[nodiscard]] uint8_t read8(Address address) noexcept override { return readLed(); }
    [[nodiscard]] uint16_t read16(Address address) noexcept override { return static_cast<uint16_t>(readLed()); }
    void write8(Address /*address*/, uint8_t value) noexcept override { writeLed(value); }
    void write16(Address /*address*/, uint16_t value) noexcept override { writeLed(static_cast<uint8_t>(value)); }
private:
    static void
    writeLed(uint8_t value) noexcept {
        digitalWrite(i960Pinout::Led, value > 0 ? HIGH : LOW);
    }
    static uint8_t
    readLed() noexcept {
        return static_cast<uint8_t>(digitalRead(i960Pinout::Led));
    }
};


class PortZThing : public IOSpaceThing {
public:
    enum class Registers : uint32_t {
        GPIO, // one byte wide
        Direction, // one byte wide
        Polarity,
        Pullup,
    };
public:
    explicit PortZThing(Address base) noexcept : IOSpaceThing(base, base + 16) { }
    ~PortZThing() override = default;
    [[nodiscard]] uint8_t read8(Address offset) noexcept override {
        switch (static_cast<Registers>(offset)) {
            case Registers::GPIO:
                return processorInterface.readPortZGPIORegister();
            case Registers::Pullup:
                return processorInterface.getPortZPullupResistorRegister();
            case Registers::Polarity:
                return processorInterface.getPortZPolarityRegister();
            case Registers::Direction:
                return processorInterface.getPortZDirectionRegister();
            default:
                return 0;
        }
    }
    void write8(Address offset, uint8_t value) noexcept override {
        switch (static_cast<Registers>(offset)) {
            case Registers::GPIO:
                processorInterface.writePortZGPIORegister(value);
                break;
            case Registers::Pullup:
                processorInterface.setPortZPullupResistorRegister(value);
                break;
            case Registers::Polarity:
                processorInterface.setPortZPolarityRegister(value);
                break;
            case Registers::Direction:
                processorInterface.setPortZDirectionRegister(value);
                break;
            default:
                break;
        }
    }
};


class ConsoleThing : public IOSpaceThing {
public:
    enum class Registers : uint32_t {
        Flush,
        Available, // 2 byte
        AvailableForWrite, // 2 byte
        IO, // 2 bytes
        BufferDoorbell, // 2 bytes
        BufferSize, // 2 bytes
        BufferLength, // 2 bytes
        BufStart, // 2 byte
    };
public:
    // make sure we allocate a ton of space just in case
    explicit ConsoleThing(Address base) noexcept : IOSpaceThing(base, base + 0x100) { }
    ~ConsoleThing() override = default;
    static constexpr auto BufferSize = 128;
    [[nodiscard]] uint8_t read8(Address offset) noexcept override {
        constexpr auto baseAddress = static_cast<uint32_t>(Registers::BufStart) * sizeof(uint16_t);
        constexpr auto endAddress = baseAddress + BufferSize;
        if (offset >= baseAddress && endAddress > offset) {
            return buf_[offset - baseAddress];
        } else {
            return 0;
        }
    }
    [[nodiscard]] uint16_t read16(Address offset) noexcept override {
        switch (offset) {
            case static_cast<uint32_t>(Registers::Available) * sizeof(uint16_t): return Serial.available();
            case static_cast<uint32_t>(Registers::AvailableForWrite) * sizeof(uint16_t): return Serial.availableForWrite();
            case static_cast<uint32_t>(Registers::IO) * sizeof(uint16_t): return Serial.read();
            case static_cast<uint32_t>(Registers::BufferDoorbell) * sizeof(uint16_t):
                return static_cast<uint16_t>(Serial.readBytes(buf_, bufCount_));
            case static_cast<uint32_t>(Registers::BufferLength) * sizeof(uint16_t):
                return static_cast<uint16_t>(bufCount_);
            case static_cast<uint32_t>(Registers::BufferSize) * sizeof(uint16_t):
                return static_cast<uint16_t>(BufferSize);
            default:
                return 0;
        }
    }
    void write8(Address offset, uint8_t value) noexcept override {
        constexpr auto baseAddress = static_cast<uint32_t>(Registers::BufStart) * sizeof(uint16_t);
        constexpr auto endAddress = baseAddress + BufferSize;
        if (offset >= baseAddress && endAddress > offset) {
            buf_[offset - baseAddress] = value;
        }
    }
    void write16(Address offset, uint16_t value) noexcept override {
        switch (offset) {
            case static_cast<uint32_t>(Registers::IO) * sizeof(uint16_t):
                Serial.write(static_cast<char>(value));
            case static_cast<uint32_t>(Registers::Flush) * sizeof(uint16_t):
                Serial.flush();
                break;
            case static_cast<uint32_t>(Registers::BufferDoorbell) * sizeof(uint16_t):
                Serial.write(buf_, bufCount_);
                break;
            case static_cast<uint32_t>(Registers::BufferLength) * sizeof(uint16_t):
                bufCount_ = value > BufferSize ? BufferSize : value;
                break;
            default:
                break;
        }
    }
private:
    char buf_[BufferSize] = { 0 };
    byte bufCount_ = 0;

};

class RAMThing : public MemoryThing {
public:
    RAMThing() noexcept : MemoryThing(RamStartingAddress, RamEndingAddress) { }
    ~RAMThing() override {
        // while this will never get called, it is still a good idea to be complete
        theRAM_.close();
    }
    [[nodiscard]] uint8_t
    read8(Address offset) noexcept override {
        theRAM_.seek(offset); // jump to that point in memory
        return theRAM_.read();
    }
    void
    write8(Address offset, uint8_t value) noexcept override {
        theRAM_.seek(offset);
        theRAM_.write(static_cast<uint8_t>(value));
        theRAM_.flush();
    }
    [[nodiscard]] uint16_t
    read16(Address offset) noexcept override {
        uint16_t storage;
        theRAM_.seek(offset);
        theRAM_.read(&storage, 2);
        return storage;
    }
    void
    write16(Address offset, uint16_t value) noexcept override {
        auto thePtr = reinterpret_cast<uint8_t*>(&value);
        theRAM_.seek(offset);
        theRAM_.write(thePtr, 2);
        theRAM_.flush();
    }
    [[nodiscard]] Address
    makeAddressRelative(Address input) const noexcept override {
        // in this case, we want relative offsets
        return input & RamMask;
    }

    void
    begin() noexcept override {
        if (!SD.exists("ram.bin")) {
            signalHaltState(F("NO RAM.BIN FOUND!"));
        } else {
            theRAM_ = SD.open("ram.bin", FILE_WRITE);
            Serial.println(F("RAM.BIN OPEN SUCCESS!"));
        }
    }
private:
    File theRAM_; // use an SDCard as ram for the time being
};

template<uint32_t numCacheLines, uint32_t cacheLineSize = 32>
class ROMThing : public MemoryThing {
public:
    static constexpr Address ROMStart = 0;
    static constexpr Address ROMEnd = 0x8000'0000;
    static constexpr Address ROMMask = ROMEnd - 1;
public:
    ROMThing() noexcept : MemoryThing(ROMStart, ROMEnd), theCache_(this) { }
    ~ROMThing() override {
        // while this will never get called, it is still a good idea to be complete
        theBootROM_.close();
    }

    [[nodiscard]] uint8_t
    read8(Address offset) noexcept override {
        return theCache_.getByte(offset);
    }
    [[nodiscard]] uint16_t
    read16(Address offset) noexcept override {
        // use the onboard cache to get data from
        auto result = theCache_.getWord(offset);
        return result;
    }
    [[nodiscard]] Address
    makeAddressRelative(Address input) const noexcept override {
        return input & ROMMask;
    }
    [[nodiscard]] bool
    respondsTo(Address address) const noexcept override {
        return address < size_;
    }
    void read(uint32_t baseAddress, byte *buffer, size_t size) override {
        if constexpr (displayCacheLineUpdates) {
            Serial.print(F("Accessing "));
            Serial.print(size, DEC);
            Serial.print(F(" bytes starting at 0x"));
            Serial.println(baseAddress, HEX);
        }
        theBootROM_.seek(baseAddress);
        theBootROM_.read(buffer, size);
        if constexpr (displayCacheLineUpdates) {
            Serial.println(F("READ ROMTHING!"));
            for (size_t i = 0; i < size; ++i) {
                Serial.print(F("0x"));
                Serial.print(baseAddress + i, HEX);
                Serial.print(F(": 0x"));
                Serial.print(buffer[i], HEX);
                Serial.print(F(" (0x"));
                Serial.print(reinterpret_cast<uint32_t>(&buffer[i]), HEX);
                Serial.println(F(")"));
            }
        }
    }
    void
    begin() noexcept override {
        if (!SD.exists("boot.rom")) {
            signalHaltState(F("NO BOOT.ROM!"));
        }
        theBootROM_ = SD.open("boot.rom", FILE_READ);
        Serial.println(F("BOOT.ROM OPEN SUCCESS!"));
        size_ = theBootROM_.size();
        if (size_ == 0) {
            signalHaltState(F("EMPTY BOOT.ROM"));
        } else if (size_ > 0x8000'0000) {
            signalHaltState(F("BOOT.ROM TOO LARGE")) ;
        }
        // okay now setup the initial cache block
        (void)theCache_.getByte(0);
    }
    [[nodiscard]] constexpr auto getROMSize() const noexcept { return size_; }

private:
// boot rom and sd card loading stuff
    File theBootROM_;
    uint32_t size_ = 0;
    DataCache<numCacheLines, cacheLineSize> theCache_;
};


class CPUInternalMemorySpace : public MemoryThing {
public:
    CPUInternalMemorySpace() noexcept : MemoryThing(0xFF00'0000) { }
    ~CPUInternalMemorySpace() override  = default;
    [[nodiscard]] bool
    respondsTo(Address address) const noexcept override {
        return address > 0xFF00'0000;
    }
};
class TFTShieldThing : public IOSpaceThing {
public:
    enum class Registers : uint32_t {
        Flush = 0,
        IO,
        Available,
        AvailableForWrite,
        Command,
        X,
        Y,
        W,
        H,
        Radius,
        Color,
        BGColor,
        X0,
        Y0,
        X1,
        Y1,
        X2,
        Y2,
        R,
        G,
        B,
        Doorbell,
        Backlight,
        BacklightFrequency,
        ButtonsLower,
        ButtonsUpper,
        ButtonsQuery,

    };
    enum class Opcodes : uint16_t {
        None = 0,
        SetRotation,
        InvertDisplay,
        FillRect,
        FillScreen,
        DrawLine,
        DrawRect,
        DrawCircle,
        FillCircle,
        DrawTriangle,
        FillTriangle,
        SetTextSizeSquare,
        SetTextSizeRectangle,
        SetCursor,
        SetTextColor0,
        SetTextColor1,
        SetTextWrap,
        GetWidth,
        GetHeight,
        GetRotation,
        GetCursorX,
        GetCursorY,
        DrawPixel,
        Color565,
        DrawRoundRect,
        FillRoundRect,
    };
public:
    explicit TFTShieldThing(Address base) : IOSpaceThing(base, base + 0x100),
                                            display_(static_cast<int>(i960Pinout::DISPLAY_EN),
                                                     static_cast<int>(i960Pinout::DC),
                                                     -1) { }
    ~TFTShieldThing() override = default;
    /**
     * @brief Invoke on doorbell write
     * @param value the value written to the doorbell
     * @return the value to return to the i960 if it makes sense (otherwise it will be zero)
     */
    uint16_t invoke(uint16_t /* unused */) {
        // perhaps we'll do nothing with the value but hold onto it for now
        switch (command_) {
            case Opcodes::SetRotation:
                display_.setRotation(x_);
                break;
            case Opcodes::InvertDisplay:
                display_.invertDisplay(x_ != 0);
                break;
            case Opcodes::DrawPixel:
                display_.drawPixel(x_, y_, color_);
                break;
            case Opcodes::FillRect:
                display_.fillRect(x_, y_, w_, h_, color_);
                break;
            case Opcodes::FillScreen:
                display_.fillScreen(color_);
                break;
            case Opcodes::Color565:
                return display_.color565(r_, g_, b_);
            case Opcodes::DrawLine:
                display_.drawLine(x0_, y0_, x1_, y1_, color_);
                break;
            case Opcodes::DrawRect:
                display_.drawRect(x_, y_, w_, h_, color_);
                break;
            case Opcodes::DrawCircle:
                display_.drawCircle(x0_, y0_, r_, color_);
                break;
            case Opcodes::FillCircle:
                display_.fillCircle(x0_, y0_, r_, color_);
                break;
            case Opcodes::DrawTriangle:
                display_.drawTriangle(x0_, y0_, x1_, y1_, x2_, y2_, color_);
                break;
            case Opcodes::FillTriangle:
                display_.fillCircle(x0_, y0_, r_, color_);
                break;
            case Opcodes::DrawRoundRect:
                display_.drawRoundRect(x_, y_, w_, h_, r_, color_);
                break;
            case Opcodes::FillRoundRect:
                display_.fillRoundRect(x_, y_, w_, h_, r_, color_);
                break;
            case Opcodes::SetTextSizeSquare:
                display_.setTextSize(x_);
                break;
            case Opcodes::SetTextSizeRectangle:
                display_.setTextSize(x_, y_);
                break;
            case Opcodes::SetCursor:
                display_.setCursor(x_, y_);
                break;
            case Opcodes::SetTextColor0:
                display_.setTextColor(color_);
                break;
            case Opcodes::SetTextColor1:
                display_.setTextColor(color_, bgcolor_);
                break;
            case Opcodes::SetTextWrap:
                display_.setTextWrap(x_ != 0);
                break;
            case Opcodes::GetWidth:
                return display_.width();
            case Opcodes::GetHeight:
                return display_.height();
            case Opcodes::GetRotation:
                return display_.getRotation();
            case Opcodes::GetCursorX:
                return display_.getCursorX();
            case Opcodes::GetCursorY:
                return display_.getCursorY();
            default:
                return 0;
        }
        return 0;
    }
public:
    [[nodiscard]] constexpr auto getCommand() const noexcept { return command_; }
    [[nodiscard]] constexpr auto getX() const noexcept { return x_; }
    [[nodiscard]] constexpr auto getY() const noexcept { return y_; }
    [[nodiscard]] constexpr auto getW() const noexcept { return w_; }
    [[nodiscard]] constexpr auto getH() const noexcept { return h_; }
    [[nodiscard]] constexpr auto getRadius() const noexcept { return radius_; }
    [[nodiscard]] constexpr uint16_t getColor() const noexcept { return color_; }
    [[nodiscard]] constexpr uint16_t getBackgroundColor() const noexcept { return bgcolor_; }
    [[nodiscard]] constexpr auto getX0() const noexcept { return x0_; }
    [[nodiscard]] constexpr auto getY0() const noexcept { return y0_; }
    [[nodiscard]] constexpr auto getX1() const noexcept { return x1_; }
    [[nodiscard]] constexpr auto getY1() const noexcept { return y1_; }
    [[nodiscard]] constexpr auto getX2() const noexcept { return x2_; }
    [[nodiscard]] constexpr auto getY2() const noexcept { return y2_; }
    [[nodiscard]] constexpr auto getRed() const noexcept { return r_; }
    [[nodiscard]] constexpr auto getGreen() const noexcept { return g_; }
    [[nodiscard]] constexpr auto getBlue() const noexcept { return b_; }
    void setCommand(Opcodes command) noexcept { command_ = command; }
    void setX(int16_t x) noexcept { x_ = x; }
    void setY(int16_t y) noexcept { y_ = y; }
    void setW(int16_t w) noexcept { w_ = w; }
    void setH(int16_t h) noexcept { h_ = h; }
    void setRadius(int16_t radius) noexcept { radius_ = radius; }
    void setColor(uint16_t color) noexcept { color_ = color; }
    void setBackgroundColor(uint16_t color) noexcept { bgcolor_ = color; }
    void setX0(int16_t value) noexcept { x0_ = value; }
    void setY0(int16_t value) noexcept { y0_ = value; }
    void setX1(int16_t value) noexcept { x1_ = value; }
    void setY1(int16_t value) noexcept { y1_ = value; }
    void setX2(int16_t value) noexcept { x2_ = value; }
    void setY2(int16_t value) noexcept { y2_ = value; }
    void setR(int16_t value) noexcept { r_ = value; }
    void setG(int16_t value) noexcept { g_ = value; }
    void setB(int16_t value) noexcept { b_ = value; }
    void flush() { display_.flush(); }
    void print(char c) { display_.print(c); }
    [[nodiscard]] bool available() noexcept { return true; }
    [[nodiscard]] bool availableForWriting() noexcept { return display_.availableForWrite(); }
    uint16_t read16(Address address) noexcept override {
        switch (address) {
#define X(title) case (static_cast<Address>(Registers:: title) * sizeof(uint16_t))
                X(Available) : return available();
                X(AvailableForWrite) : return availableForWriting();
                X(Command) : return static_cast<uint16_t>(getCommand());
                X(X) : return getX();
                X(Y) : return getY();
                X(W) : return getW();
                X(H) : return getH();
                X(Radius) : return getRadius();
                X(Color) : return getColor();
                X(BGColor) : return getBackgroundColor();
                X(X0) : return getX0();
                X(Y0) : return getY0();
                X(X1) : return getX1();
                X(Y1) : return getY1();
                X(X2) : return getX2();
                X(Y2) : return getY2();
                X(R) : return getRed();
                X(G) : return getGreen();
                X(B) : return getBlue();
                X(Doorbell) : return invoke(0);
                X(Backlight) : return backlightStatus_;
                X(BacklightFrequency) : return backlightFrequency_;
                X(ButtonsLower) : return buttonsCache_ & 0xFFFF;
                X(ButtonsUpper) : return (buttonsCache_ >> 16) & 0xFFFF;
#undef X
            default: return 0;
        }
    }
    void write16(Address address, uint16_t value) noexcept override {
        switch (address) {
#define X(title) case (static_cast<Address>(Registers:: title) * sizeof(uint16_t))
            X(Command) :
                setCommand(static_cast<Opcodes>(value));
                break;
            X(X) : setX(static_cast<int16_t>(value)); break;
            X(Y) : setY(static_cast<int16_t>(value)); break;
            X(W) : setW(static_cast<int16_t>(value)); break;
            X(H) : setH(static_cast<int16_t>(value)); break;
            X(Radius) : setRadius(static_cast<int16_t>(value)); break;
            X(Color) : setColor(value); break;
            X(BGColor) : setBackgroundColor(value); break;
            X(X0) : setX0(static_cast<int16_t>(value)); break;
            X(Y0) : setY0(static_cast<int16_t>(value)); break;
            X(X1) : setX1(static_cast<int16_t>(value)); break;
            X(Y1) : setY1(static_cast<int16_t>(value)); break;
            X(X2) : setX2(static_cast<int16_t>(value)); break;
            X(Y2) : setY2(static_cast<int16_t>(value)); break;
            X(R) : setR(static_cast<int16_t>(value)); break;
            X(G) : setG(static_cast<int16_t>(value)); break;
            X(B) : setB(static_cast<int16_t>(value)); break;
            X(Doorbell) :
                resultLower_ = invoke(value);
                break;
            X(Backlight) :
                backlightStatus_ = value != 0 ? TFTSHIELD_BACKLIGHT_ON : TFTSHIELD_BACKLIGHT_OFF;
                ss.setBacklight(backlightStatus_);
                break;
            X(BacklightFrequency) :
                backlightFrequency_ = value;
                ss.setBacklightFreq(backlightFrequency_);
                break;
            X(ButtonsQuery):
                buttonsCache_ = ss.readButtons();
                break;
#undef X
            default: break;
        }
    }
    inline void fillScreen(uint16_t value) noexcept { display_.fillScreen(value); }
    inline void setCursor(int16_t x, int16_t y) noexcept { display_.setCursor(x, y); }
    inline void setTextColor(uint16_t value) noexcept { display_.setTextColor(value); }
    inline void setTextSize(uint16_t size) noexcept { display_.setTextSize(size); }
    template<typename T, typename ... Args>
    inline void println(T msg, Args&& ... args) noexcept {
        display_.println(msg, args...);
    }
    void
    begin() noexcept override {
        Serial.println(F("Setting up the seesaw"));
        if (!ss.begin()) {
            signalHaltState(F("NO SEESAW"));
        }
        Serial.println(F("seesaw started"));
        Serial.print(F("Version: "));
        Serial.println(ss.getVersion(), HEX);
        ss.setBacklight(TFTSHIELD_BACKLIGHT_OFF);
        ss.tftReset();
        display_.initR(INITR_BLACKTAB); // initialize a ST7735S, black tab
        ss.setBacklight(TFTSHIELD_BACKLIGHT_ON);
        Serial.println(F("TFT OK!"));
        display_.fillScreen(ST77XX_CYAN);
        Serial.println(F("Screen should have cyan in it!"));
        delay(100);
        display_.fillScreen(ST77XX_BLACK);
        display_.setCursor(0, 0);
        display_.setTextColor(ST77XX_WHITE);
        display_.setTextSize(3);
        display_.println(F("i960Sx!"));
    }
private:
    Adafruit_ST7735 display_;
    Opcodes command_;
    int16_t x_ = 0;
    int16_t y_ = 0;
    int16_t w_ = 0;
    int16_t h_ = 0;
    int16_t radius_ = 0;
    uint16_t color_ = 0;
    uint16_t bgcolor_ = 0;
    int16_t x0_ = 0;
    int16_t y0_ = 0;
    int16_t x1_ = 0;
    int16_t y1_ = 0;
    int16_t x2_ = 0;
    int16_t y2_ = 0;
    int16_t r_ = 0;
    int16_t g_ = 0;
    int16_t b_ = 0;
    uint16_t resultLower_ = 0;
    uint16_t resultUpper_ = 0;
    uint16_t backlightStatus_ = TFTSHIELD_BACKLIGHT_ON;
    uint16_t backlightFrequency_ = 0;
    uint32_t buttonsCache_ = 0;
    Adafruit_TFTShield18 ss;

};
BuiltinLedThing theLed(BuiltinLedOffsetBaseAddress);
PortZThing portZThing(BuiltinPortZBaseAddress);
ConsoleThing theConsole(0x100);
TFTShieldThing displayCommandSet(0x200);
RAMThing ram;
ROMThing<16,16> rom;
CPUInternalMemorySpace internalMemorySpaceSink;
// list of memory devices to walk through
MemoryThing* things[] {
        &rom,
        &ram,
        &theLed,
        &portZThing,
        &theConsole,
        &displayCommandSet,
        &internalMemorySpaceSink,
};

void
performWrite(Address address, uint16_t value, LoadStoreStyle style) noexcept {
    for (auto* currentThing : things) {
        if (!currentThing) {
            continue;
        }
        if (currentThing->respondsTo(address, style)) {
            currentThing->write(address, value, style);
            break;
        }
    }
}

uint16_t
performRead(Address address, LoadStoreStyle style) noexcept {
    for (auto* currentThing : things) {
        if (!currentThing) {
            continue;
        }
        if (currentThing->respondsTo(address, style)) {
           auto result = currentThing->read(address, style);
           return result;
        }
    }
    return 0;
}

// ----------------------------------------------------------------
// state machine
// ----------------------------------------------------------------
// The bootup process has a separate set of states
// TStart - Where we start
// TSystemTest - Processor performs self test
//
// TStart -> TSystemTest via FAIL being asserted
// TSystemTest -> Ti via FAIL being deasserted
// 
// State machine will stay here for the duration
// State diagram based off of i960SA/SB Reference manual
// Basic Bus States
// Ti - Idle State
// Ta - Address State
// Td - Data State
// Tr - Recovery State
// Tw - Wait State
// TChecksumFailure - Checksum Failure State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in 

// Ti -> Ti via no request
// Tr -> Ti via no request
// Tr -> Ta via request pending
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Tr after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted
// Tr -> TChecksumFailure if FAIL is asserted

// NOTE: Tw may turn out to be synthetic
volatile uint32_t baseAddress = 0;
volatile bool performingRead = false;
constexpr auto NoRequest = 0;
constexpr auto NewRequest = 1;
constexpr auto ReadyAndBurst = 2;
constexpr auto NotReady = 3;
constexpr auto ReadyAndNoBurst = 4;
constexpr auto RequestPending = 5;
constexpr auto ToDataState = 6;
constexpr auto PerformSelfTest = 7;
constexpr auto SelfTestComplete = 8;
constexpr auto ChecksumFailure = 9;
void startupState() noexcept;
void systemTestState() noexcept;
void idleState() noexcept;
void doAddressState() noexcept;
void processDataRequest() noexcept;
void doRecoveryState() noexcept;
void enteringDataState() noexcept;
void enteringChecksumFailure() noexcept;
State tStart(nullptr, startupState, nullptr);
State tSystemTest(nullptr, systemTestState, nullptr);
Fsm fsm(&tStart);
State tIdle(nullptr,
            idleState,
            nullptr);
State tAddr([]() { processorInterface.clearASTrigger(); },
            doAddressState,
            nullptr);
State tData(enteringDataState,
            processDataRequest,
            nullptr);
State tRecovery(nullptr,
                doRecoveryState,
                nullptr);
State tChecksumFailure(enteringChecksumFailure, nullptr, nullptr);


void startupState() noexcept {
    if (processorInterface.failTriggered()) {
        fsm.trigger(PerformSelfTest);
    }
}
void systemTestState() noexcept {
    if (!processorInterface.failTriggered()) {
        fsm.trigger(SelfTestComplete);
    }
}
void onASAsserted() {
    processorInterface.triggerAS();
}
void onDENAsserted() {
    processorInterface.triggerDEN();
}

void idleState() noexcept {
    if (processorInterface.failTriggered()) {
        fsm.trigger(ChecksumFailure);
    } else {
        if (processorInterface.asTriggered()) {
            fsm.trigger(NewRequest);
        }
    }
}
void doAddressState() noexcept {
    if (processorInterface.denTriggered()) {
        fsm.trigger(ToDataState);
    }
}



void
enteringDataState() noexcept {
    // when we do the transition, record the information we need
    processorInterface.clearDENTrigger();
    baseAddress = processorInterface.getAddress();
    performingRead = processorInterface.isReadOperation();
}
void processDataRequest() noexcept {
    auto burstAddress = processorInterface.getBurstAddress(baseAddress);
    if (performingRead) {
        processorInterface.setDataBits(performRead(burstAddress, processorInterface.getStyle()));
    } else {
        performWrite(burstAddress, processorInterface.getDataBits(), processorInterface.getStyle());
    }
    // setup the proper address and emit this over serial
    auto blastAsserted = processorInterface.blastTriggered();
    processorInterface.signalReady();
    if (blastAsserted) {
        // we not in burst mode
        fsm.trigger(ReadyAndNoBurst);
    }

}

void doRecoveryState() noexcept {
    if (processorInterface.failTriggered()) {
        fsm.trigger(ChecksumFailure);
    } else {
        if (processorInterface.asTriggered()) {
            fsm.trigger(RequestPending);
        } else {
            fsm.trigger(NoRequest);
        }
    }
}


// ----------------------------------------------------------------
// setup routines
// ----------------------------------------------------------------

void setupBusStateMachine() noexcept {
    fsm.add_transition(&tStart, &tSystemTest, PerformSelfTest, nullptr);
    fsm.add_transition(&tSystemTest, &tIdle, SelfTestComplete, nullptr);
    fsm.add_transition(&tIdle, &tAddr, NewRequest, nullptr);
    fsm.add_transition(&tIdle, &tChecksumFailure, ChecksumFailure, nullptr);
    fsm.add_transition(&tAddr, &tData, ToDataState, nullptr);
    fsm.add_transition(&tData, &tRecovery, ReadyAndNoBurst, nullptr);
    fsm.add_transition(&tRecovery, &tAddr, RequestPending, nullptr);
    fsm.add_transition(&tRecovery, &tIdle, NoRequest, nullptr);
    fsm.add_transition(&tRecovery, &tChecksumFailure, ChecksumFailure, nullptr);
    fsm.add_transition(&tData, &tChecksumFailure, ChecksumFailure, nullptr);
}

void setupPeripherals() {
    Serial.println(F("Setting up peripherals..."));
    displayCommandSet.begin();
    internalMemorySpaceSink.begin();
    if (!SD.begin(static_cast<int>(i960Pinout::SD_EN))) {
        signalHaltState(F("SD CARD INIT FAILed"));
    }
    rom.begin();
    ram.begin();
    // setup the bus things
    Serial.println(F("Done setting up peripherals..."));
}

// the setup routine runs once when you press reset:
void setup() {
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    setupPins(OUTPUT,
              i960Pinout::SPI_BUS_EN,
              i960Pinout::DISPLAY_EN,
              i960Pinout::SD_EN,
              i960Pinout::Reset960,
              i960Pinout::Ready,
              i960Pinout::GPIOSelect,
              i960Pinout::Led,
              i960Pinout::Int0_);
    PinAsserter<i960Pinout::Reset960> holdi960InReset;
    // all of these pins need to be pulled high
    digitalWriteBlock(HIGH,
                      i960Pinout::SPI_BUS_EN,
                      i960Pinout::SD_EN,
                      i960Pinout::DISPLAY_EN,
                      i960Pinout::Ready,
                      i960Pinout::GPIOSelect,
                      i960Pinout::Int0_);
    setupPins(INPUT,
              i960Pinout::BLAST_,
              i960Pinout::AS_,
              i960Pinout::W_R_,
              i960Pinout::DEN_,
              i960Pinout::FAIL);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::AS_)), onASAsserted, FALLING);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::DEN_)), onDENAsserted, FALLING);
    digitalWrite(i960Pinout::Led, LOW);
    Serial.begin(115200);
    while(!Serial);
    theLed.begin();
    theConsole.begin();
    Serial.println(F("i960Sx chipset bringup"));
    SPI.begin();
    processorInterface.begin();
    // setup the CPU Interface
    setupBusStateMachine();
    setupPeripherals();
    delay(1000);
    // we want to jump into the code as soon as possible after this point
    Serial.println(F("i960Sx chipset brought up fully!"));
}
void loop() {
    fsm.run_machine();
}

void enteringChecksumFailure() noexcept {
    auto msg = F("CHECKSUM FAILURE");
    displayCommandSet.fillScreen(ST77XX_RED);
    displayCommandSet.setCursor(0, 0);
    displayCommandSet.setTextSize(2);
    displayCommandSet.println(msg);
    Serial.println(msg);
}
template<typename T>
[[noreturn]] void signalHaltState(T haltMsg) {
    enteringChecksumFailure();
    while(true) {
        delay(1000);
    }
}
/// @todo Eliminate after MightyCore update
#if __cplusplus >= 201402L

void operator delete(void * ptr, size_t)
{
    ::operator delete(ptr);
}

void operator delete[](void * ptr, size_t)
{
    ::operator delete(ptr);
}

#endif // end language is C++14 or greater