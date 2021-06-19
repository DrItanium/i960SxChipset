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
#include <Fsm.h>
#include <SD.h>

#include <Adafruit_TFTShield18.h>
#include <Adafruit_ST7735.h>
#include "Pinout.h"

#include "ProcessorSerializer.h"
#include "MemoryThing.h"
#include "MemoryMappedFileThing.h"
#include "SDCardFileSystemInterface.h"
#include "OPL2Thing.h"
#ifdef ADAFRUIT_FEATHER
#include "FeatherWingPeripherals.h"
#endif

#if defined(ARDUINO_GRAND_CENTRAL_M4)
#include <Adafruit_ZeroTimer.h>
#endif
//OPL2Thing<i960Pinout::SPI_BUS_A0, i960Pinout::SPI_BUS_A1, i960Pinout::SPI_BUS_A2> thingy(0x1000);

bool displayReady = false;
MemoryThing* getThing(Address address, LoadStoreStyle style) noexcept;
/**
 * @brief Describes a single cache line which associates an address with 16 bytes of storage
 */
ProcessorInterface& processorInterface = ProcessorInterface::getInterface();
// ----------------------------------------------------------------
// Load/Store routines
// ----------------------------------------------------------------
class CoreChipsetFeatures : public IOSpaceThing {
public:
    static constexpr auto AllowDebuggingStatements = true;
    enum class Registers : uint32_t {
        Led, // one byte
        DisplayMemoryReadsAndWrites,
        DisplayCacheLineUpdates,
        PortZGPIO = 0x10, // one byte wide
        PortZGPIODirection, // one byte wide
        PortZGPIOPolarity,
        PortZGPIOPullup,
        ConsoleFlush = 0x20, // 2 bytes for alignment purposes
        ConsoleFlushUpper, // 2 bytes for alignment purposes
#define TwoByteEntry(Prefix) Prefix, Prefix ## Upper
#define FourByteEntry(Prefix) Prefix ## LowerHalf, Prefix ## UpperLowerHalf, Prefix ## UpperHalf, Prefix ## UpperUpperHalf
        TwoByteEntry(ConsoleAvailable),
        TwoByteEntry(ConsoleAvailableForWrite),
        TwoByteEntry(ConsoleIO),
        /// @todo implement console buffer
        FourByteEntry(ConsoleBufferAddress),
        ConsoleBufferLength, // up to 256 bytes in length
        ConsoleBufferDoorbell, // read from this to do a buffered read, write to this to do a write from memory to console
#undef FourByteEntry
#undef TwoByteEntry
    };
    explicit CoreChipsetFeatures(Address offsetFromIOBase = 0) : IOSpaceThing(offsetFromIOBase, offsetFromIOBase + 0x100) {
        consoleBufferBaseAddress_.full = 0;

    }
    ~CoreChipsetFeatures() override = default;
private:
    uint8_t buffer_[256] = { 0 };
    uint8_t
    readFromStream(Stream& aStream, Address baseAddress, uint8_t length) noexcept {
        uint8_t count = 0;
        if (auto thing = getThing(baseAddress, LoadStoreStyle::Lower8); thing) {
            // force the i960sx to wait
            count = aStream.readBytes(buffer_, length);
            thing->write(thing->makeAddressRelative(baseAddress), buffer_, length);
        }
        return count;
    }
    uint8_t
    writeToStream(Stream& aStream, Address baseAddress, uint8_t length) noexcept {
        uint8_t count = 0;
        if (auto thing = getThing(baseAddress, LoadStoreStyle::Lower8); thing) {
            thing->read(thing->makeAddressRelative(baseAddress), buffer_, length);
            count = aStream.write(buffer_, length);
        }
        return count;
    }
public:
    [[nodiscard]] uint8_t read8(Address address) noexcept override {
        switch (static_cast<Registers>(address)) {
            case Registers::Led:
                return readLed();
            case Registers::DisplayMemoryReadsAndWrites:
                return displayMemoryReadsAndWrites();
            case Registers::DisplayCacheLineUpdates:
                return displayCacheLineUpdates();
            case Registers::PortZGPIO:
                return processorInterface.readPortZGPIORegister();
            case Registers::PortZGPIODirection:
                return processorInterface.getPortZDirectionRegister();
            case Registers::PortZGPIOPolarity:
                return processorInterface.getPortZPolarityRegister();
            case Registers::PortZGPIOPullup:
                return processorInterface.getPortZPullupResistorRegister();
            case Registers::ConsoleBufferLength:
                return consoleBufferLength_;
            case Registers::ConsoleBufferDoorbell:
                return readFromStream(Serial, consoleBufferBaseAddress_.full, consoleBufferLength_);
            default:
                return 0;
        }
    }
    [[nodiscard]] uint16_t read16(Address address) noexcept override {
        switch (static_cast<Registers>(address)) {
            case Registers::ConsoleIO: return Serial.read();
            case Registers::ConsoleAvailable: return Serial.available();
            case Registers::ConsoleAvailableForWrite: return Serial.availableForWrite();
            case Registers::ConsoleBufferAddressLowerHalf: return consoleBufferBaseAddress_.halves[0];
            case Registers::ConsoleBufferAddressUpperHalf: return consoleBufferBaseAddress_.halves[1];
            default: return 0;
        }
    }
    void write16(Address address, uint16_t value) noexcept override {
        switch (static_cast<Registers>(address)) {
            case Registers::ConsoleFlush:
                Serial.flush();
                break;
            case Registers::ConsoleIO:
                Serial.write(static_cast<char>(value));
                break;
            case Registers::ConsoleBufferAddressLowerHalf:
                consoleBufferBaseAddress_.halves[0] = value;
                break;
            case Registers::ConsoleBufferAddressUpperHalf:
                consoleBufferBaseAddress_.halves[1] = value;
                break;
            default:
                break;
        }

    }
    void write8(Address address, uint8_t value) noexcept override {
        switch (static_cast<Registers>(address)) {
            case Registers::Led:
                writeLed(value);
                break;
            case Registers::DisplayMemoryReadsAndWrites:
                setDisplayMemoryReadsAndWrites(value != 0);
                break;
            case Registers::DisplayCacheLineUpdates:
                setDisplayCacheLineUpdates(value != 0);
                break;
            case Registers::PortZGPIO:
                processorInterface.writePortZGPIORegister(value);
                break;
            case Registers::PortZGPIOPullup:
                processorInterface.setPortZPullupResistorRegister(value);
                break;
            case Registers::PortZGPIOPolarity:
                processorInterface.setPortZPolarityRegister(value);
                break;
            case Registers::PortZGPIODirection:
                processorInterface.setPortZDirectionRegister(value);
                break;
            case Registers::ConsoleBufferLength:
                consoleBufferLength_ = value;
                break;
            case Registers::ConsoleBufferDoorbell:
                (void)writeToStream(Serial, consoleBufferBaseAddress_.full, consoleBufferLength_);
                break;
            default:
                break;
        }
    }
    [[nodiscard]] constexpr bool displayMemoryReadsAndWrites() const noexcept { return AllowDebuggingStatements && displayMemoryReadsAndWrites_; }
    [[nodiscard]] constexpr bool displayCacheLineUpdates() const noexcept { return AllowDebuggingStatements && displayCacheLineUpdates_; }
    void setDisplayMemoryReadsAndWrites(bool value) noexcept {
        if constexpr (AllowDebuggingStatements) {
            displayMemoryReadsAndWrites_ = value;
        }
    }
    void setDisplayCacheLineUpdates(bool value) noexcept {
        if constexpr (AllowDebuggingStatements) {
            displayCacheLineUpdates_ = value;
        }
    }
    [[nodiscard]] constexpr bool debuggingActive() const noexcept { return AllowDebuggingStatements; }
    void begin() noexcept override {
    }
private:
    static void
    writeLed(uint8_t value) noexcept {
        digitalWrite(i960Pinout::Led, value > 0 ? HIGH : LOW);
    }
    static uint8_t
    readLed() noexcept {
        return static_cast<uint8_t>(digitalRead(i960Pinout::Led));
    }
    bool displayMemoryReadsAndWrites_ = false;
    bool displayCacheLineUpdates_ = false;
    union {
        uint16_t halves[2];
        uint32_t full;
    } consoleBufferBaseAddress_;
    uint8_t consoleBufferLength_ = 0;
};

CoreChipsetFeatures chipsetFunctions(0);



class RAMThing : public MemoryMappedFile<TargetBoard::numberOfDataCacheLines(), TargetBoard::getDataCacheLineSize()> {
public:
    static constexpr Address MaxRamSize = 32 * 0x0100'0000; // 32 Memory Spaces or 512 Megabytes
    static constexpr auto RamMask = MaxRamSize - 1;
    static constexpr Address RamStartingAddress = 0x8000'0000; // start this at 512 megabytes
    static constexpr auto RamEndingAddress = RamStartingAddress + MaxRamSize;
    using Parent = MemoryMappedFile<TargetBoard::numberOfDataCacheLines(), TargetBoard::getDataCacheLineSize()>;
    RAMThing() noexcept : Parent(RamStartingAddress, RamEndingAddress, MaxRamSize, "ram.bin", FILE_WRITE) { }
    ~RAMThing() override = default;
    [[nodiscard]] Address
    makeAddressRelative(Address input) const noexcept override {
        // in this case, we want relative offsets
        return input & RamMask;
    }

    void begin() noexcept override {
        Parent::begin();
        if (Parent::getFileSize() != MaxRamSize) {
            signalHaltState(F("RAM.BIN MUST BE 512 MEGS IN SIZE!"));
        }
    }
};

class ROMThing : public MemoryMappedFile<TargetBoard::numberOfInstructionCacheLines(), TargetBoard::getInstructionCacheLineSize()> {
public:
    static constexpr Address ROMStart = 0;
    static constexpr Address ROMEnd = 0x2000'0000;
    static constexpr Address ROMMask = ROMEnd - 1;
    using Parent = MemoryMappedFile<TargetBoard::numberOfInstructionCacheLines(), TargetBoard::getInstructionCacheLineSize()>;
public:
    ROMThing() noexcept : Parent(ROMStart, ROMEnd, ROMEnd - 1, "boot.rom", FILE_READ){ }
    ~ROMThing() override = default;
    [[nodiscard]] uint16_t read(Address address, LoadStoreStyle style) noexcept override {
        auto result = MemoryThing::read(address, style);
        if (chipsetFunctions.displayMemoryReadsAndWrites()) {
            Serial.print(F("ROM: READING FROM ADDRESS 0x"));
            Serial.print(address, HEX);
            Serial.print(F(" yielded value 0x"));
            Serial.println(result, HEX);
        }
        return result;
    }
    [[nodiscard]] Address
    makeAddressRelative(Address input) const noexcept override {
        return input & ROMMask;
    }
    [[nodiscard]] bool
    respondsTo(Address address) const noexcept override {
        return address < Parent::getFileSize();
    }
    using MemoryThing::respondsTo;

};

/// @todo add support for the boot data section that needs to be copied into ram by the i960 on bootup
class DataROMThing : public MemoryMappedFile<1, 512> {
public:
    // two clusters are held onto at a time
    static constexpr uint32_t numCacheLines = 1;
    static constexpr uint32_t cacheLineSize = 512;
    static constexpr Address ROMStart = 0x2000'0000;
    static constexpr Address ROMEnd = 0x8000'0000;
    static constexpr Address DataSizeMax = ROMEnd - ROMStart;
    using Parent = MemoryMappedFile<1, 512>;
public:
    DataROMThing() noexcept : Parent(ROMStart, ROMEnd, DataSizeMax, "boot.dat", FILE_READ) { }
    ~DataROMThing() override = default;
    uint16_t read(Address address, LoadStoreStyle style) noexcept override {
        if constexpr (TargetBoard::onGrandCentralM4()) {
            Serial.print(F("DATA.ROM: READ FROM 0x"));
            Serial.println(address, HEX);
        }
        return MemoryThing::read(address, style);
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
    void flush() {
        display_.flush();
    }
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
        display_.fillScreen(ST77XX_BLACK);
        display_.setCursor(0, 0);
        display_.setTextColor(ST77XX_WHITE);
        display_.setTextSize(3);
        display_.println(F("i960Sx!"));
    }
    void
    clearScreen() {
        display_.fillScreen(ST7735_BLACK);
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

#ifndef ADAFRUIT_FEATHER
using DisplayThing = TFTShieldThing;
#else
using DisplayThing = AdafruitFeatherWingDisplay128x32Thing;
#endif
DisplayThing displayCommandSet(0x200);
RAMThing ram; // we want 4k but laid out for multiple sd card clusters, we can hold onto 8 at a time
ROMThing rom; // 4k rom sections
DataROMThing dataRom;

SDCardFilesystemInterface fs(0x300);
#ifdef ADAFRUIT_FEATHER
AdafruitLIS3MDLThing lsi3mdl(0x1000);
AdafruitLSM6DSOXThing lsm6dsox(0x1100);
AdafruitADT7410Thing adt7410(0x1200);
AdafruitADXL343Thing adxl343(0x1300);
#endif

// list of io memory devices to walk through
MemoryThing* things[] {
        &rom,
        &dataRom,
        &ram,
        &chipsetFunctions,
        &displayCommandSet,
#ifdef ADAFRUIT_FEATHER
        &lsi3mdl,
        &lsm6dsox,
        &adt7410,
        &adxl343,
#endif
        &fs,
};

MemoryThing*
getThing(Address address, LoadStoreStyle style) noexcept {
    for (auto* currentThing : things) {
        if (currentThing->respondsTo(address, style)) {
            return currentThing;
        }
    }
    return nullptr;
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
#ifdef ARDUINO_ARCH_SAMD
Adafruit_ZeroTimer burstTransactionTimer(3);
#endif
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
State tStart(nullptr, startupState, [](){Serial.println(F("LEAVING START"));});
State tSystemTest(nullptr, systemTestState, []() { Serial.println(F("LEAVING SYSTEM TEST"));});
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



volatile uint64_t cycleCount = 0;
volatile uint64_t previousCycleCount = 0;
void
enteringDataState() noexcept {
    if constexpr (!TargetBoard::onAtmega1284p()) {
#ifdef ARDUINO_ARCH_SAMD
        cycleCount = 0;
        previousCycleCount = cycleCount;
        burstTransactionTimer.enable(true);
#endif
        // since we are entering at this point, we count this is part of the cycle count so capture it
    }
    // when we do the transition, record the information we need
    processorInterface.newDataCycle();
}
bool
readyForNextDataRequest() noexcept {
    if constexpr (TargetBoard::onAtmega1284p()) {
        return true;
    } else {
        // we are on a much more powerful board so we have to wait around until we've hit the next 960 processor cycle
        return previousCycleCount < cycleCount;
    }
}
void processDataRequest() noexcept {
    if (readyForNextDataRequest()) {
        processorInterface.updateDataCycle();
        if (Address burstAddress = processorInterface.getAddress(); burstAddress < 0xFF00'0000) {
            // do not allow writes or reads into processor internal memory
            //processorInterface.setDataBits(performRead(burstAddress, style));
            LoadStoreStyle style = processorInterface.getStyle();
            Serial.print(F("\tBURST ADDRESS: 0x"));
            Serial.println(burstAddress, HEX);
            if (auto theThing = getThing(burstAddress, style); theThing) {
                if (processorInterface.isReadOperation()) {
                    processorInterface.setDataBits(theThing->read(burstAddress, style));
                } else {
                    theThing->write(burstAddress, processorInterface.getDataBits(), style);
                }
            } else {
                if (processorInterface.isReadOperation()) {
                    Serial.print(F("UNMAPPED READ FROM 0x"));
                } else {
                    Serial.print(F("UNMAPPED WRITE OF 0x"));
                    // expensive but something has gone horribly wrong anyway so whatever!
                    Serial.print(processorInterface.getDataBits(), HEX);
                    Serial.print(F(" TO 0x"));

                }
                Serial.println(burstAddress, HEX);
                delay(10);
            }
        }
        // setup the proper address and emit this over serial
        processorInterface.signalReady();
        if (processorInterface.blastTriggered()) {
            // we not in burst mode
            fsm.trigger(ReadyAndNoBurst);
        }
        if constexpr (!TargetBoard::onAtmega1284p()) {
            // we are now on _much_ faster boards if it isn't the 1284p
            previousCycleCount = cycleCount;
        }
    }
}

void doRecoveryState() noexcept {
#ifdef ARDUINO_ARCH_SAMD
    // turn of the burst transaction timer
    burstTransactionTimer.enable(false);
#endif
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
    displayReady = true;
    rom.begin();
    dataRom.begin();
    ram.begin();
#ifdef ADAFRUIT_FEATHER
    lsi3mdl.begin();
    lsm6dsox.begin();
    adt7410.begin();
    adxl343.begin();
#endif
    // setup the bus things
    Serial.println(F("Done setting up peripherals..."));
}
void setupClockSource() {
#ifdef ARDUINO_SAMD_FEATHER_M0
    // setup PORTS PA15 and PA20 as clock sources (D
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(4) | GCLK_GENDIV_DIV(2);
    GCLK->GENCTRL.reg = GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_ID(4) | GCLK_GENCTRL_SRC_DFLL48M;
    while (GCLK->STATUS.bit.SYNCBUSY);// Syncronize write to GENCTRL reg.
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(1) | GCLK_GENDIV_DIV(1);
    GCLK->GENCTRL.reg = GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_DFLL48M;
    while (GCLK->STATUS.bit.SYNCBUSY);// Syncronize write to GENCTRL reg.
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TC6_TC7_Val) | GCLK_CLKCTRL_GEN(GCLK_CLKCTRL_GEN_GCLK4_Val) | GCLK_CLKCTRL_CLKEN;
    PORT->Group[0].PMUX[20/2].reg |= PORT_PMUX_PMUXE_H;
    PORT->Group[0].PINCFG[20].reg |= PORT_PINCFG_PMUXEN; // enable mux for pin PA20
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TC4_TC5_Val) | GCLK_CLKCTRL_GEN(GCLK_CLKCTRL_GEN_GCLK1_Val) | GCLK_CLKCTRL_CLKEN;
    PORT->Group[0].PMUX[15/2].reg |= PORT_PMUX_PMUXO_H;
    PORT->Group[0].PINCFG[15].reg |= PORT_PINCFG_PMUXEN;
#endif
#ifdef ARDUINO_GRAND_CENTRAL_M4
    // pins on the digital block with access to the GCLK are:
    // 36 - GCLK / IO3
    // 37 - GCLK / IO2
    // 38 - GCLK / IO1
    // 39 - GCLK / IO0
    // let's choose pin 39 for this purpose
    constexpr float theTimerFrequency = 10_MHz;
    constexpr auto ClockDivider_10MHZ = 6;
    constexpr auto ClockDivider_12MHZ = 5;
    constexpr auto ClockDivider_15MHZ = 4;
    constexpr auto ClockDivider_20MHZ = 3;
    constexpr auto ClockDivider_40MHZ = 2; // THIS IS ALSO DAMN DANGEROUS
    constexpr auto ClockDivider_80MHZ = 1; // DEAR GOD DO NOT USE THIS!!!!
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_DIV(ClockDivider_10MHZ) |
                           GCLK_GENCTRL_IDC |
                           GCLK_GENCTRL_GENEN |
                           GCLK_GENCTRL_OE |
                           GCLK_GENCTRL_SRC_DPLL0;
    while(GCLK->SYNCBUSY.bit.GENCTRL0);
    PORT->Group[g_APinDescription[39].ulPort].PINCFG[g_APinDescription[39].ulPin].bit.PMUXEN = 1;
    // enable on pin 39 or PB14
    PORT->Group[g_APinDescription[39].ulPort].PMUX[g_APinDescription[39].ulPin >> 1].reg |= PORT_PMUX_PMUXE(MUX_PB14M_GCLK_IO0);
    // now we need to setup a timer which will count 10 MHz cycles and trigger an interrupt each time
    auto compare = TargetBoard::getCPUFrequency() / theTimerFrequency;
    auto divider = 1;
    auto prescaler = TC_CLOCK_PRESCALER_DIV1;
    Serial.print(F("Compare: "));
    Serial.println(compare);
    Serial.print(F("Divider: "));
    Serial.println(divider);
    Serial.print(F("Prescaler: "));
    Serial.println(prescaler);
    burstTransactionTimer.enable(false);
    burstTransactionTimer.configure(prescaler, TC_COUNTER_SIZE_16BIT, TC_WAVE_GENERATION_MATCH_FREQ);
    burstTransactionTimer.setCompare(0, compare);
    burstTransactionTimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, []() { ++cycleCount; });

#endif
}
// the setup routine runs once when you press reset:
void setup() {

    setupClockSource();
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
    fs.begin();
    chipsetFunctions.begin();
    Serial.println(F("i960Sx chipset bringup"));
    SPI.begin();
    processorInterface.begin();
    // setup the CPU Interface
    setupBusStateMachine();
    setupPeripherals();
    Serial.println(F("i960Sx chipset brought up fully!"));
    delay(1000);
    // we want to jump into the code as soon as possible after this point
}
void loop() {
    fsm.run_machine();
}

[[noreturn]]
void
signalHaltState(const __FlashStringHelper* haltMsg) {
    if (displayReady) {
        displayCommandSet.clearScreen();
        displayCommandSet.setCursor(0, 0);
        displayCommandSet.setTextSize(2);
        displayCommandSet.println(haltMsg);
    }
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}
void enteringChecksumFailure() noexcept {
    signalHaltState(F("CHECKSUM FAILURE!"));
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