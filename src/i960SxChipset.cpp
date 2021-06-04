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
#include <Wire.h>

#include <Adafruit_TFTShield18.h>
#include <Adafruit_ST7735.h>
#include "Pinout.h"

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADT7410.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "Pinout.h"
#include "ProcessorSerializer.h"
#include "DependentFalse.h"
#include "MemoryThing.h"

bool tftSetup = false;
Adafruit_TFTShield18 ss;
Adafruit_ST7735 tft(static_cast<int>(i960Pinout::DISPLAY_EN),
                     static_cast<int>(i960Pinout::DC),
                     -1);
/// Set to false to prevent the console from displaying every single read and write
constexpr bool displayMemoryReadsAndWrites = false;
// boot rom and sd card loading stuff
File theBootROM;
uint32_t bootRomSize = 0;
ProcessorInterface processorInterface;
template<typename T>
constexpr Address computeIOAddress(Address base, Address index) noexcept {
    return base + (sizeof(T) * index);
}
constexpr Address shortWordMemoryAddress(Address base, Address index) noexcept {
    return computeIOAddress<uint16_t>(base, index);
}
constexpr Address wordMemoryAddress(Address base, Address index) noexcept {
    return computeIOAddress<uint32_t>(base, index);
}
constexpr Address longWordMemoryAddress(Address base, Address index) noexcept {
    return computeIOAddress<uint64_t>(base, index);
}
constexpr auto FlashStartingAddress = 0x0000'0000;
constexpr Address OneMemorySpace = 0x0100'0000; // 16 megabytes
// the upper 2G is for non-program related stuff, according to my memory map information, we support a maximum of 512 Megs of RAM
// so the "ram" file is 512 megs in size. If the range is between 0x8000'0000 and
constexpr Address OneMemorySpaceMask = OneMemorySpace - 1;
constexpr Address MaxRamSize = 32 * OneMemorySpace; // 32 Memory Spaces or 512 Megabytes
constexpr auto RamMask = MaxRamSize - 1;
constexpr Address RamStartingAddress = 0x8000'0000;
constexpr auto RamEndingAddress = RamStartingAddress + MaxRamSize;
constexpr Address IOSpaceBaseAddress = 0xFE00'0000;
constexpr Address BaseMCUPeripheralsBaseAddress = 0;
constexpr Address BuiltinLedOffsetBaseAddress = BaseMCUPeripheralsBaseAddress;
constexpr Address BuiltinPortZBaseAddress = BaseMCUPeripheralsBaseAddress + 0x10;
/**
 * @brief Describes the Addresses associated with the port-z mmio registers
 */
enum class PortZAddresses : uint32_t {
    GPIO, // one byte wide
    Direction, // one byte wide
    Polarity,
    Pullup,
    Unused0,
    Unused1,
    Unused2,
    Unused3,
    Unused4,
    Unused5,
    Unused6,
    Unused7,
    Unused8,
    Unused9,
    Unused10,
    Unused11,
    Count,
};
static_assert(static_cast<int>(PortZAddresses::Count) == 0x10, "PortZRegister space must be exactly 16 entries wide!");
constexpr Address computePortZAddresses(PortZAddresses offset) noexcept {
    return BuiltinPortZBaseAddress + static_cast<uint32_t>(offset);
}
constexpr Address ConsoleOffsetBaseAddress = BaseMCUPeripheralsBaseAddress + 0x20;
enum class ConsoleAddresses : uint32_t {
    Flush,
    IO,
    Available,
    AvailableForWrite,
};
constexpr Address computeConsoleAddress(ConsoleAddresses offset) noexcept {
    return shortWordMemoryAddress(ConsoleOffsetBaseAddress, static_cast<uint32_t>(offset));
}
constexpr Address DisplayBaseOffset = 0x0200;
constexpr Address TFTShieldFeaturesBaseOffset = 0x0300;


uint16_t backlightFrequency = 0;
uint16_t backlightStatus = TFTSHIELD_BACKLIGHT_ON;
uint32_t buttonsCache = 0;

// with the display I want to expose a 16 color per pixel interface. Each specific function needs to be exposed
// set a single pixel in the display, storage area
// we map these pixel values to specific storage cells on the microcontroller
// A four address is used to act as a door bell!
// storage area for the display + a doorbell address as well
// the command is setup to send off to the display
template<typename DisplayType>
class DisplayCommand {
public:
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
                display_.fillCircle(x0_, y0_, x1_, y1_, x2_, y2_, color_);
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
        }
        return 0;
    }
public:
    /// @todo extend BusDevice and take in a base address
    explicit DisplayCommand(DisplayType& display) : display_(display) { }
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
    const DisplayType& getAssociatedDisplay() const noexcept { return display_; }
    DisplayType& getAssociatedDisplay() noexcept { return display_; }
    void flush() { display_.flush(); }
    void print(char c) { display_.print(c); }
    [[nodiscard]] bool available() noexcept { return true; }
    [[nodiscard]] bool availableForWriting() noexcept { return display_.availableForWrite(); }
private:
    DisplayType& display_;
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
};

enum class TFTShieldAddresses : uint32_t {
    // display tweakables
    Flush,
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
    /// @todo implement constexpr offset calculation
};

constexpr Address tftShieldAddress(TFTShieldAddresses address) noexcept {
    return shortWordMemoryAddress(TFTShieldFeaturesBaseOffset, static_cast<Address>(address));
}
/// The base address which describes the different memory map aspects, it is accessible from the i960's memory map
constexpr Address MemoryMapBase = 0xFD00'0000;
union MemoryMapEntry {
    constexpr MemoryMapEntry(Address value) : value_(value) {}
    Address value_;
    uint16_t shorts[2];
};
enum class MemoryMapAddresses : uint32_t {
    ProgramSpaceStart,
    ProgramSpaceEnd,
    RamSpaceSize,
    RamSpaceStart,
    RamSpaceEnd,
    IOSpaceSize,
    IOSpaceStart,
    IOSpaceEnd,
    BuiltinLEDAddress,
    ConsoleFlushRegisterAddress,
    ConsoleIOPort,
    ConsoleAvailablePort,
    ConsoleAvailableForWritePort,
    PortZGPIORegister,
    PortZDirectionRegister,
    PortZPolarityRegister,
    PortZPullupRegister,
    DisplayFlush,
    DisplayIO,
    DisplayAvailable,
    DisplayAvailableForWrite,
    DisplayCommand,
    DisplayX,
    DisplayY,
    DisplayW,
    DisplayH,
    DisplayRadius,
    DisplayColor,
    DisplayX0,
    DisplayY0,
    DisplayX1,
    DisplayY1,
    DisplayX2,
    DisplayY2,
    DisplayR,
    DisplayG,
    DisplayB,
    DisplayDoorbell,
    DisplayBacklight,
    DisplayBacklightFrequency,
    DisplayButtonsLower,
    DisplayButtonsUpper,
};
constexpr MemoryMapEntry FixedMemoryMap[] {
        0x0000'0000, RamStartingAddress, // Program Space Range [start, end)
        MaxRamSize, RamStartingAddress, RamEndingAddress,  // ram data: size, start, end values
        0x100'0000, IOSpaceBaseAddress, 0xFF00'0000, // IO Space [Start, End)
        BuiltinLedOffsetBaseAddress, // address of the builtin-led
        // Console addresses
        computeConsoleAddress(ConsoleAddresses::Flush),
        computeConsoleAddress(ConsoleAddresses::IO),
        computeConsoleAddress(ConsoleAddresses::Available),
        computeConsoleAddress(ConsoleAddresses::AvailableForWrite),
        // Port z addresses
        computePortZAddresses(PortZAddresses::GPIO),
        computePortZAddresses(PortZAddresses::Direction),
        computePortZAddresses(PortZAddresses::Polarity),
        computePortZAddresses(PortZAddresses::Pullup),
        // display command setup
        tftShieldAddress(TFTShieldAddresses::Flush),
        tftShieldAddress(TFTShieldAddresses::IO),
        tftShieldAddress(TFTShieldAddresses::Available),
        tftShieldAddress(TFTShieldAddresses::AvailableForWrite),
        tftShieldAddress(TFTShieldAddresses::Command),
        tftShieldAddress(TFTShieldAddresses::X),
        tftShieldAddress(TFTShieldAddresses::Y),
        tftShieldAddress(TFTShieldAddresses::W),
        tftShieldAddress(TFTShieldAddresses::H),
        tftShieldAddress(TFTShieldAddresses::Radius),
        tftShieldAddress(TFTShieldAddresses::Color),
        tftShieldAddress(TFTShieldAddresses::X0),
        tftShieldAddress(TFTShieldAddresses::Y0),
        tftShieldAddress(TFTShieldAddresses::X1),
        tftShieldAddress(TFTShieldAddresses::Y1),
        tftShieldAddress(TFTShieldAddresses::X2),
        tftShieldAddress(TFTShieldAddresses::Y2),
        tftShieldAddress(TFTShieldAddresses::R),
        tftShieldAddress(TFTShieldAddresses::G),
        tftShieldAddress(TFTShieldAddresses::B),
        tftShieldAddress(TFTShieldAddresses::Doorbell),
        tftShieldAddress(TFTShieldAddresses::Backlight),
        tftShieldAddress(TFTShieldAddresses::BacklightFrequency),
        tftShieldAddress(TFTShieldAddresses::ButtonsLower),
        tftShieldAddress(TFTShieldAddresses::ButtonsUpper),
};

constexpr MemoryMapEntry getMemoryMapEntry(MemoryMapAddresses address) noexcept {
    return FixedMemoryMap[static_cast<uint32_t>(address)];
}
constexpr Address memoryMapAddress(MemoryMapAddresses address) noexcept {
    return getMemoryMapEntry(address).value_;
}
DisplayCommand<decltype(tft)> displayCommandSet(tft);

// for the feather m0 only
// LIS3MDL + LSM6DSOX featherwing
Adafruit_LSM6DSOX lsm6ds;
Adafruit_LIS3MDL lis3mdl;
// adxl343 + adt7410 featherwing
Adafruit_ADT7410 tempSensor;
Adafruit_ADXL343 accel1(12345);
Adafruit_SSD1306 display(128,32,&Wire);

bool oledDisplaySetup = false;
template<typename T>
[[noreturn]] void signalHaltState(T haltMsg) {
    if (tftSetup) {
        tft.fillScreen(ST77XX_RED);
        tft.setCursor(0,0);
        tft.setTextSize(1);
        tft.println(haltMsg);
    } else if (oledDisplaySetup) {
        display.clearDisplay();
        display.display();
        display.setCursor(0,0);
        display.println(haltMsg);
        display.display();
    }
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}
// ----------------------------------------------------------------
// Load/Store routines
// ----------------------------------------------------------------
/**
 * @brief An intermediate type which automatically adds the IOBaseAddress to the start and end addresses
 */
class IOSpaceThing : public MemoryThing {
public:
    IOSpaceThing(Address base, Address end) : MemoryThing(base + IOSpaceBaseAddress, end + IOSpaceBaseAddress) { }
    explicit IOSpaceThing(Address base) : MemoryThing(base + IOSpaceBaseAddress) { }
    ~IOSpaceThing() override = default;
};
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
        if constexpr (displayMemoryReadsAndWrites) {
            Serial.println(F("LED WRITE!"));
        }
        digitalWrite(i960Pinout::Led, value > 0 ? HIGH : LOW);
    }
    static uint8_t
    readLed() noexcept {
        if constexpr (displayMemoryReadsAndWrites) {
            Serial.println(F("LED READ!"));
        }
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
        Unused0,
        Unused1,
        Unused2,
        Unused3,
        Unused4,
        Unused5,
        Unused6,
        Unused7,
        Unused8,
        Unused9,
        Unused10,
        Unused11,
        Count,
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
    [[nodiscard]] uint16_t read16(Address offset) noexcept override {
        // do nothing at this point as it doesn't make sense!
        return 0;
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
    void write16(Address offset, uint16_t value) noexcept override {
        // 16-bit writes are ignored
    }
};


class ConsoleThing : public IOSpaceThing {
public:
    enum class Registers : uint32_t {
        Flush = 0x0, // 2 byte
        Available = 0x2, // 2 byte
        AvailableForWrite = 0x4, // 2 byte
        IO = 0x6, // 2 bytes
        /// @todo add support for DMA style printing via the HOLD/HLDA signals
    };
public:
    // make sure we allocate a ton of space just in case
    explicit ConsoleThing(Address base) noexcept : IOSpaceThing(base, base + 0x100) { }
    ~ConsoleThing() override = default;

    [[nodiscard]] uint8_t read8(Address offset) noexcept override { return 0; }
    [[nodiscard]] uint16_t read16(Address offset) noexcept override {
        switch (static_cast<Registers>(offset)) {
            case Registers::Available:
                return Serial.available();
            case Registers::AvailableForWrite:
                return Serial.availableForWrite();
            case Registers::IO:
                return Serial.read();
            default:
                return 0;
        }
    }
    void write8(Address offset, uint8_t value) noexcept override {
        // do nothing
    }
    void write16(Address offset, uint16_t value) noexcept override {
        // 16-bit writes are ignored
        switch (static_cast<Registers>(offset)) {
            case Registers::IO:
                Serial.write(static_cast<char>(value));
                // always flush afterwards
            case Registers::Flush:
                Serial.flush();
                break;
            default:
                break;
        }
    }

};

class RAMThing : public MemoryThing {
public:
    RAMThing() noexcept : MemoryThing(RamStartingAddress, RamEndingAddress) { }
    ~RAMThing() override = default;
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
            theRAM = SD.open("ram.bin", FILE_WRITE);
            Serial.println(F("RAM.BIN OPEN SUCCESS!"));
        }
    }
private:
    File theRAM_; // use an SDCard as ram for the time being


};

BuiltinLedThing theLed;
PortZThing portZThing(BuiltinPortZBaseAddress);
ConsoleThing theConsole(0x100);

void
ioSpaceWrite8(Address offset, uint8_t value) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print(F("ioSpaceWrite8 to Offset: 0x"));
        Serial.println(offset, HEX);
        Serial.println(memoryMapAddress(MemoryMapAddresses::BuiltinLEDAddress), HEX);
    }
    switch (offset) {
        case memoryMapAddress(MemoryMapAddresses::DisplayFlush):
            displayCommandSet.flush();
            break;
        case memoryMapAddress(MemoryMapAddresses::DisplayIO):
            displayCommandSet.print(static_cast<char>(value));
            break;
        default:
            break;
    }
}
uint8_t
ioSpaceRead8(Address offset) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print(F("ioSpaceRead8 to Offset: 0x"));
        Serial.println(offset, HEX);
    }
    switch (offset) {
        case memoryMapAddress(MemoryMapAddresses::DisplayAvailable):
            return static_cast<uint16_t>(displayCommandSet.available());
        case memoryMapAddress(MemoryMapAddresses::DisplayAvailableForWrite):
            return static_cast<uint16_t>(displayCommandSet.availableForWriting());
        default:
            return 0;
    }
}
void
ioSpaceWrite16(Address offset, uint16_t value) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print(F("ioSpaceWrite16 to Offset: 0x"));
        Serial.println(offset, HEX);
    }

    // we are writing to two separate addresses
    switch (offset) {
        case memoryMapAddress(MemoryMapAddresses::DisplayBacklight):
            backlightStatus = value != 0 ? TFTSHIELD_BACKLIGHT_ON : TFTSHIELD_BACKLIGHT_OFF;
            ss.setBacklight(backlightStatus);
            break;
        case memoryMapAddress(MemoryMapAddresses::DisplayBacklightFrequency):
            backlightFrequency = value;
            ss.setBacklightFreq(backlightFrequency);
            break;
        default:
            break;
    }
}
uint16_t
ioSpaceRead16(Address offset) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print(F("ioSpaceRead16 to Offset: 0x"));
        Serial.println(offset, HEX);
    }
    switch (offset) {
        case memoryMapAddress(MemoryMapAddresses::DisplayBacklight):
            return backlightStatus;
        case memoryMapAddress(MemoryMapAddresses::DisplayBacklightFrequency):
            return backlightFrequency;
        case memoryMapAddress(MemoryMapAddresses::DisplayButtonsLower):
            buttonsCache = ss.readButtons();
            return static_cast<uint16_t>(buttonsCache & 0x0000'FFFF);
        case memoryMapAddress(MemoryMapAddresses::DisplayButtonsUpper):
            return static_cast<uint16_t>((buttonsCache >> 16) & 0x0000'FFFF);
        default:
            return 0;
    }
}
void
ioSpaceWrite(Address address, uint16_t value, LoadStoreStyle style) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print("IO Address: 0x");
        Serial.println(address, HEX);
    }
    if (theLed.respondsTo(address, style)) {
        theLed.write(address, value, style);
    } else if (portZThing.respondsTo(address, style)) {
        portZThing.write(address, value, style);
    } else if (theConsole.respondsTo(address, style)) {
        theConsole.write(address, value, style);
    } else {
        auto offset = 0x00FF'FFFF & address;
        switch (style) {
            case LoadStoreStyle::Upper8:
                // it is the next byte address over
                ioSpaceWrite8(offset + 1, value >> 8);
                break;
            case LoadStoreStyle::Lower8:
                ioSpaceWrite8(offset, value);
                break;
            case LoadStoreStyle::Full16:
                ioSpaceWrite16(offset, value);
                break;
            default:
                break;
        }
    }
}
void
performWrite(Address address, uint16_t value, LoadStoreStyle style) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print(F("Write 0x"));
        Serial.print(value, HEX);
        Serial.print(F(" to 0x"));
        Serial.println(address, HEX);
    }
    if (address < RamStartingAddress) {
        // we are in program land for the time being so do nothing!
    } else {
        // upper half needs to be walked down further
        if (address >= 0xFF00'0000) {
            // cpu internal space, we should never get to this location
            if (displayMemoryReadsAndWrites) {
                Serial.println(F("Request to write into CPU internal space"));
            }
        } else if ((address >= 0xFE00'0000) && (address < 0xFF00'0000)) {
            ioSpaceWrite(address, value, style);
        } else if (address < RamEndingAddress){
            auto thePtr = reinterpret_cast<uint8_t*>(&value);
            // we are writing to "RAM" at this point, what it consists of at this point is really inconsequential
            // for the initial design it is just going to be straight off of the SDCard itself, slow but a great test
            // in the ram section
            // now we need to make it relative to 512 megabytes
            auto actualAddress = RamMask & address;
            /// @todo figure out what to do if we couldn't read enough?
            switch (style) {
                case LoadStoreStyle::Upper8:
                    theRAM.seek(actualAddress + 1);
                    theRAM.write(value >> 8);
                    break;
                case LoadStoreStyle::Lower8:
                    theRAM.seek(actualAddress);
                    theRAM.write(static_cast<uint8_t>(value));
                    break;
                case LoadStoreStyle::Full16:
                    theRAM.seek(actualAddress);
                    theRAM.write(thePtr, 2);
                    break;
                default:
                    break;
            }
            theRAM.flush();
        }
    }
    /// @todo implement
}

uint16_t
ioSpaceRead(Address address, LoadStoreStyle style) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print("IO Address: 0x");
        Serial.println(address, HEX);
    }
    if (theLed.respondsTo(address, style)) {
        return theLed.read(address, style);
    } else if (portZThing.respondsTo(address, style)) {
        return portZThing.read(address, style);
    } else if (theConsole.respondsTo(address, style)) {
        return theConsole.read(address, style);
    } else {
        auto offset = 0x00FF'FFFF & address;
        switch (style) {
            case LoadStoreStyle::Full16:
                return ioSpaceRead16(offset);
            case LoadStoreStyle::Upper8:
                // next address over
                // then make sure it is returned in the upper portion
                return static_cast<uint16_t>(ioSpaceRead8(offset + 1)) << 8;
            case LoadStoreStyle::Lower8:
                return static_cast<uint16_t>(ioSpaceRead8(offset));
            default:
                return 0;
        }
    }
}
uint16_t
performRead(Address address, LoadStoreStyle style) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print(F("Read from 0x"));
        Serial.println(address, HEX);
    }
    /// @todo implement
    if (address < RamStartingAddress) {
        if (address < bootRomSize) {
            uint16_t result = 0;
            // okay cool beans, we can load from the boot rom
            theBootROM.seek(address);
            theBootROM.read(&result, 2);
            if constexpr (displayMemoryReadsAndWrites) {
                Serial.print(F("\tGot value: 0x"));
                Serial.println(result, HEX);
            }
            return result;
        }
        // read from flash
    } else {
        // upper half needs to be walked down further
        if (address >= 0xFF00'0000) {
            // cpu internal space, we should never get to this location
            if constexpr (displayMemoryReadsAndWrites) {
                Serial.println(F("Request to read from CPU internal space?"));
            }
        } else if ((address >= 0xFE00'0000) && (address < 0xFF00'0000)) {
            // this is the internal IO space
            return ioSpaceRead(address, style);
        } else if ((address >= MemoryMapBase) && (address < IOSpaceBaseAddress)) {
        } else if (address < RamEndingAddress){
            uint16_t output = 0;
            // in the ram section
            // now we need to make it relative to 512 megabytes
            auto actualAddress = RamMask & address;
            theRAM.seek(actualAddress); // jump to that point in memory
            /// @todo figure out what to do if we couldn't read enough?
            theRAM.read(&output, 2);
            // do not evaluate the load store style in this case because the processor will do the ignoring
            return output;
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
    Serial.print(F("Setting up bus state machine..."));
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
    Serial.println(F("done"));
}
//State tw(nullptr, nullptr, nullptr); // at this point, this will be synthetic
//as we have no concept of waiting inside of the mcu

constexpr auto computeAddressStart(Address start, Address size, Address count) noexcept {
    return start + (size * count);
}
// we have access to 12 Winbond Flash Modules, which hold onto common program code, This gives us access to 96 megabytes of Flash.
// At this point it is a massive pain in the ass to program all these devices but who cares


void setupSDCard() {
    if (!SD.begin(static_cast<int>(i960Pinout::SD_EN))) {
        signalHaltState(F("SD CARD INIT FAILed"));
    }
    if (!SD.exists("boot.rom")) {
        signalHaltState(F("NO BOOT.ROM!"));
    } else {
        theBootROM = SD.open("boot.rom", FILE_READ);
        Serial.println(F("BOOT.ROM OPEN SUCCESS!"));
        bootRomSize = theBootROM.size();
        if (bootRomSize == 0) {
            signalHaltState(F("EMPTY BOOT.ROM"));
        } else if (bootRomSize > 0x8000'0000) {
            signalHaltState(F("BOOT.ROM TOO LARGE")) ;
        }

    }

    if (!SD.exists("ram.bin")) {
        signalHaltState(F("NO RAM.BIN FOUND!"));
    } else {
        theRAM = SD.open("ram.bin", FILE_WRITE);
        Serial.println(F("RAM.BIN OPEN SUCCESS!"));
    }


    // the sd card is on a separate SPI Bus in some cases
}
void setupTFTShield() {
    Serial.println(F("Setting up the seesaw"));
    if (!ss.begin()) {
        signalHaltState(F("NO SEESAW"));
    }
    Serial.println(F("seesaw started"));
    Serial.print(F("Version: "));
    Serial.println(ss.getVersion(), HEX);
    ss.setBacklight(TFTSHIELD_BACKLIGHT_OFF);
    ss.tftReset();
    tft.initR(INITR_BLACKTAB); // initialize a ST7735S, black tab
    ss.setBacklight(TFTSHIELD_BACKLIGHT_ON);
    Serial.println(F("TFT OK!"));
    tft.fillScreen(ST77XX_CYAN);
    Serial.println(F("Screen should have cyan in it!"));
    delay(100);
    tftSetup = true;
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(3);
    tft.println(F("i960Sx!"));
}
void bringupAnalogDevicesFeatherWing() {
    if constexpr (TargetBoard::onFeatherBoard()) {
        if (!accel1.begin()) {
            signalHaltState(F("ADXL343 Bringup Failure"));
        }
        // configure for your project
        accel1.setRange(ADXL343_RANGE_16_G);
        if (!tempSensor.begin()) {
            signalHaltState(F("ADT7410 Bringup Failure"));
        }
        // sensor takes 250 ms to get readings
        delay(250);
    }
}
void bringupOLEDFeatherWing() {
    if constexpr (TargetBoard::onFeatherBoard()) {
        Serial.println(F("Setting up OLED Featherwing"));
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
        Serial.println(F("OLED begun"));
        display.display();
        delay(1000);
        display.clearDisplay();
        display.display();
        // I have cut the pins for the buttons on the featherwing display
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(F("i960Sx!"));
        display.display();
        Serial.println(F("Done setting up OLED Featherwing"));
        oledDisplaySetup = true;
    }
}
void bringupLIS3MDLAndLSM6DS() {
    if constexpr (TargetBoard::onFeatherBoard()) {
        Serial.println(F("Bringing up LIS3MDL"));
        if (!lis3mdl.begin_I2C()) {
            signalHaltState(F("FAILED TO BRING UP LIS3MDL"));
        }
        if (!lsm6ds.begin_I2C()) {
            signalHaltState(F("FAILED TO BRING UP LSM6DS"));
        }
    }
}
void setupPeripherals() {
    Serial.println(F("Setting up peripherals..."));
    if constexpr (TargetBoard::onFeatherBoard()) {
        bringupOLEDFeatherWing();
        bringupAnalogDevicesFeatherWing();
        bringupLIS3MDLAndLSM6DS();
    } else {
        setupTFTShield();
    }
    setupSDCard();
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
              i960Pinout::Led,
              i960Pinout::Ready,
              i960Pinout::GPIOSelect,
              i960Pinout::Int0_);
    PinAsserter<i960Pinout::Reset960> holdi960InReset;
    digitalWrite(i960Pinout::Led, LOW);
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
    Serial.begin(115200);
    while (!Serial);
    Serial.println(F("i960Sx chipset bringup"));
    Serial.println(F("Memory Map Information: "));
    for (unsigned int i = 0; i < (sizeof(FixedMemoryMap) / sizeof(MemoryMapEntry)) ;++i) {
        Serial.print(i);
        Serial.print(F(": 0x"));
        Serial.println(FixedMemoryMap[i].value_, HEX);
    }
    SPI.begin();
    processorInterface.begin();
    // setup the CPU Interface
    processorInterface.setHOLDPin(LOW);
    processorInterface.setLOCKPin(HIGH);
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
    if (tftSetup) {
        tft.fillScreen(ST77XX_RED);
        tft.setCursor(0, 0);
        tft.setTextSize(2);
        tft.println(msg);
    } else if (oledDisplaySetup) {
        display.clearDisplay();
        display.display();
        display.setCursor(0, 0);
        display.println(msg);
        display.display();
    }
    Serial.println(msg);
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