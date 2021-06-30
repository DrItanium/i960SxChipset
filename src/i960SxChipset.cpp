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
#include <SdFat.h>

#include <Adafruit_TFTShield18.h>
#include <Adafruit_ST7735.h>
#include "Pinout.h"

#include "ProcessorSerializer.h"
#include "MemoryThing.h"
#include "MemoryMappedFileThing.h"
#include "SDCardFileSystemInterface.h"
#include "OPL2Thing.h"
#include "CoreChipsetFeatures.h"
#include "Cache.h"
#ifdef ADAFRUIT_FEATHER
#include "FeatherWingPeripherals.h"
#endif

#if defined(ARDUINO_GRAND_CENTRAL_M4)
#include <Adafruit_ZeroTimer.h>
#endif
//OPL2Thing<i960Pinout::SPI_BUS_A0, i960Pinout::SPI_BUS_A1, i960Pinout::SPI_BUS_A2> thingy(0x1000);

bool displayReady = false;
/**
 * @brief Describes a single cache line which associates an address with 16 bytes of storage
 */
ProcessorInterface& processorInterface = ProcessorInterface::getInterface();
// ----------------------------------------------------------------
// Load/Store routines
// ----------------------------------------------------------------

CoreChipsetFeatures chipsetFunctions(0);



class RAMFile : public MemoryMappedFile {
        //<TargetBoard::numberOfDataCacheLines(), TargetBoard::getDataCacheLineSize()>
public:
    static constexpr Address MaxRamSize = 32 * 0x0100'0000; // 32 Memory Spaces or 512 Megabytes
    static constexpr auto RamMask = MaxRamSize - 1;
    static constexpr Address RamStartingAddress = 0x8000'0000; // start this at 512 megabytes
    static constexpr auto RamEndingAddress = RamStartingAddress + MaxRamSize;
    using Parent = MemoryMappedFile;
    RAMFile() noexcept : Parent(RamStartingAddress, RamEndingAddress, MaxRamSize, "ram.bin", FILE_WRITE) { }
    ~RAMFile() override = default;
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
using RAMThing = DataCache<TargetBoard::numberOfDataCacheLines(), TargetBoard::getDataCacheLineSize()>;
using ROMThing = DataCache<TargetBoard::numberOfInstructionCacheLines(), TargetBoard::getInstructionCacheLineSize()>;

class ROMTextSection : public MemoryMappedFile {
public:
    static constexpr Address ROMStart = 0;
    static constexpr Address ROMEnd = 0x2000'0000;
    static constexpr Address ROMMask = ROMEnd - 1;
    using Parent = MemoryMappedFile;
public:
    ROMTextSection() noexcept : Parent(ROMStart, ROMEnd, ROMEnd - 1, "boot.rom", FILE_READ){ }
    ~ROMTextSection() override = default;
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
class ROMDataSection : public MemoryMappedFile {
public:
    // two clusters are held onto at a time
    static constexpr Address ROMStart = 0x2000'0000;
    static constexpr Address ROMEnd = 0x8000'0000;
    static constexpr Address DataSizeMax = ROMEnd - ROMStart;
    using Parent = MemoryMappedFile;
public:
    ROMDataSection() noexcept : Parent(ROMStart, ROMEnd, DataSizeMax, "boot.dat", FILE_READ) { }
    ~ROMDataSection() override = default;
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
RAMFile ramSection; // we want 4k but laid out for multiple sd card clusters, we can hold onto 8 at a time
ROMTextSection textSection;
ROMDataSection dataRom;
ROMThing rom(textSection);
RAMThing ram(ramSection);

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
// Tw - Wait State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in 

// Ti -> Ti via no request
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Ti after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted

// NOTE: Tw may turn out to be synthetic
#ifdef ARDUINO_ARCH_SAMD
Adafruit_ZeroTimer burstTransactionTimer(3); // I'm not going to be using tone on the grand central
#endif
constexpr auto NewRequest = 0;
constexpr auto ReadyAndNoBurst = 1;
constexpr auto ToDataState = 2;
constexpr auto PerformSelfTest = 3;
constexpr auto SelfTestComplete = 4;
void startupState() noexcept;
void systemTestState() noexcept;
void idleState() noexcept;
void doAddressState() noexcept;
void processDataRequest() noexcept;
void enteringDataState() noexcept;
void exitingDataState() noexcept;
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
            exitingDataState);


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
        signalHaltState(F("CHECKSUM FAILURE!"));
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



uint32_t cycleCount = 0;
SPISettings theSettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0);
MemoryThing* theThing = nullptr;
SplitWord16 burstCache[8] = { 0 };
void
enteringDataState() noexcept {
    // when we do the transition, record the information we need
    SPI.beginTransaction(theSettings);
    processorInterface.newDataCycle();
    if (!theThing->respondsTo(processorInterface.getAddress(), LoadStoreStyle::Full16)) {
        theThing = getThing(processorInterface.getAddress(), LoadStoreStyle::Full16);
    }
    if (!theThing) {
        // halt here because we've entered into unmapped memory state
        if (processorInterface.isReadOperation()) {
            Serial.print(F("UNMAPPED READ FROM 0x"));
        } else {
            Serial.print(F("UNMAPPED WRITE OF 0x"));
            // expensive but something has gone horribly wrong anyway so whatever!
            Serial.print(processorInterface.getDataBits(), HEX);
            Serial.print(F(" TO 0x"));

        }
        Serial.println(processorInterface.getAddress(), HEX);
        signalHaltState(F("UNMAPPED MEMORY REQUEST!"));
    }
}
void waitTillNexti960SxCycle() noexcept {
#ifdef ARDUINO_ARCH_SAMD
    // we are now on _much_ faster boards if it isn't the 1284p
    cycleCount = 0;
    // enable the timer and perform a wait based on the action
    burstTransactionTimer.enable(true);
    while (cycleCount != 0);
#endif
}
void processDataRequest() noexcept {
    processorInterface.updateDataCycle();
    // do not allow writes or reads into processor internal memory
    Address burstAddress = processorInterface.getAddress();
    LoadStoreStyle style = processorInterface.getStyle();
    if (processorInterface.isReadOperation()) {
        processorInterface.setDataBits(theThing->read(burstAddress, style));
    } else {
        theThing->write(burstAddress, processorInterface.getDataBits(), style);
    }
    // setup the proper address and emit this over serial
    processorInterface.signalReady();
    if (processorInterface.blastTriggered()) {
        // we not in burst mode
        fsm.trigger(ReadyAndNoBurst);
        return;
    } else {
        if constexpr (!TargetBoard::onAtmega1284p()) {
            waitTillNexti960SxCycle();
        }
    }
    processorInterface.updateDataCycle();
    // do not allow writes or reads into processor internal memory
    burstAddress = processorInterface.getAddress();
    style = processorInterface.getStyle();
    if (processorInterface.isReadOperation()) {
        processorInterface.setDataBits(theThing->read(burstAddress, style));
    } else {
        theThing->write(burstAddress, processorInterface.getDataBits(), style);
    }
    // setup the proper address and emit this over serial
    processorInterface.signalReady();
    if (processorInterface.blastTriggered()) {
        // we not in burst mode
        fsm.trigger(ReadyAndNoBurst);
        return;
    } else {
        if constexpr (!TargetBoard::onAtmega1284p()) {
            waitTillNexti960SxCycle();
        }
    }
}

void
exitingDataState() noexcept {
    SPI.endTransaction();
}


// ----------------------------------------------------------------
// setup routines
// ----------------------------------------------------------------

void setupBusStateMachine() noexcept {
    fsm.add_transition(&tStart, &tSystemTest, PerformSelfTest, nullptr);
    fsm.add_transition(&tSystemTest, &tIdle, SelfTestComplete, nullptr);
    fsm.add_transition(&tIdle, &tAddr, NewRequest, nullptr);
    fsm.add_transition(&tAddr, &tData, ToDataState, nullptr);
    fsm.add_transition(&tData, &tIdle, ReadyAndNoBurst, nullptr);
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
    constexpr auto theTimerFrequency = 10_MHz;
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
    uint32_t compare = TargetBoard::getCPUFrequency() / theTimerFrequency;
    auto prescaler = TC_CLOCK_PRESCALER_DIV1;
#if 0
    Serial.print(F("Compare: "));
    Serial.println(compare);
    Serial.print(F("Divider: "));
    Serial.println(divider);
    Serial.print(F("Prescaler: "));
    Serial.println(prescaler);
#endif
    burstTransactionTimer.enable(false);
    burstTransactionTimer.configure(prescaler, TC_COUNTER_SIZE_16BIT, TC_WAVE_GENERATION_MATCH_FREQ);
    burstTransactionTimer.setCompare(0, compare);
    burstTransactionTimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0,
                                      []() {
                                          ++cycleCount;
                                          burstTransactionTimer.enable(false);
                                      });

#endif
}
// the setup routine runs once when you press reset:
void setup() {

    Serial.begin(115200);
    while(!Serial) {
        delay(10);
    }
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
    theThing = &rom;
    fs.begin();
    chipsetFunctions.begin();
    Serial.println(F("i960Sx chipset bringup"));
    SPI.begin();
    processorInterface.begin();
    // setup the CPU Interface
    setupBusStateMachine();
    setupPeripherals();
    delay(1000);
    Serial.println(F("i960Sx chipset brought up fully!"));
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
SdFat SD;
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

#ifdef ARDUINO_ARCH_SAMD
void
TC3_Handler()
{
    Adafruit_ZeroTimer::timerHandler(3);
}
#endif