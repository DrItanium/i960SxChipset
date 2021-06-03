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

bool tftSetup = false;
Adafruit_TFTShield18 ss;
Adafruit_ST7735 tft(static_cast<int>(i960Pinout::DISPLAY_EN),
                     static_cast<int>(i960Pinout::DC),
                     -1);
constexpr bool displaySDCardStatsDuringInit = false;
/// Set to false to prevent the console from displaying every single read and write
constexpr bool displayMemoryReadsAndWrites = false;
// boot rom and sd card loading stuff
File theBootROM;
File theRAM; // use an SDCard as ram for the time being
uint32_t bootRomSize = 0;
ProcessorInterface processorInterface;

constexpr auto FlashStartingAddress = 0x0000'0000;
constexpr Address OneMemorySpace = 0x0100'0000; // 16 megabytes
// the upper 2G is for non-program related stuff, according to my memory map information, we support a maximum of 512 Megs of RAM
// so the "ram" file is 512 megs in size. If the range is between 0x8000'0000 and
constexpr Address OneMemorySpaceMask = OneMemorySpace - 1;
constexpr Address MaxRamSize = 32 * OneMemorySpace; // 32 Memory Spaces or 512 Megabytes
constexpr auto RamMask = MaxRamSize - 1;
constexpr Address RamStartingAddress = 0x8000'0000;
constexpr auto RamEndingAddress = RamStartingAddress + MaxRamSize;

constexpr Address AbsoluteConsoleBaseAddress = 0xFE00'0100;
constexpr Address ConsoleOffsetBaseAddress = 0x0100;
constexpr Address ConsoleReadWriteBaseAddress = ConsoleOffsetBaseAddress + 0x2;
constexpr Address ConsoleFlushPort = ConsoleOffsetBaseAddress + 0x0;
constexpr Address ConsoleAvailableForWrite = ConsoleOffsetBaseAddress + 0x4;
constexpr Address DisplayBaseOffset = 0x0200;
constexpr Address TFTShieldFeaturesBaseOffset = 0x0300;
constexpr Address DisplayBacklightPort = TFTShieldFeaturesBaseOffset + 0x00;
constexpr Address DisplayBacklightFrequencyPort = TFTShieldFeaturesBaseOffset + 0x02;
constexpr Address DisplayButtonsPort = TFTShieldFeaturesBaseOffset + 0x04; // 32-bits

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
        SetTextSize,
        SetCursor,
        SetTextColor0,
        SetTextColor1,
        SetTextWrap,
        GetWidth,
        GetHeight,
        GetRotation,
        GetCursorX,
        GetCursorY,
        WritePixel,
        WriteColor,
        WriteFillRect,
        DrawPixel,
    };
public:
    /// @todo extend BusDevice and take in a base address
    DisplayCommand(DisplayType& display) : display_(display) { }
    [[nodiscard]] constexpr auto getCommand() const noexcept { return command_; }
    [[nodiscard]] constexpr auto getX() const noexcept { return x_; }
    [[nodiscard]] constexpr auto getY() const noexcept { return y_; }
    [[nodiscard]] constexpr auto getW() const noexcept { return w_; }
    [[nodiscard]] constexpr auto getH() const noexcept { return h_; }
    [[nodiscard]] constexpr auto getRadius() const noexcept { return radius_; }
    [[nodiscard]] constexpr uint16_t getColor() const noexcept { return color_; }
    [[nodiscard]] constexpr auto getX0() const noexcept { return x0_; }
    [[nodiscard]] constexpr auto getY0() const noexcept { return y0_; }
    [[nodiscard]] constexpr auto getX1() const noexcept { return x1_; }
    [[nodiscard]] constexpr auto getY1() const noexcept { return y1_; }
    [[nodiscard]] constexpr auto getX2() const noexcept { return x2_; }
    [[nodiscard]] constexpr auto getY2() const noexcept { return y2_; }
    void setCommand(Opcodes command) noexcept { command_ = command; }
    void setX(int16_t x) noexcept { x_ = x; }
    void setY(int16_t y) noexcept { y_ = y; }
    void setW(int16_t w) noexcept { w_ = w; }
    void setH(int16_t h) noexcept { h_ = h; }
    void setRadius(int16_t radius) noexcept { radius_ = radius; }
    void setColor(uint16_t color) noexcept { color_ = color; }
    void setX0(int16_t value) noexcept { x0_ = value; }
    void setY0(int16_t value) noexcept { y0_ = value; }
    void setX1(int16_t value) noexcept { x1_ = value; }
    void setY1(int16_t value) noexcept { y1_ = value; }
    void setX2(int16_t value) noexcept { x2_ = value; }
    void setY2(int16_t value) noexcept { y2_ = value; }
    const DisplayType& getAssociatedDisplay() const noexcept { return display_; }
    DisplayType& getAssociatedDisplay() noexcept { return display_; }
    /**
     * @brief Invoke on doorbell write
     * @param value the value written to the doorbell
     */
    void invoke(uint16_t value) {
        // perhaps we'll do nothing with the value but hold onto it for now
    }
private:
    DisplayType& display_;
    Opcodes command_;
    int16_t x_ = 0;
    int16_t y_ = 0;
    int16_t w_ = 0;
    int16_t h_ = 0;
    int16_t radius_ = 0;
    uint16_t color_ = 0;
    int16_t x0_ = 0;
    int16_t y0_ = 0;
    int16_t x1_ = 0;
    int16_t y1_ = 0;
    int16_t x2_ = 0;
    int16_t y2_ = 0;
};

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
// ----------------------------------------------------------------
// Load/Store routines
// ----------------------------------------------------------------
void
writeLed(uint8_t value) noexcept {
    digitalWrite(i960Pinout::Led, value > 0 ? HIGH : LOW);
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.println(F("LED WRITE!"));
    }
}
void
ioSpaceWrite8(Address offset, uint8_t value) noexcept {
    switch (offset) {
        case 0: // builtin led
            writeLed(value);
            break;
        default:
            break;
    }
}
void
ioSpaceWrite16(Address offset, uint16_t value) noexcept {
    // we are writing to two separate addresses
    switch (offset) {
        case 0: // and 1
            writeLed(value & 0xFF);
            /// @todo figure out if writes like this should be ignored?
            /// @todo write to address 1 as well since it would be both, but do not go through the write8 interface
            break;
        case ConsoleFlushPort:
            Serial.flush();
            break;
        case ConsoleReadWriteBaseAddress:
            Serial.write(static_cast<uint8_t>(value));
            Serial.flush();
            break;
        case DisplayBacklightPort:
            backlightStatus = value != 0 ? TFTSHIELD_BACKLIGHT_ON : TFTSHIELD_BACKLIGHT_OFF;
            ss.setBacklight(backlightStatus);
            break;
        case DisplayBacklightFrequencyPort:
            backlightFrequency = value;
            ss.setBacklightFreq(backlightFrequency);
            break;
        default:
            break;
    }
}
uint16_t
ioSpaceRead16(Address offset) noexcept {
    switch (offset) {
        case 0:
            /// @todo this would be an amalgamation of the contents addresses zero and one
            /// @todo figure out if we should ignore 16-bit writes to 8-bit registers or not... it could be gross
            return static_cast<uint16_t>(digitalRead(i960Pinout::Led));
        case ConsoleReadWriteBaseAddress:
            return static_cast<uint16_t>(Serial.read());
        case ConsoleAvailableForWrite:
            return static_cast<uint16_t>(Serial.availableForWrite());
        case DisplayBacklightPort:
            return backlightStatus;
        case DisplayBacklightFrequencyPort:
            return backlightFrequency;
        case DisplayButtonsPort + 0x00: // lower two bytes
            buttonsCache = ss.readButtons();
            return static_cast<uint16_t>(buttonsCache & 0x0000'FFFF);
        case DisplayButtonsPort + 0x02: // upper two bytes
            return static_cast<uint16_t>((buttonsCache >> 16) & 0x0000'FFFF);
        default:
            return 0;
    }
}
void
ioSpaceWrite(Address address, uint16_t value, ProcessorInterface::LoadStoreStyle style) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print("IO Address: 0x");
        Serial.println(address, HEX);
    }
    auto offset = 0x00FF'FFFF & address;
    switch (style) {
        case ProcessorInterface::LoadStoreStyle::Upper8:
            // it is the next byte address over
            ioSpaceWrite8(offset + 1, value >> 8);
            break;
        case ProcessorInterface::LoadStoreStyle::Lower8:
            ioSpaceWrite8(offset, value);
            break;
        case ProcessorInterface::LoadStoreStyle::Full16:
            ioSpaceWrite16(offset, value);
            break;
        default:
            break;
    }
}
void
performWrite(Address address, uint16_t value, ProcessorInterface::LoadStoreStyle style) noexcept {
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
            if constexpr (displayMemoryReadsAndWrites) {
                // this is the internal IO space
                Serial.println(F("Request to write into IO space"));
            }
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
                case ProcessorInterface::LoadStoreStyle::Upper8:
                    theRAM.seek(actualAddress + 1);
                    theRAM.write(value >> 8);
                    break;
                case ProcessorInterface::LoadStoreStyle::Lower8:
                    theRAM.seek(actualAddress);
                    theRAM.write(static_cast<uint8_t>(value));
                    break;
                case ProcessorInterface::LoadStoreStyle::Full16:
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
uint8_t
readLed() noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.println(F("LED READ!"));
    }
    return static_cast<uint8_t>(digitalRead(i960Pinout::Led));
}
uint8_t
ioSpaceRead8(Address offset) noexcept {
    switch (offset) {
        case 0:
            return readLed();
        default:
            return 0;
    }
}

uint16_t
ioSpaceRead(Address address, ProcessorInterface::LoadStoreStyle style) noexcept {
    if constexpr (displayMemoryReadsAndWrites) {
        Serial.print("IO Address: 0x");
        Serial.println(address, HEX);
    }
    auto offset = 0x00FF'FFFF & address;
    switch (style) {
        case ProcessorInterface::LoadStoreStyle::Full16:
            return ioSpaceRead16(offset);
        case ProcessorInterface::LoadStoreStyle::Upper8:
            // next address over
            // then make sure it is returned in the upper portion
            return static_cast<uint16_t>(ioSpaceRead8(offset + 1)) << 8;
        case ProcessorInterface::LoadStoreStyle::Lower8:
            return static_cast<uint16_t>(ioSpaceRead8(offset));
        default:
            return 0;
    }
}
uint16_t
performRead(Address address, ProcessorInterface::LoadStoreStyle style) noexcept {
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
            if constexpr (displayMemoryReadsAndWrites) {
                Serial.println(F("Request to read from IO space"));
            }
            return ioSpaceRead(address, style);
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
void bringupOLEDFeatherWing() {
    Serial.println(F("Setting up OLED Featherwing")) ;
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    Serial.println(F("OLED begun"));
    display.display();
    delay(1000);
    display.clearDisplay();
    display.display();
    // I have cut the pins for the buttons on the featherwing display
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("i960Sx!"));
    display.display();
    Serial.println(F("Done setting up OLED Featherwing"));
    oledDisplaySetup = true;
}
void bringupLIS3MDLAndLSM6DS() {
    Serial.println(F("Bringing up LIS3MDL"));
    if (!lis3mdl.begin_I2C()) {
        signalHaltState(F("FAILED TO BRING UP LIS3MDL"));
    }
    if (!lsm6ds.begin_I2C()) {
        signalHaltState(F("FAILED TO BRING UP LSM6DS")) ;
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