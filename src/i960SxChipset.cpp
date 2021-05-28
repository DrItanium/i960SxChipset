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
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADT7410.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "Pinout.h"
#include "ProcessorSerializer.h"

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
// the upper 64 elements of the bus are exposed for direct processor usage

static constexpr auto FlashStartingAddress = 0x0000'0000;
static constexpr Address OneMemorySpace = 0x100'0000; // 16 megabytes
// the upper 2G is for non-program related stuff, according to my memory map information, we support a maximum of 512 Megs of RAM
// so the "ram" file is 512 megs in size. If the range is between 0x8000'0000 and
static constexpr Address OneMemorySpaceMask = OneMemorySpace - 1;
static constexpr Address MaxRamSize = 32 * OneMemorySpace; // 32 Memory Spaces or 512 Megabytes
static constexpr auto RamMask = MaxRamSize - 1;
static constexpr Address RamStartingAddress = 0x8000'0000;
static constexpr auto RamEndingAddress = RamStartingAddress + MaxRamSize;


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
    if (processorInterface.failTriggered()) {
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
void
writeLed(uint8_t value) noexcept {
    digitalWrite(i960Pinout::Led, value > 0 ? HIGH : LOW);
    Serial.println(F("LED WRITE!"));
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
        case 0x100:
            Serial.print(static_cast<char>(value));
            Serial.flush();
            break;
        default:
            break;
    }
}
void
ioSpaceWrite(Address address, uint16_t value, ProcessorInterface::LoadStoreStyle style) noexcept {
    Serial.print("IO Address: 0x");
    Serial.println(address, HEX);
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
        Serial.print(F("Write 0b"));
        Serial.print(value, BIN);
        Serial.print(F(" to 0b"));
        Serial.println(address, BIN);
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
    Serial.println(F("LED READ!"));
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
ioSpaceRead16(Address offset) noexcept {
    switch (offset) {
        case 0:
            /// @todo this would be an amalgamation of the contents addresses zero and one
            /// @todo figure out if we should ignore 16-bit writes to 8-bit registers or not... it could be gross
            return static_cast<uint16_t>(readLed());
        case 0x100:
            return static_cast<int16_t>(Serial.read());
        default:
            return 0;
    }
}

uint16_t
ioSpaceRead(Address address, ProcessorInterface::LoadStoreStyle style) noexcept {
    Serial.print("IO Address: 0x");
    Serial.println(address, HEX);
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
        Serial.print(F("Read from 0b"));
        Serial.println(address, BIN);
    }
    /// @todo implement
    if (address < RamStartingAddress) {
        if (address < bootRomSize) {
            uint16_t result = 0;
            // okay cool beans, we can load from the boot rom
            theBootROM.seek(address);
            theBootROM.read(&result, 2);
            if constexpr (displayMemoryReadsAndWrites) {
                Serial.print(F("\tGot value: 0b"));
                Serial.println(result, BIN);
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
    if (TargetBoard::onFeatherBoard()) {
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
#ifdef ARDUINO_GRAND_CENTRAL_M4
    #if 0
    // pins on the digital block with access to the GCLK are:
    // 36 - GCLK / IO3
    // 37 - GCLK / IO2
    // 38 - GCLK / IO1
    // 39 - GCLK / IO0
    // let's choose pin 39 for this purpose
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_DIV(6) |
                           GCLK_GENCTRL_IDC |
                           GCLK_GENCTRL_GENEN |
                           GCLK_GENCTRL_OE |
                           GCLK_GENCTRL_SRC_DPLL0;
    while(GCLK->SYNCBUSY.bit.GENCTRL0);
    PORT->Group[g_APinDescription[39].ulPort].PINCFG[g_APinDescription[39].ulPin].bit.PMUXEN = 1;
    // enable on pin 39 or PB14
    PORT->Group[g_APinDescription[39].ulPort].PMUX[g_APinDescription[39].ulPin >> 1].reg |= PORT_PMUX_PMUXE(MUX_PB14M_GCLK_IO0);
#endif
#endif // end ARDUINO_GRAND_CENTRAL_M4
    Serial.begin(115200);
    while (!Serial);
    if constexpr (TargetBoard::onAtmega1284p()) {
        if constexpr (!TargetBoard::usesDisplayShield()) {
            static_assert()
            Serial.println(F("ASSERTION FAILURE"));
        }
    }
    Serial.println(F("i960Sx chipset bringup"));
    Wire.begin();
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