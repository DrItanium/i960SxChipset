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

#include <ArduinoJson.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_seesaw.h>
#include <Adafruit_TFTShield18.h>
#include <Adafruit_ST7735.h>
#include "Pinout.h"
#include "BusDevice.h"
#include "SPIBus.h"
#include "IOExpanders.h"



/**
 * Normally generated by the linker as the value used for validation purposes
 * on system bootup. Since we are on a microcontroller, this can be done
 * dynamically. This is taken from start.ld in the SA/SB Reference manual's
 * Appendix D.
 */
constexpr auto computeCS1(uint32_t satPtr, uint32_t pcrbPtr, uint32_t startIP) noexcept {
	return - (satPtr + pcrbPtr + startIP);
}
volatile bool tftSetup = false;
Adafruit_TFTShield18 ss;
Adafruit_ST7735 tft(static_cast<int>(i960Pinout::DISPLAY_EN),
                     static_cast<int>(i960Pinout::DC),
                     -1);
constexpr bool displaySDCardStatsDuringInit = false;
/// Set to false to prevent the console from displaying every single read and write
constexpr bool displayMemoryReadsAndWrites = false;
// boot rom and sd card loading stuff
Sd2Card theBootSDCard;
SdVolume theBootVolume;
SdFile rootDirectory;
SdFile theBootROM;
SdFile theRAM; // use an SDCard as ram for the time being
uint32_t bootRomSize = 0;

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

SPIBus theSPIBus;
// with the display I want to expose a 16 color per pixel interface. Each specific function needs to be exposed
// set a single pixel in the display, storage area
// we map these pixel values to specific storage cells on the microcontroller
// A four address is used to act as a door bell!
// storage area for the display + a doorbell address as well
// the command is setup to send off to the display

template<typename DisplayType>
class DisplayCommand {
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
    void setCommand(uint16_t command) noexcept { command_ = command; }
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
    uint16_t command_ = 0;
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
volatile bool asTriggered = false;
volatile bool denTriggered = false;
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
State tAddr([]() { asTriggered = false; }, 
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
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(PerformSelfTest);
	}
}
void systemTestState() noexcept {
    if (DigitalPin<i960Pinout::FAIL>::isDeasserted()) {
		fsm.trigger(SelfTestComplete);
	}
}
void onASAsserted() {
    asTriggered = true;
}
void onDENAsserted() {
    denTriggered = true;
}

void idleState() noexcept {
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(ChecksumFailure);
	} else {
		if (asTriggered) {
			fsm.trigger(NewRequest);
		}
	}
}
void doAddressState() noexcept {
	if (denTriggered) {
		fsm.trigger(ToDataState);
	}
}



void
enteringDataState() noexcept {
	// when we do the transition, record the information we need
	denTriggered = false;
	baseAddress = getAddress();
	performingRead = isReadOperation();
}
LoadStoreStyle getStyle() noexcept { return static_cast<LoadStoreStyle>(getByteEnableBits()); }
void
ioSpaceWrite8(Address offset, uint8_t value) noexcept {
    switch (offset) {
        case 0: // builtin led
            digitalWrite(i960Pinout::Led, value > 0 ? HIGH : LOW);
            if constexpr (displayMemoryReadsAndWrites) {
                Serial.println(F("LED WRITE!"));
            }
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
            digitalWrite(i960Pinout::Led, (value & 0xFF) > 0 ? HIGH : LOW);
            /// @todo figure out if writes like this should be ignored?
            /// @todo write to address 1 as well since it would be both, but do not go through the write8 interface
            break;
        case ConsoleFlushPort:
            Serial.flush();
            break;
        case ConsoleReadWriteBaseAddress:
            Serial.write(static_cast<uint8_t>(value));
            break;

        default:
            break;
    }
}
void
ioSpaceWrite(Address address, uint16_t value, LoadStoreStyle style) noexcept {
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
            if constexpr (displayMemoryReadsAndWrites) {
                // this is the internal IO space
                Serial.println(F("Request to write into IO space"));
            }
            ioSpaceWrite(address, value, style);
        } else if (address < RamEndingAddress){
           // we are writing to "RAM" at this point, what it consists of at this point is really inconsequential
           // for the initial design it is just going to be straight off of the SDCard itself, slow but a great test
            // in the ram section
            // now we need to make it relative to 512 megabytes
            auto actualAddress = RamMask & address;
            theRAM.seekSet(actualAddress);
            /// @todo figure out what to do if we couldn't read enough?
            switch (style) {
                case LoadStoreStyle::Upper8:
                    theRAM.seekCur(1); // jump ahead by one
                    theRAM.write(value >> 8);
                    break;
                case LoadStoreStyle::Lower8:
                    theRAM.write(&value, 1);
                    break;
                case LoadStoreStyle::Full16:
                    theRAM.write(&value, 2);
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
ioSpaceRead8(Address offset) noexcept {
    switch (offset) {
        case 0:
            if constexpr (displayMemoryReadsAndWrites) {
                Serial.println(F("LED READ!"));
            }
            return static_cast<uint8_t>(digitalRead(i960Pinout::Led));
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
            return static_cast<uint16_t>(digitalRead(i960Pinout::Led));
        case ConsoleReadWriteBaseAddress:
            return static_cast<uint16_t>(Serial.read());
        case ConsoleAvailableForWrite:
            return static_cast<uint16_t>(Serial.availableForWrite());
        default:
            return 0;
    }
}

uint16_t
ioSpaceRead(Address address, LoadStoreStyle style) noexcept {
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
            theBootROM.seekSet(address);
            theBootROM.read(&result, 2);
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
            theRAM.seekSet(actualAddress); // jump to that point in memory
            /// @todo figure out what to do if we couldn't read enough?
            theRAM.read(&output, 2);
            // do not evaluate the load store style in this case because the processor will do the ignoring
            return output;
        }
    }
    return 0;
}
void processDataRequest() noexcept {
    auto burstAddress = getBurstAddress(baseAddress);
	if (performingRead) {
		setDataBits(performRead(burstAddress, getStyle()));
	} else {
	    performWrite(burstAddress, getDataBits(), getStyle());
	}
	// setup the proper address and emit this over serial
	auto blastPin = getBlastPin();
	DigitalPin<i960Pinout::Ready>::pulse();
	if (blastPin == LOW) {
		// we not in burst mode
		fsm.trigger(ReadyAndNoBurst);
	} 

}

void doRecoveryState() noexcept {
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(ChecksumFailure);
	} else {
		if (asTriggered) {
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
void setupCPUInterface() {
    Serial.println(F("Setting up interrupts!"));
	setupPins(OUTPUT,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Int0_);
	digitalWriteBlock(HIGH,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Int0_);
	setHOLDPin(LOW);
	setLOCKPin(HIGH);
	setupPins(INPUT,
			i960Pinout::BLAST_,
			i960Pinout::AS_,
			i960Pinout::W_R_,
			i960Pinout::DEN_,
			i960Pinout::FAIL);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::AS_)), onASAsserted, FALLING);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::DEN_)), onDENAsserted, FALLING);
    Serial.println(F("Done setting up interrupts!"));
}
void setupIOExpanders() {
    Serial.println(F("Setting up IOExpanders!"));
	// at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
	// disabled. We can send out a single IOCON.HAEN enable message and all
	// should receive it. 
	// so do a begin operation on all chips (0b000)
	dataLines.begin(); 
	// set IOCON.HAEN on all chips
	dataLines.enableHardwareAddressPins();
	// now we have to refresh our on mcu flags for each io expander
	lower16.refreshIOCon();
	upper16.refreshIOCon();
	extraMemoryCommit.refreshIOCon();
	// now all devices tied to this ~CS pin have separate addresses
	// make each of these inputs
	lower16.writeGPIOsDirection(0xFFFF);
	upper16.writeGPIOsDirection(0xFFFF);
	dataLines.writeGPIOsDirection(0xFFFF);
	// set lower eight to inputs and upper eight to outputs
	extraMemoryCommit.writeGPIOsDirection(0x00FF);
	// then indirectly mark the outputs
	pinMode(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), OUTPUT, extraMemoryCommit);
	pinMode(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), OUTPUT, extraMemoryCommit);
	theSPIBus.setup();
    Serial.println(F("Done setting up io expanders!"));
}

constexpr auto computeAddressStart(Address start, Address size, Address count) noexcept {
    return start + (size * count);
}
// we have access to 12 Winbond Flash Modules, which hold onto common program code, This gives us access to 96 megabytes of Flash.
// At this point it is a massive pain in the ass to program all these devices but who cares

void setupTFT() {
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
    tft.setCursor(0,0);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(3);
    tft.println(F("i960Sx!"));
}
template<typename T>
[[noreturn]] void signalHaltState(T haltMsg) {
    if (!tftSetup) {
        Serial.println(haltMsg);
    } {
        tft.fillScreen(ST77XX_RED);
        tft.setCursor(0,0);
        tft.setTextSize(1);
        tft.println(haltMsg);
    }
    while(true) {
        delay(1000);
    }
}

void setupSDCard() {
    Serial.println(F("Bringing up SD CARD"));
    if constexpr (displaySDCardStatsDuringInit) {
        Serial.print(F("SD Card Enable Pin: "));
        Serial.println(static_cast<int>(i960Pinout::SD_EN));
        Serial.println(static_cast<int>(i960Pinout::SD_MOSI));
        Serial.println(static_cast<int>(i960Pinout::SD_MISO));
        Serial.println(static_cast<int>(i960Pinout::SD_SCK));
    }
        // the sd card is on a separate SPI Bus in some cases
    if (!theBootSDCard.init(SPI_FULL_SPEED,
                            static_cast<int>(i960Pinout::SD_EN)
#ifdef ARDUINO_GRAND_CENTRAL_M4
,
                            static_cast<int>(i960Pinout::SD_MOSI),
                            static_cast<int>(i960Pinout::SD_MISO),
                            static_cast<int>(i960Pinout::SD_SCK)
#endif
    )) {
        Serial.println(F("SD Card initialization failed"));
        Serial.println(F("Make sure of the following:"));
        Serial.println(F("1) Is an SD Card is inserted?"));
        Serial.println(F("2) Is the wiring is correct?"));
        Serial.println(F("3) Does the ~CS pin match the shield or module?"));
        signalHaltState(F("NO SD CARD"));
    }
    Serial.println(F("SD Card initialization successful"));
    if constexpr (displaySDCardStatsDuringInit) {
        Serial.println();
        Serial.println(F("Card Info"));
        Serial.println(F("Card Type:\t"));
        switch (theBootSDCard.type()) {
            case SD_CARD_TYPE_SD1:
                Serial.println(F("SD1"));
                break;
            case SD_CARD_TYPE_SD2:
                Serial.println(F("SD2"));
                break;
            case SD_CARD_TYPE_SDHC:
                Serial.println(F("SDHC"));
                break;
            default:
                Serial.println(F("Unknown"));
                break;
        }
    }
    if (!theBootVolume.init(theBootSDCard)) {
        Serial.println(F("Could not find a valid FAT16/FAT32 parition"));
        Serial.println(F("Make sure you've formatted the card!"));
        signalHaltState(F("BAD SD FORMAT"));
    }
    if constexpr (displaySDCardStatsDuringInit) {
        Serial.print(F("Clusters:\t"));
        Serial.println(theBootVolume.clusterCount());
        Serial.print(F("Blocks x Cluster:\t"));
        Serial.println(theBootVolume.blocksPerCluster());
        Serial.print(F("Total Blocks:\t"));
        Serial.println(theBootVolume.blocksPerCluster() * theBootVolume.clusterCount());
        Serial.println();

        Serial.print(F("Volume type is: FAT"));
        Serial.println(theBootVolume.fatType(), DEC);
        uint32_t volumeSize = theBootVolume.blocksPerCluster(); // clusters are collections of blocks
        volumeSize *= theBootVolume.clusterCount(); // sd cards have many clusters
        volumeSize /= 2; // SD Card blocks are always 512 bytes
        Serial.print(F("Volume size (Kb):\t"));
        Serial.println(volumeSize);
        Serial.print(F("Volume size (Mb):\t"));
        Serial.println(volumeSize / 1024);
        Serial.print(F("Volume size (Gb):\t"));
        Serial.println(static_cast<float>(volumeSize / 1024 / 1024));

        Serial.println();
        Serial.println(F("Files found on the card (name, date and size in bytes): "));
    }
    rootDirectory.openRoot(theBootVolume);
    if constexpr (displaySDCardStatsDuringInit) {
        rootDirectory.ls(LS_R | LS_DATE | LS_SIZE);
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println();
    }
    Serial.print(F("Checking for file /boot.rom...."));
    if (!theBootROM.open(rootDirectory, "boot.rom", O_READ)) {
        Serial.println(F("FAILED!"));
        signalHaltState(F("NO BOOT.ROM"));
    }
    Serial.println(F("FOUND!"));
    Serial.print(F("Size of boot.rom: 0x"));
    bootRomSize = theBootROM.fileSize();
    if (bootRomSize >= RamStartingAddress) {
        Serial.println(F("BOOT.ROM is too large!"));
        signalHaltState(F("BOOT.ROM TOO LARGE!"));
    }
    if (bootRomSize == 0) {
        Serial.println(F("BOOT.ROM is empty!"));
        signalHaltState(F("BOOT.ROM EMPTY!"));
    }
    Serial.print(theBootROM.fileSize(), HEX);
    Serial.println(F(" bytes"));
    Serial.print(F("Checking for file /ram.bin...."));
    if (!theRAM.open(rootDirectory, "ram.bin", O_RDWR)) {
        Serial.println(F("NOT FOUND!"));
        signalHaltState(F("NO RAM.BIN"));
    }
    /// @todo lookup the sdcard map information
    Serial.println(F("FOUND!"));
    Serial.print(F("Size of ram.bin: 0x"));
    Serial.print(theRAM.fileSize(), HEX);
    Serial.println(F(" bytes"));
    rootDirectory.close();

    Serial.println(F("Successfully setup SD card"));
}
void
setupSeesaw() {
    Serial.println(F("Setting up the seesaw"));
    if (!ss.begin()) {
        Serial.println(F("seesaw could not be initialized!"));
        signalHaltState("NO SEESAW");
    }
    Serial.println(F("seesaw started"));
    Serial.print(F("Version: "));
    Serial.println(ss.getVersion(), HEX);

}
void setupPeripherals() {
    Serial.println(F("Setting up peripherals..."));
    setupSeesaw();
    setupTFT();
    setupSDCard();
    Serial.println(F("Done setting up peripherals..."));
}
// the setup routine runs once when you press reset:
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println(F("i960Sx chipset bringup"));
    setupPins(OUTPUT,
              i960Pinout::Reset960,
              i960Pinout::SPI_BUS_EN,
              i960Pinout::DISPLAY_EN,
              i960Pinout::SD_EN);

	digitalWrite(i960Pinout::SPI_BUS_EN, HIGH);
    digitalWrite(i960Pinout::SD_EN, HIGH);
    digitalWrite(i960Pinout::DISPLAY_EN, HIGH);
    pinMode(i960Pinout::Led, OUTPUT);
    digitalWrite(i960Pinout::Led, LOW);
    Wire.begin();
    SPI.begin();
	PinAsserter<i960Pinout::Reset960> holdi960InReset;
	setupIOExpanders();
	setupCPUInterface();
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
    tft.fillScreen(ST77XX_RED);
    tft.setCursor(0,0);
    tft.setTextSize(2);
    tft.println(F("CHECKSUM FAILURE"));
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