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
#include <Timer.h>
#include <SD.h>
#include <Wire.h>

#include <ArduinoJson.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
using Address = uint32_t;

enum class i960Pinout : decltype(A0) {
    // PORT B
    Led = 0, 	  // output
    CLOCK_OUT, // output, unusable
    AS_,     // input, AVR Int2
    DC, 	 // output
    GPIOSelect,		// output
    MOSI,		  // reserved
    MISO,		  // reserved
    SCK, 		  // reserved
// PORT D
    RX0, 		  // reserved
    TX0, 		  // reserved
    DEN_,	  // AVR Interrupt INT0
    AVR_INT1, 		// AVR Interrupt INT1
    PWM0,	       //
    PWM1, 		  // unused
    PWM2, 		  // unused
    PWM3, 		  // unused
// PORT C
    SCL,		  // reserved
    SDA, 		  // reserved
    Ready, 	  // output
    Int0_,		  // output
    W_R_, 		  // input
    Reset960,		  // output
    BLAST_, 	 // input
    FAIL, 	     // input
// PORT A
    SPI_BUS_EN, // output
    Analog1,
    Analog2,
    Analog3,
    Analog4,
    Analog5,
    Analog6,
    Analog7,
    Count,		  // special
};
static_assert(static_cast<decltype(HIGH)>(i960Pinout::Count) <= 32);

inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
	digitalWrite(static_cast<int>(ip), value);
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
	pinMode(static_cast<int>(ip), value);
}

inline auto digitalRead(i960Pinout ip) {
	return digitalRead(static_cast<int>(ip));
}

enum class IOExpanderAddress : byte {
	DataLines = 0b000,
	Lower16Lines,
	Upper16Lines,
	MemoryCommitExtras,
	OtherDevice0,
	OtherDevice1,
	OtherDevice2,
	OtherDevice3,
};
template<i960Pinout pin>
struct DigitalPin {
	DigitalPin() = delete;
	~DigitalPin() = delete;
	DigitalPin(const DigitalPin&) = delete;
	DigitalPin(DigitalPin&&) = delete;
	DigitalPin& operator=(const DigitalPin&) = delete;
	DigitalPin& operator=(DigitalPin&&) = delete;
	static constexpr bool isInputPin() noexcept { return false; }
	static constexpr bool isOutputPin() noexcept { return false; }
	static constexpr bool getDirection() noexcept { return false; }
	static constexpr auto getPin() noexcept { return pin; }
};

#define DefOutputPin(pin, asserted, deasserted) \
	template<> \
	struct DigitalPin< pin > { \
	static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
		DigitalPin() = delete; \
		~DigitalPin() = delete; \
		DigitalPin(const DigitalPin&) = delete; \
		DigitalPin(DigitalPin&&) = delete; \
		DigitalPin& operator=(const DigitalPin&) = delete; \
		DigitalPin& operator=(DigitalPin&&) = delete; \
		static constexpr auto isInputPin() noexcept { return false; } \
		static constexpr auto isOutputPin() noexcept { return true; } \
		static constexpr auto getPin() noexcept { return pin; } \
		static constexpr auto getDirection() noexcept { return OUTPUT; } \
		static constexpr auto getAssertionState() noexcept { return asserted; } \
		static constexpr auto getDeassertionState() noexcept { return deasserted; } \
		inline static void assert() noexcept { digitalWrite(pin, getAssertionState()); } \
		inline static void deassert() noexcept { digitalWrite(pin, getDeassertionState()); } \
		inline static void write(decltype(LOW) value) noexcept { digitalWrite(pin, value); } \
		inline static void pulse() noexcept { \
			assert(); \
			deassert(); \
		} \
	}
#define DefInputPin(pin, asserted, deasserted) \
	template<> \
	struct DigitalPin< pin > { \
		static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
		DigitalPin() = delete; \
		~DigitalPin() = delete; \
		DigitalPin(const DigitalPin&) = delete; \
		DigitalPin(DigitalPin&&) = delete; \
		DigitalPin& operator=(const DigitalPin&) = delete; \
		DigitalPin& operator=(DigitalPin&&) = delete; \
		static constexpr auto isInputPin() noexcept { return true; } \
		static constexpr auto isOutputPin() noexcept { return false; } \
		static constexpr auto getPin() noexcept { return pin; } \
		static constexpr auto getDirection() noexcept { return INPUT; } \
		static constexpr auto getAssertionState() noexcept { return asserted; } \
		static constexpr auto getDeassertionState() noexcept { return deasserted; } \
		inline static auto read() noexcept { return digitalRead(pin); } \
		inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
		inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
	}
#define DefInputPullupPin(pin, asserted, deasserted) \
	template<> \
	struct DigitalPin< pin > { \
		static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
		DigitalPin() = delete; \
		~DigitalPin() = delete; \
		DigitalPin(const DigitalPin&) = delete; \
		DigitalPin(DigitalPin&&) = delete; \
		DigitalPin& operator=(const DigitalPin&) = delete; \
		DigitalPin& operator=(DigitalPin&&) = delete; \
		static constexpr auto isInputPin() noexcept { return true; } \
		static constexpr auto isOutputPin() noexcept { return false; } \
		static constexpr auto getPin() noexcept { return pin; } \
		static constexpr auto getDirection() noexcept { return INPUT_PULLUP; } \
		static constexpr auto getAssertionState() noexcept { return asserted; } \
		static constexpr auto getDeassertionState() noexcept { return deasserted; } \
		inline static bool isAsserted() noexcept { return digitalRead(pin) == getAssertionState(); } \
		inline static bool isDeasserted() noexcept { return digitalRead(pin) == getDeassertionState(); } \
		inline static auto read() noexcept { return digitalRead(pin); } \
	}
DefOutputPin(i960Pinout::GPIOSelect, LOW, HIGH);
DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefOutputPin(i960Pinout::SPI_BUS_EN, LOW, HIGH);
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::AS_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
#undef DefInputPin
#undef DefOutputPin
#undef DefInputPullupPin

/**
 * Normally generated by the linker as the value used for validation purposes
 * on system bootup. Since we are on a microcontroller, this can be done
 * dynamically. This is taken from start.ld in the SA/SB Reference manual's
 * Appendix D.
 */
constexpr auto computeCS1(uint32_t satPtr, uint32_t pcrbPtr, uint32_t startIP) noexcept {
	return - (satPtr + pcrbPtr + startIP);
}

template<IOExpanderAddress addr, int enablePin = static_cast<int>(i960Pinout::GPIOSelect)>
using IOExpander = bonuspin::MCP23S17<static_cast<int>(addr), enablePin>;


template<typename ... Pins>
void setupPins(decltype(OUTPUT) direction, Pins ... pins) {
	(pinMode(pins, direction), ...);
}

template<typename ... Pins>
void digitalWriteBlock(decltype(HIGH) value, Pins ... pins) {
	(digitalWrite(pins, value), ...);
}

template<i960Pinout pinId, decltype(HIGH) onConstruction, decltype(LOW) onDestruction>
class PinToggler {
	public:
		PinToggler() { digitalWrite(pinId, onConstruction); }
		~PinToggler() { digitalWrite(pinId, onDestruction); }
};

template<i960Pinout pinId>
using HoldPinLow = PinToggler<pinId, LOW, HIGH>;

template<i960Pinout pinId>
using HoldPinHigh = PinToggler<pinId, HIGH, LOW>;

template<i960Pinout pinId>
class PinAsserter {
	public:
		static_assert(DigitalPin<pinId>::isOutputPin());
		PinAsserter() { DigitalPin<pinId>::assert(); }
		~PinAsserter() { DigitalPin<pinId>::deassert(); }
};

// 8 IOExpanders to a single enable line for SPI purposes
// 4 of them are reserved
IOExpander<IOExpanderAddress::DataLines> dataLines;
IOExpander<IOExpanderAddress::Lower16Lines> lower16;
IOExpander<IOExpanderAddress::Upper16Lines> upper16;
IOExpander<IOExpanderAddress::MemoryCommitExtras> extraMemoryCommit;
Timer t;
Adafruit_ILI9341 tft(static_cast<int>(i960Pinout::SPI_BUS_EN),
                     static_cast<int>(i960Pinout::DC));

Address
getAddress() noexcept {
	auto lower16Addr = static_cast<Address>(lower16.readGPIOs());
	auto upper16Addr = static_cast<Address>(upper16.readGPIOs()) << 16;
	return lower16Addr | upper16Addr;
}
uint16_t
getDataBits() noexcept {
	dataLines.writeGPIOsDirection(0xFFFF);
	return static_cast<uint16_t>(dataLines.readGPIOs());
}

void
setDataBits(uint16_t value) noexcept {
	dataLines.writeGPIOsDirection(0);
	dataLines.writeGPIOs(value);
}


// layout of the extra memory commit expander
// PA0 - BurstAddress1 - input
// PA1 - BurstAddress2 - input
// PA2 - BurstAddress3 - input
// PA3 - BE0_ - input
// PA4 - BE1_ - input
// PA5 - HOLD  - output
// PA6 - HLDA  - input 
// PA7 - _LOCK - output
// PB0-PB7 - Unused

enum class ExtraGPIOExpanderPinout : decltype(A0) {
	BurstAddress1,
	BurstAddress2,
	BurstAddress3,
	ByteEnable0,
	ByteEnable1,
	HOLD,
	HLDA,
	LOCK_,
	// add support for upto 256 spi devices
	Unused0,
	Unused1,
	Unused2,
	Unused3,
	Unused4,
	Unused5,
	Unused6,
	Unused7,
	Count,
};
static_assert(static_cast<int>(ExtraGPIOExpanderPinout::Count) == 16);
enum class LoadStoreStyle : uint8_t {
    // based off of BE0,BE1 pins
    Full16 = 0b00,
    Upper8 = 0b01,
    Lower8 = 0b10,
    None = 0b11,
};
// The upper eight lines of the Extra GPIO expander is used as an SPI address selector
// This provides up to 256 devices to be present on the SPI address bus and controlable through the single SPI_BUS_EN pin from the 1284p
// The "SPI Address" is used to select which CS line to pull low when ~SPI_BUS_EN is pulled low by the MCU. It requires some extra logic to
// use but is stupid simple to layout physically. These IDs have _no_ direct correlation to the memory map exposed to the i960 itself.
// This is only used internally. The idea is to use the SDCard to define the memory mapping bound to a given SPIBusDevice.

// Physically, the 8-bit address is broken up into a 2:3:3 design as follows:
// [0b00'000'000, 0b00'111'111] : "Onboard" Devices
// [0b01'000'000, 0b01'111'111] : "More Onboard/Optional" Devices
// [0b10'000'000, 0b10'111'111] : "Even More Onboard/Optional" Devices
// [0b11'000'000, 0b11'111'111] : User Devices [Direct access from the i960]
enum class SPIBusDevice : uint16_t {
    // Unused
    Unused0,
    Unused1,
    Unused2,
    Unused3,
    Unused4,
    Unused5,
    Unused6,
    Unused7,
    // WINBOND 4Megabit Flash
    Flash0,
    Flash1 ,
    Flash2,
    Flash3,
    Flash4,
    Flash5,
    Flash6,
    Flash7,
    // 23LC1024 SRAM Chips (128Kbyte)
    SRAM0,
    SRAM1,
    SRAM2,
    SRAM3,
    SRAM4,
    SRAM5,
    SRAM6,
    SRAM7,
    //
    SRAM8,
    SRAM9,
    SRAM10,
    SRAM11,
    SRAM12,
    SRAM13,
    SRAM14,
    SRAM15,
    // ESPRESSIF PSRAM64H (8Megabyte/64-megabit)
    PSRAM0,
    PSRAM1,
    PSRAM2,
    PSRAM3,
    PSRAM4,
    PSRAM5,
    PSRAM6,
    PSRAM7,
    //
    PSRAM8,
    PSRAM9,
    PSRAM10,
    PSRAM11,
    PSRAM12,
    PSRAM13,
    PSRAM14,
    PSRAM15,
    //
    PSRAM16,
    PSRAM17,
    PSRAM18,
    PSRAM19,
    PSRAM20,
    PSRAM21,
    PSRAM22,
    PSRAM23,
    // Misc Device Block Set 7
    TFT_DISPLAY,
    SD_CARD0,
    Airlift,
    BluetoothLEFriend,
    GPIO128_0, // 128-gpios via Eight MCP23S17s connected to a single SPI Enable Line
    ADC0, // A single MCP3008/MCP3208 Analog to Digital Converter
    DAC0, // A single ??? Digital to Analog converter
    // Misc Device Block Set 7

    Count
};
static_assert (static_cast<int>(SPIBusDevice::Count) <= 256);
/**
 * @brief Describes an arbitrary device of an arbitrary size that is mapped into memory at a given location
 */
class Device {
public:
    constexpr Device(uint32_t start, uint32_t len) noexcept : startAddress_(start), length_(len) {}
    virtual ~Device() = default;
    constexpr auto getStartAddress() const noexcept { return startAddress_; }
    constexpr auto getLength() const noexcept { return length_; }
    constexpr auto getEndAddress() const noexcept { return length_ + startAddress_; }
    constexpr bool mapsToGivenRange(uint32_t address) const noexcept {
        return (address >= startAddress_) && (getEndAddress() > address);
    }
    virtual uint16_t read(uint32_t address, LoadStoreStyle style) noexcept = 0;
    virtual void write(uint32_t address, uint16_t value, LoadStoreStyle style) noexcept = 0;
private:
    uint32_t startAddress_ = 0;
    uint32_t length_ = 0;
};
/**
 * @brief Describes a block of RAM of an arbitrary size that is mapped into memory at a given location
 */
class RAM : public Device{
public:
    constexpr RAM(uint32_t start, uint32_t ramSize) : Device(start, ramSize) { }
    ~RAM() override = default;
    uint16_t read(uint32_t address, LoadStoreStyle style) noexcept override {
        switch (style) {
            case LoadStoreStyle::Full16:
                return read16(address);
            case LoadStoreStyle::Lower8:
                return read8(address);
            case LoadStoreStyle::Upper8:
                return read8(address+1);
            default:
                return 0;
        }
    }
    void write (uint32_t address, uint16_t value, LoadStoreStyle style) noexcept override {
        switch (style) {
            case LoadStoreStyle::Full16:
                write16(address, value);
                break;
            case LoadStoreStyle::Lower8:
                write8(address, static_cast<uint8_t>(value));
                break;
            case LoadStoreStyle::Upper8:
                write8(address + 1, static_cast<uint8_t>(value >> 8));
                break;
            default:
                // do nothing
                break;
        }
    }
protected:
    virtual uint8_t read8(uint32_t address) = 0;
    virtual uint16_t read16(uint32_t address) = 0;
    virtual void write8(uint32_t address, uint8_t value) = 0;
    virtual void write16(uint32_t address, uint16_t value) = 0;
};

volatile SPIBusDevice busId = SPIBusDevice::Unused0;
void setSPIBusId(SPIBusDevice id) noexcept {
    static bool initialized = false;
    bool setMemoryId = false;
    if (!initialized) {
        initialized = true;
        setMemoryId = true;
    } else {
       setMemoryId = (id != busId);
    }
    if (setMemoryId) {
        busId = id;
        extraMemoryCommit.writePortB(static_cast<uint8_t>(busId));
    }
}

uint8_t getByteEnableBits() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b11000) >> 3;
}

auto getByteEnable0() noexcept {
	return (getByteEnableBits() & 1) == 0 ? LOW : HIGH;
}
auto getByteEnable1() noexcept {
	return (getByteEnableBits() & 0b10) == 0 ? LOW : HIGH;
}

uint8_t getBurstAddressBits() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b111) << 1;
}

constexpr Address getBurstAddress(Address base, Address burstBits) noexcept {
	return (base & (~0b1110)) | burstBits;
}
Address getBurstAddress(Address base) noexcept {
	return getBurstAddress(base, static_cast<Address>(getBurstAddressBits()));
}
Address getBurstAddress() noexcept {
	return getBurstAddress(getAddress());
}

bool isReadOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isAsserted(); }
bool isWriteOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isDeasserted(); }
auto getBlastPin() noexcept { return DigitalPin<i960Pinout::BLAST_>::read(); }
void setHOLDPin(decltype(LOW) value) noexcept {
	digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), value, extraMemoryCommit);
}
void setLOCKPin(decltype(LOW) value) noexcept {
	digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), value, extraMemoryCommit);
}

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
void startingSystemTest() noexcept;
void systemTestPassed() noexcept;
void startupState() noexcept;
void systemTestState() noexcept;
void idleState() noexcept;
void doAddressState() noexcept;
void processDataRequest() noexcept;
void doRecoveryState() noexcept;
void enteringDataState() noexcept;
void enteringIdleState() noexcept;
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
State tChecksumFailure(nullptr, nullptr, nullptr);


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
/**
 * @brief Represents a single Espressif PSRAM64H device
 */
class PSRAM64H : public RAM {
public:
    enum class Opcodes : uint8_t {
        Read = 0x03,
        FastRead = 0x0B,
        FastReadQuad = 0xEB,
        Write = 0x02,
        QuadWrite = 0x38,
        EnterQuadMode = 0x35,
        ExitQuadMode = 0xF5,
        ResetEnable = 0x66,
        Reset = 0x99,
        SetBurstLength = 0xC0,
        ReadID = 0x9F,
    };
    static constexpr uint32_t Size = 8 * static_cast<uint32_t>(1024) * static_cast<uint32_t>(1024);
public:
    constexpr PSRAM64H(uint32_t startAddress, SPIBusDevice id) : RAM(startAddress, Size), busId_(id) { }
    ~PSRAM64H() override = default;
    constexpr auto getBusID() const noexcept { return busId_; }
protected:
    uint8_t read8(uint32_t address) override;
    uint16_t read16(uint32_t address) override;
    void write8(uint32_t address, uint8_t value) override;
    void write16(uint32_t address, uint16_t value) override;
private:
    SPIBusDevice busId_;
};

uint8_t
PSRAM64H::read8(uint32_t address) {
    setSPIBusId(busId_);
    byte a = static_cast<byte>(address >> 16);
    byte b = static_cast<byte>(address >> 8);
    byte c = static_cast<byte>(address);
    HoldPinLow<i960Pinout::SPI_BUS_EN> transaction;
    SPI.transfer(static_cast<byte>(Opcodes::Read));
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    return SPI.transfer(0x00);
}

uint16_t
PSRAM64H::read16(uint32_t address) {
    setSPIBusId(busId_);
    byte a = static_cast<byte>(address >> 16);
    byte b = static_cast<byte>(address >> 8);
    byte c = static_cast<byte>(address);
    HoldPinLow<i960Pinout::SPI_BUS_EN> transaction;
    SPI.transfer(static_cast<byte>(Opcodes::Read));
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    return SPI.transfer16(0x00);
}

void
PSRAM64H::write8(uint32_t address, uint8_t value) {
    setSPIBusId(busId_);
    byte a = static_cast<byte>(address >> 16);
    byte b = static_cast<byte>(address >> 8);
    byte c = static_cast<byte>(address);
    HoldPinLow<i960Pinout::SPI_BUS_EN> transaction;
    SPI.transfer(static_cast<byte>(Opcodes::Write));
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    SPI.transfer(value);
}
void
PSRAM64H::write16(uint32_t address, uint16_t value) {
    setSPIBusId(busId_);
    byte a = static_cast<byte>(address >> 16);
    byte b = static_cast<byte>(address >> 8);
    byte c = static_cast<byte>(address);
    HoldPinLow<i960Pinout::SPI_BUS_EN> transaction;
    SPI.transfer(static_cast<byte>(Opcodes::Write));
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    SPI.transfer16(value);
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
performWrite(Address address, uint16_t value) noexcept {
    Serial.print(F("Write 0x"));
    Serial.print(value, HEX);
    Serial.print(F(" to 0x"));
    Serial.println(address, HEX);
    /// @todo implement
}
uint16_t
performRead(Address address) noexcept {
    Serial.print(F("Read from 0x"));
    Serial.println(address, HEX);
    /// @todo implement
    return 0;
}
void processDataRequest() noexcept {
    auto burstAddress = getBurstAddress(baseAddress);
	if (performingRead) {
		setDataBits(performRead(burstAddress));
	} else {
	    performWrite(burstAddress, getDataBits());
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
    Serial.println(F("Setup io expanders!"));
}
void transferAddress(uint32_t address) {
    SPI.transfer(static_cast<uint8_t>(address >> 16));
    SPI.transfer(static_cast<uint8_t>(address >> 8));
    SPI.transfer(static_cast<uint8_t>(address));
}
// the setup routine runs once when you press reset:
void setup() {
    Serial.begin(115200);
    Serial.println(F("i960Sx chipset bringup"));
    setupPins(OUTPUT,
              i960Pinout::Reset960,
              i960Pinout::SPI_BUS_EN);
	digitalWrite(i960Pinout::SPI_BUS_EN, HIGH);
    pinMode(static_cast<int>(i960Pinout::Led), OUTPUT);
    t.oscillate(static_cast<int>(i960Pinout::Led), 1000, HIGH);
    SPI.begin();
	PinAsserter<i960Pinout::Reset960> holdi960InReset;
	setupIOExpanders();
	setupCPUInterface();
	setupBusStateMachine();
	#if 0
	setSPIBusId(SPIBusDevice::TFTDisplay);
	tft.begin();
	tft.fillScreen(ILI9341_BLACK);
	tft.setCursor(0,0);
	tft.setTextColor(ILI9341_WHITE);
	tft.setTextSize(3);
	tft.println("i960Sx!");
    #endif
	delay(1000);
	// we want to jump into the code as soon as possible after this point
}
void loop() {
	fsm.run_machine();
    t.update();
}
