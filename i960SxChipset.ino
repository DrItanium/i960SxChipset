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
#include "Board.h"
#include <SPI.h>
#include <libbonuspin.h>
#ifdef ARDUINO_AVR_ATmega1284
#include <Timer.h>
#endif
#include <Fsm.h>
template<typename T>
class TreatAs final {
	public:
		using ReturnType = T;
};
using Address = uint32_t;
using Short = uint16_t;
using BusDatum = Short;
using Byte = uint8_t;
using TreatAsByte = TreatAs<uint8_t>;
using TreatAsShort = TreatAs<uint16_t>;
using TreatAsWord = TreatAs<uint32_t>;
constexpr auto using1284p() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
    return true;
#else
    return false;
#endif
}
constexpr auto usingUno() noexcept {
#ifdef ARDUINO_AVR_UNO
    return true;
#else
    return false;
#endif
}
constexpr auto getCPUFrequency() noexcept {
    return F_CPU;
}
enum class TargetBoard {
    Unknown = 0,
    MightyCore_1284p,
    ArduinoUno,
};

constexpr auto getTargetBoard() noexcept {
    if (using1284p()) {
        return TargetBoard::MightyCore_1284p;
    } else if (usingUno()) {
        return TargetBoard::ArduinoUno;
    } else {
        return TargetBoard::Unknown;
    }
}
#if 0
enum class i960Pinout : decltype(A0) {
#ifdef ARDUINO_AVR_ATmega1284
// PORT B
	Led = 0, 	  // output
   	CLOCK_OUT, // output, unusable
	AS_,     // input, AVR Int2
	PWM4, 	 // unused 
	GPIOSelect,		// output
	MOSI,		  // reserved
	MISO,		  // reserved
	SCK, 		  // reserved
// PORT D
	RX0, 		  // reserved
	TX0, 		  // reserved
	DEN_,	  // AVR Interrupt INT0
	AVR_INT1, 		// AVR Interrupt INT1
	PWM0,	  // input
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
#elif defined(ARDUINO_AVR_UNO)
    Led = LED_BUILTIN,
    SCL = A5,
    SDA = A4,
    AS_ = 2,
    DEN_ = 3,
    Ready = 4,
    Int0_ = 5,
    W_R_ = 6,
    Reset960 = 7,
    BLAST_ = 8,
    FAIL = 9,
    GPIOSelect = 10,
    MOSI = 11,
    MISO = 12,
    SCK = 13,
    SPI_BUS_EN = A0,
    Analog1 = A1,
    Analog2 = A2,
    Analog3 = A3,
    Analog4 = A4,
    Analog5 = A5,
#else
#error "Unsupported board, unknown pinout"
#endif
};
#endif

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
#if 0
Adafruit_ILI9341 tft(
		static_cast<decltype(A0)>(i960Pinout::DISP_CS_),
		static_cast<decltype(A0)>(i960Pinout::DC),
		static_cast<decltype(A0)>(i960Pinout::MOSI),
		static_cast<decltype(A0)>(i960Pinout::SCK),
		static_cast<decltype(A0)>(i960Pinout::DISP_RESET_),
		static_cast<decltype(A0)>(i960Pinout::MISO));
Adafruit_RGBLCDShield lcd;
#endif


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

#ifdef ARDUINO_AVR_ATmega1284
Timer t;
#endif

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
void setIsolatedSPIBusId(uint8_t id) noexcept {
	extraMemoryCommit.writePortB(id);
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
uint16_t makeCommandOperation() noexcept {
	if (auto enableBits = getByteEnableBits(); DigitalPin<i960Pinout::W_R_>::isAsserted()) {
		return enableBits;
	} else {
		return 0b100 | enableBits;
	}
}
void setHOLDPin(decltype(LOW) value) noexcept {
	digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), value, extraMemoryCommit);
}
void setLOCKPin(decltype(LOW) value) noexcept {
	digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), value, extraMemoryCommit);
}


/// @todo add the FAIL pin based off of the diagrams I have (requires external
// circuitry.
// We talk to the FT232H via an external SPI SRAM of 1 Megabit (128 kbytes)
// there are two addresses used in the design

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
// Td -> Tw if not ready 
// Tw -> Td if ready and burst (blast high)
// Tw -> Tr after signaling ready and no burst (blast low)

// Ti -> TChecksumFailure if FAIL is asserted
// Tr -> TChecksumFailure if FAIL is asserted

// NOTE: Tw may turn out to be synthetic
volatile bool asTriggered = false;
volatile bool denTriggered = false;
volatile uint32_t baseAddress = 0;
volatile bool performingRead = false;
uint16_t readResult = 0;
constexpr auto NoRequest = 0;
constexpr auto NewRequest = 1;
constexpr auto ReadyAndBurst = 2;
constexpr auto NotReady = 3;
constexpr auto ReadyAndNoBurst = 4;
constexpr auto RequestPending = 5;
constexpr auto ToDataState = 6;
constexpr auto ToSignalReadyState = 7;
constexpr auto ToSignalWaitState = 8;
constexpr auto PerformSelfTest = 9;
constexpr auto SelfTestComplete = 10;
constexpr auto ChecksumFailure = 11;
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
State tRdy(nullptr, []() {
#if 0
#endif
		}, nullptr);
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
#ifdef ARDUINO_AVR_ATmega1284
#define AS_ISR INT2_vect
#define DEN_ISR INT0_vect
#elif defined(ARDUINO_AVR_UNO)
#define AS_ISR INT0_vect
#define DEN_ISR INT1_vect
#else
#error "AS and DEN ISR must be defined for this unknown board"
#endif
ISR (AS_ISR)
{
	asTriggered = true;
	// this is the AS_ pin doing its thing
}
ISR (DEN_ISR)
{
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
void
performWrite(Address address, uint16_t value) noexcept {
   Serial.print("Write 0x");
   Serial.print(value, HEX);
   Serial.print(" to 0x");
   Serial.println(address, HEX);
}
uint16_t
performRead(Address address) noexcept {
    Serial.print("Read from 0x");
    Serial.println(address, HEX);
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
#if 0
void setupInterrupts() noexcept {
    if constexpr (using1284p()) {
        EIMSK |= 0b101; // enable INT2 and INT0 pin
        EICRA |= 0b100010; // trigger on falling edge
    } else if constexpr (usingUno()) {
        EIMSK |= 0b11; // enable INT1 and INT0 pin
        EICRA |= 0b1010; // trigger on falling edge
    }

}
#endif
//State tw(nullptr, nullptr, nullptr); // at this point, this will be synthetic
//as we have no concept of waiting inside of the mcu
void setupCPUInterface() {
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
	setupInterrupts();
}
void setupIOExpanders() {
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
	lower16.writeGPIOsDirection(0b11111111'11111111);
	upper16.writeGPIOsDirection(0b11111111'11111111);
	dataLines.writeGPIOsDirection(0b11111111'11111111);
	// set lower eight to inputs and upper eight to outputs
	extraMemoryCommit.writeGPIOsDirection(0b00000000'11111111);
	// then indirectly mark the outputs
	pinMode(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), OUTPUT, extraMemoryCommit);
	pinMode(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), OUTPUT, extraMemoryCommit);
}
void transferAddress(uint32_t address) {
    SPI.transfer(static_cast<uint8_t>(address >> 16));
    SPI.transfer(static_cast<uint8_t>(address >> 8));
    SPI.transfer(static_cast<uint8_t>(address));
}
enum class SRAMOpcodes : uint8_t {
    RDSR = 0x05,
    RDMR = RDSR,
    WRSR = 0x01,
    WRMR = WRSR,
    READ = 0x03,
    WRITE = 0x02,
    EDIO = 0x3B,
    EQIO = 0x38,
    RSTIO = 0xFF,
    Read = READ,
    Write = WRITE,
};
void transferOpcode(SRAMOpcodes opcode) noexcept {
    SPI.transfer(static_cast<uint8_t>(opcode));
}
template<i960Pinout enablePin>
void write8(uint32_t address, uint8_t value) noexcept {
    static_assert(DigitalPin<enablePin>::isOutputPin());
    PinAsserter<enablePin> asserter;
    transferOpcode(SRAMOpcodes::Write);
    transferAddress(address);
    SPI.transfer(value);
}
template<i960Pinout enablePin>
uint8_t read8(uint32_t address) noexcept {
    static_assert(DigitalPin<enablePin>::isOutputPin());
    PinAsserter<enablePin> asserter;
    transferOpcode(SRAMOpcodes::Read);
    transferAddress(address);
    auto result = SPI.transfer(0);
    // make sure that we do the transfer correctly
    return result;
}
template<i960Pinout enablePin>
void testMemoryBoard() {
    static_assert(DigitalPin<enablePin>::isOutputPin());
    static constexpr auto mask = 0b1111'1110'0000'0000'0000'0000;
    auto previousValue = -1;
    for (uint32_t i = 0; i < 0b0010'0000'0000'0000'0000'0000; ++i) {
        auto controlBits = static_cast<uint8_t>((mask & i) >> 17);
        auto targetDevice = static_cast<uint8_t>(controlBits & 0b0111'111);
        extraMemoryCommit.writePortB(targetDevice);
        auto value = static_cast<uint8_t>(i);
        write8<enablePin>(i, i);
        auto result = read8<enablePin>(i);
        if (result != value) {
            Serial.print("Failure, id: ") ;
            Serial.print(targetDevice, DEC);
            Serial.print(" address: 0x");
            Serial.print(i, HEX);
            Serial.print(" value: 0x");
            Serial.print(value, HEX);
            Serial.print(" got: 0x");
            Serial.println(result, HEX);
            delay(1);
        }
    }
    Serial.println("Done!");
}
// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(115200);
    setupPins(OUTPUT,
              i960Pinout::Reset960,
              i960Pinout::Led,
              i960Pinout::SPI_BUS_EN);
	digitalWrite(i960Pinout::SPI_BUS_EN, HIGH);
    boardSpecificSetup();
	PinAsserter<i960Pinout::Reset960> holdi960InReset;
	SPI.begin();
	setupIOExpanders();
	setupCPUInterface();
	setupBusStateMachine();
	#if 0
    Serial.println("Running memory boards through their paces");
    testMemoryBoard<i960Pinout::SPI_BUS_EN>();
    #endif
	delay(1000);
	// we want to jump into the code as soon as possible after this point
}
void loop() {
	fsm.run_machine();
	boardSpecificLoopBody();
}
