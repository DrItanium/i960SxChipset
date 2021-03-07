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
#include <Timer.h>
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
enum class i960Pinout : decltype(A0) {
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
	AVR_INT0,	  // AVR Interrupt INT0
	AVR_INT1, 		// AVR Interrupt INT1
	MEMACK_,	  // input
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
	DEN_, 	     // input
// PORT A
	STATE_IDLE_, 	// output
	STATE_ADDR_, 	// output
	STATE_DATA_, 	// output
	STATE_WAIT_, 	// output
	STATE_RECOVER_, // output
	STATE_FAIL_,    // output
	SRAM_EN_,		// output
	FAIL,			// input, PCINT7
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
DefOutputPin(i960Pinout::SRAM_EN_, LOW, HIGH);
DefOutputPin(i960Pinout::STATE_DATA_, LOW, HIGH);
DefOutputPin(i960Pinout::STATE_WAIT_, LOW, HIGH);
DefOutputPin(i960Pinout::STATE_IDLE_, LOW, HIGH);
DefOutputPin(i960Pinout::STATE_ADDR_, LOW, HIGH);
DefOutputPin(i960Pinout::STATE_RECOVER_, LOW, HIGH);
DefOutputPin(i960Pinout::STATE_FAIL_, LOW, HIGH);
DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::AS_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
DefInputPin(i960Pinout::MEMACK_, LOW, HIGH);
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

constexpr auto MemoryRequestLocation = 0x00'0000;
constexpr auto MemoryResultLocation = 0x00'0100;
constexpr auto OpcodeWrite = 0b0000'0010;
constexpr auto OpcodeRead = 0b0000'0011;

uint16_t transfer16Bits(uint16_t value) noexcept {
	return SPI.transfer16(value);
}

void transfer32Bits(uint32_t value) noexcept {
	SPI.transfer16(value >> 16);
	SPI.transfer16(value);
}
void transferAddress(Address address) noexcept {
	SPI.transfer(static_cast<uint8_t>(address >> 16));
	SPI.transfer(static_cast<uint8_t>(address >> 8));
	SPI.transfer(static_cast<uint8_t>(address));
}
void writeMemoryRequest(Address address, uint16_t value) noexcept {
	uint16_t operation = isWriteOperation() ? 0b100 : 0b000;
	operation |= getByteEnableBits();
	setIsolatedSPIBusId(0); // SRAM
	Serial.print(" Opcode: 0x"); Serial.println(OpcodeWrite, HEX);
	Serial.print(" Location: 0x"); Serial.println(MemoryRequestLocation, HEX);
	Serial.print(" Operation: 0x"); Serial.println(operation, HEX);
	Serial.print(" Address: 0x"); Serial.println(address, HEX);
	Serial.print(" Value: 0x"); Serial.println(value, HEX);
	{
		PinAsserter<i960Pinout::SRAM_EN_> holder;
		SPI.transfer(OpcodeWrite);
		// we save to address zero in the sram
		transferAddress(MemoryRequestLocation);
		transfer16Bits(operation);
		transfer32Bits(address);
		transfer16Bits(value);
	}
	Serial.println("----");
	{
		PinAsserter<i960Pinout::SRAM_EN_> holder;
		SPI.transfer(OpcodeRead);
		// we save to address zero in the sram
		transferAddress(MemoryRequestLocation);
		Serial.println(transfer16Bits(0), HEX);
		Serial.println(transfer16Bits(0), HEX);
		Serial.println(transfer16Bits(0), HEX);
		Serial.println(transfer16Bits(0), HEX);
	}


}

uint16_t readMemoryResult() noexcept {
	setIsolatedSPIBusId(0); // SRAM
	PinAsserter<i960Pinout::SRAM_EN_> holder;
	SPI.transfer(OpcodeRead);
	transferAddress(MemoryResultLocation);
	return transfer16Bits(0);
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
// Td -> Tw if not ready 
// Tw -> Td if ready and burst (blast high)
// Tw -> Tr after signaling ready and no burst (blast low)

// Ti -> TChecksumFailure if FAIL is asserted
// Tr -> TChecksumFailure if FAIL is asserted

// NOTE: Tw may turn out to be synthetic
volatile bool asTriggered = false;
volatile uint32_t baseAddress = 0;
volatile bool performingRead = false;
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
State tSystemTest(DigitalPin<i960Pinout::STATE_FAIL_>::assert, systemTestState, DigitalPin<i960Pinout::STATE_FAIL_>::deassert);
Fsm fsm(&tStart);
State tIdle(DigitalPin<i960Pinout::STATE_IDLE_>::assert, 
		idleState, 
		DigitalPin<i960Pinout::STATE_IDLE_>::deassert);
State tAddr([]() {
			asTriggered = false;
			DigitalPin<i960Pinout::STATE_ADDR_>::assert();
		}, 
		doAddressState, 
		DigitalPin<i960Pinout::STATE_ADDR_>::deassert);
State tData(enteringDataState, 
		processDataRequest, 
		DigitalPin<i960Pinout::STATE_DATA_>::deassert);
State tRecovery(DigitalPin<i960Pinout::STATE_RECOVER_>::assert,
		doRecoveryState,
		DigitalPin<i960Pinout::STATE_RECOVER_>::deassert);
State tWait(DigitalPin<i960Pinout::STATE_WAIT_>::assert,
		[]() {
			Serial.println("WAIT");
			if (DigitalPin<i960Pinout::MEMACK_>::isAsserted()) {
				fsm.trigger(ToSignalReadyState);
			}
		},
		DigitalPin<i960Pinout::STATE_WAIT_>::deassert);
State tRdy(nullptr, []() {
			Serial.println("RDY");
			if (auto result = readMemoryResult(); performingRead) {
				Serial.print("Result value: 0x");
				Serial.println(result, HEX);
				setDataBits(result);
			} 
			auto blastPin = getBlastPin();
			DigitalPin<i960Pinout::Ready>::pulse();
			if (blastPin == LOW) {
				// we not in burst mode
				fsm.trigger(ReadyAndNoBurst);
			}  else {
				// we are in burst mode so move back to data state
				fsm.trigger(ToDataState);
			}
		}, nullptr);
State tChecksumFailure(DigitalPin<i960Pinout::STATE_FAIL_>::assert, nullptr, nullptr);


void startupState() noexcept {
	Serial.println("Startup");
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(PerformSelfTest);
	}
}
void systemTestState() noexcept {
	Serial.println("System Test");
	if (DigitalPin<i960Pinout::FAIL>::isDeasserted()) {
		fsm.trigger(SelfTestComplete);
	}
}

ISR (INT2_vect)
{
	asTriggered = true;
	// this is the AS_ pin doing its thing
}



void idleState() noexcept {
	Serial.println("IDLE");
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(ChecksumFailure);
	} else {
		if (asTriggered) {
			fsm.trigger(NewRequest);
		}
	}
}
void doAddressState() noexcept {
	Serial.println("ADDRESS");
	if (DigitalPin<i960Pinout::DEN_>::isAsserted()) {
		fsm.trigger(ToDataState);
	}
}


void
enteringDataState() noexcept {
	DigitalPin<i960Pinout::STATE_DATA_>::assert();
	// when we do the transition, record the information we need
	baseAddress = getAddress();
	performingRead = isReadOperation();
}


void processDataRequest() noexcept {
	Serial.println("DATA");
	auto usedAddress = getBurstAddress(baseAddress);
	writeMemoryRequest(usedAddress, performingRead ? 0 : getDataBits());
	fsm.trigger(ToSignalWaitState);
	// at the end of the day, signal ready on the request
	// get the base address 
}

void doRecoveryState() noexcept {
	Serial.println("RECOVERY");
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
	fsm.add_transition(&tData, &tWait, ToSignalWaitState, nullptr);
	fsm.add_transition(&tWait, &tRdy, ToSignalReadyState, nullptr);
	fsm.add_transition(&tRdy, &tRecovery, ReadyAndNoBurst, nullptr);
	fsm.add_transition(&tRdy, &tData, ToDataState, nullptr);
	fsm.add_transition(&tRecovery, &tAddr, RequestPending, nullptr);
	fsm.add_transition(&tRecovery, &tIdle, NoRequest, nullptr);
	fsm.add_transition(&tRecovery, &tChecksumFailure, ChecksumFailure, nullptr);
}
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
			i960Pinout::FAIL,
			i960Pinout::MEMACK_);
	EIMSK |= 0b100; // enable INT2 pin
	EICRA |= 0b100000; // trigger on falling edge
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

// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(115200);
	Serial.println("80960Sx Chipset Starting up...");
	setupPins(OUTPUT, 
			i960Pinout::Reset960,
			i960Pinout::Led,
			i960Pinout::STATE_IDLE_,
			i960Pinout::STATE_ADDR_,
			i960Pinout::STATE_DATA_,
			i960Pinout::STATE_WAIT_,
			i960Pinout::STATE_RECOVER_,
			i960Pinout::STATE_FAIL_,
			i960Pinout::SRAM_EN_);
	digitalWriteBlock(HIGH,
			i960Pinout::STATE_ADDR_,
			i960Pinout::STATE_DATA_,
			i960Pinout::STATE_WAIT_,
			i960Pinout::STATE_RECOVER_,
			i960Pinout::STATE_FAIL_,
			i960Pinout::SRAM_EN_);
	DigitalPin<i960Pinout::STATE_IDLE_>::assert();
	t.oscillate(static_cast<int>(i960Pinout::Led), 1000, HIGH);
	PinAsserter<i960Pinout::Reset960> holdi960InReset;
	SPI.begin();

	setupIOExpanders();
	setupCPUInterface();
	setupBusStateMachine();

	Serial.println("Finished starting up!");
	Serial.println("Holding reset line for a second to make sure!");
	delay(1000);
	// we want to jump into the code as soon as possible after this point
}
void loop() {
	fsm.run_machine();
	// update the fail state pin
	DigitalPin<i960Pinout::STATE_FAIL_>::write(DigitalPin<i960Pinout::FAIL>::isDeasserted());
	t.update();
}
