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
#include <Wire.h>
#include <libbonuspin.h>
#include <Timer.h>
#include <SD.h>
#include <Arduino_JSON.h>
#include <Adafruit_GFX.h>
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
constexpr auto onBoardCacheSize = 16 / sizeof(Short);
volatile Short onBoardCache[onBoardCacheSize] = { 0 };

enum class i960Pinout : decltype(A0) {
// PORT B
	Led = 0, 	  // output
   	CLOCK_OUT, // output, unusable
	DEN_,     // input, AVR Int2
	ResetGPIO,	      // output 
	GPIOSelect, // output
	MOSI,		  // reserved
	MISO,		  // reserved
	SCK, 		  // reserved
// PORT D
	RX0, 		  // reserved
	TX0, 		  // reserved
	AVR_INT0,	  // AVR Interrupt INT0
	AVR_INT1,	  // AVR Interrupt INT1
	PWM0,		  // unused
	PWM1, 		  // unused
	PWM2, 		  // unused
	PWM3, 		  // unused
// PORT C
	SCL,		  // reserved
	SDA, 		  // reserved
	Ready, 	  // output
	Int0_,		  // output
	Hold,         // input
	HLDA,         // input
	W_R_, 		  // input
	Reset960,		  // output
// PORT A
	BLAST_, 		  // input
	Lock_, 		  // output (actually bidirectional)
	Analog2, 	  // unused
	Analog3, 	  // unused
	Analog4,	  // unused
	Analog5,	  // unused
	Analog6,	  // unused
	Analog7,	  // unused
	Count,		   // special
};
static_assert(static_cast<decltype(HIGH)>(i960Pinout::Count) <= 32);

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


template<IOExpanderAddress addr>
using IOExpander = bonuspin::MCP23S17<static_cast<int>(addr),
	  static_cast<int>(i960Pinout::GPIOSelect),
	  static_cast<int>(i960Pinout::ResetGPIO)>;

inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
	digitalWrite(static_cast<int>(ip), value);
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
	pinMode(static_cast<int>(ip), value);
}

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

inline auto digitalRead(i960Pinout ip) {
	return digitalRead(static_cast<int>(ip));
}
template<i960Pinout pinId>
using HoldPinLow = PinToggler<pinId, LOW, HIGH>;

template<i960Pinout pinId>
using HoldPinHigh = PinToggler<pinId, HIGH, LOW>;

// 8 IOExpanders to a single enable line for SPI purposes 
// 3 of them are reserved
IOExpander<IOExpanderAddress::DataLines> dataLines;
IOExpander<IOExpanderAddress::Lower16Lines> lower16;
IOExpander<IOExpanderAddress::Upper16Lines> upper16;
IOExpander<IOExpanderAddress::MemoryCommitExtras> extraMemoryCommit;

// upper five are actually unused but could be through these devices
// this would give us 80 pins to work with for other purposes
IOExpander<IOExpanderAddress::OtherDevice0> dev0;
IOExpander<IOExpanderAddress::OtherDevice1> dev1;
IOExpander<IOExpanderAddress::OtherDevice2> dev2;
IOExpander<IOExpanderAddress::OtherDevice3> dev3;

Timer t;
Address
getAddress() noexcept {
	auto lower16Addr = static_cast<Address>(lower16.readGPIOs());
	auto upper16Addr = static_cast<Address>(upper16.readGPIOs()) << 16;
	return lower16Addr | upper16Addr;
}
uint16_t
getDataBits() noexcept {
	dataLines.writeGPIOsDirection(0);
	return static_cast<uint16_t>(dataLines.readGPIOs());
}

void
setDataBits(uint16_t value) noexcept {
	dataLines.writeGPIOsDirection(0xFFFF);
	dataLines.writeGPIOs(value);
}


// layout of the extra memory commit expander
// PA0 - BurstAddress1 - input
// PA1 - BurstAddress2 - input
// PA2 - BurstAddress3 - input
// PA3 - BE0_ - input
// PA4 - BE1_ - input
// PA5 - AS_ - input
// PA6 - ALE - input
// PA7 - DT\R_ - input 
// PB0 - Unused - input
// PB1 - Unused
// PB2 - Unused
// PB3 - Unused
// PB4 - Unused
// PB5 - Unused
// PB6 - Unused
// PB7 - Unused

uint8_t getByteEnableBits() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b11000) >> 3;
}

bool getByteEnable0() noexcept {
	return getByteEnableBits() & 1;
}
bool getByteEnable1() noexcept {
	return getByteEnableBits() & 0b10;
}

uint8_t getBurstAddressBits() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b111) << 1;
}

Address getBurstAddress() noexcept {
	return (getAddress() & (~0b1110)) | static_cast<Address>(getBurstAddressBits());
}

bool inAddressState() noexcept {
	return !static_cast<bool>((extraMemoryCommit.readGPIOs() >> 5) & 1);
}

bool addressLatchEnabled() noexcept {
	return static_cast<bool>((extraMemoryCommit.readGPIOs() >> 6) & 1);
}

bool shouldPerformDataTransmit() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b1000'0000);
}

bool shouldPerformDataReceive() noexcept {
	return !shouldPerformDataTransmit();
}

bool isReadOperation() noexcept {
	return digitalRead(i960Pinout::W_R_) == LOW;
}
bool isWriteOperation() noexcept {
	return digitalRead(i960Pinout::W_R_) == HIGH;
}
bool getBlastPin() noexcept {
	return digitalRead(i960Pinout::BLAST_) == HIGH;
}
bool isLastBurstTransaction() noexcept {
	return !getBlastPin();
}

void signalReady() noexcept {
	HoldPinLow<i960Pinout::Ready> holdItLow;
	delayMicroseconds(500);
}

/**
 * FAIL Circuit as defined in the i960 docs but in software instead of
 * hardware.
 */
constexpr bool failureOnBootup(bool be0, bool be1, bool blast) noexcept {
	return (!blast) && (be0 && be1);
}

bool failureOnBootup() noexcept {
	return failureOnBootup(getByteEnable0(), getByteEnable1(), getBlastPin());
}

/// @todo add the FAIL pin based off of the diagrams I have (requires external
// circuitry.
uint16_t load(Address address) noexcept {
	return 0;
}

void store(Address address, uint16_t value) noexcept {

}
ISR (INT2_vect) 
{
	// this is the DEN_ pin doing its thing
}
void setupCPUInterface() {
	Serial.print("Setting up cpu interface pins...");
	setupPins(OUTPUT,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Lock_,
			i960Pinout::Int0_,
			i960Pinout::Hold);
	digitalWriteBlock(HIGH,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Lock_,
			i960Pinout::Int0_);
	digitalWrite(i960Pinout::Hold, LOW);
	setupPins(INPUT,
			i960Pinout::BLAST_,
			i960Pinout::DEN_,
			i960Pinout::W_R_,
			i960Pinout::HLDA);
	EIMSK |= 0b100; // enable INT2 pin
	EICRA |= 0b100000; // trigger on falling edge
	Serial.println("Done!");
	Serial.print("Setting up on-board cache...");
	for (int i = 0; i < onBoardCacheSize; ++i) {
		onBoardCache[i] = 0;
	}
	Serial.println("Done!");
}
void setupIOExpanders() {
	dataLines.begin();
	lower16.begin();
	upper16.begin();
	extraMemoryCommit.begin();
	dataLines.reset();
	lower16.reset();
	upper16.reset();
	extraMemoryCommit.reset();
	lower16.writeGPIOsDirection(0);
	upper16.writeGPIOsDirection(0);
	extraMemoryCommit.writeGPIOsDirection(0);
}

void emitCharState(bool condition, char onTrue, char onFalse) noexcept {
	Serial.print(condition ? onTrue : onFalse);
}
	
// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(115200);
	Serial.println("80960Sx Chipset Starting up...");
	setupPins(OUTPUT, 
			i960Pinout::ResetGPIO, 
			i960Pinout::Reset960,
			i960Pinout::Led);
	HoldPinLow<i960Pinout::Reset960> holdi960InReset;
	HoldPinLow<i960Pinout::ResetGPIO> gpioReset;
	t.oscillate(static_cast<int>(i960Pinout::Led), 1000, HIGH);
	setupCPUInterface();
	SPI.begin();
	setupIOExpanders();
	/// wait two seconds to ensure that reset is successful
	delay(2000);
	sei();
	// At this point the cpu will have started up and we must check out the
	// fail circuit during bootup.
}

void processingLoop() {
	auto inAddrState = inAddressState();
	auto inDatState = !inAddrState;
	emitCharState(inAddressState, 'A', 'D');
	auto failed = failureOnBootup();
	emitCharState(failed, 'F', 'P');
	Serial.print(getBurstAddressBits(), BIN);
	emitCharState(getByteEnable0(), '1', '0');
	emitCharState(getByteEnable1(), '1', '0');
	if (!failed && inDatState) {
		auto isWriting = isWriteOperation();
		auto isReading = !isWriting;
		emitCharState(isWriting, 'W', 'R');
		auto address = getAddress();
		Serial.print(" 0x");
		Serial.print(address, HEX);
		if (isReading) {
			setDataBits(load(address));
		} else /* is writing */ {
			store(address, getDataBits());
		}
		signalReady();
		dataLines.writeGPIOsDirection(0xFFFF);
	}
	Serial.println();
	delay(1000);
}
// the loop routine runs over and over again forever:
void loop() {
	processingLoop();
	//digitalWrite(i960Pinout::Led, HIGH);   // turn the LED on (HIGH is the voltage level)
	//delay(1000);               // wait for a second
	//digitalWrite(i960Pinout::Led, LOW);    // turn the LED off by making the voltage LOW
	//delay(1000);               // wait for a second
	t.update();
}
