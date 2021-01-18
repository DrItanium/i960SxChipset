/*
   Blink
   Turns on an LED on for one second, then off for one second, repeatedly.

   This example code is in the public domain.
 */
#include <SPI.h>
#include <Wire.h>
#include <libbonuspin.h>
#include <SD.h>
// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
enum class i960Pinout : decltype(A0) {
// PORT B
	Led = 0, 	  // output
	Reset960,     // output
	Unused2, 	  // AVR Interrupt  INT2
	Ready,		  // output
	GPIOSelect,   // output
	MOSI,		  // reserved
	MISO,		  // reserved
	SCK, 		  // reserved
// PORT D
	RX0, 		  // reserved
	TX0, 		  // reserved
	Unused3,	  // AVR Interrupt INT0
	Unused4,	  // AVR Interrupt INT1
	Lock_,		  // bidirectional but default to output
	Int0_,	      // output 
	DT_R, 		  // input
	DEN_, 		  // input
// PORT C
	SCL,		  // reserved
	SDA, 		  // reserved
	W_R, 		  // input
	Hold,		  // output
	HLDA,         // input
	ALE,          // input
	ResetGPIO,    // output
	AS_, 		  // input
// PORT A
	Unused12,		  // input
	Unused6, // input
	Unused7, // input
	Unused8, // input
	Unused9, // input
	Unused10, // input
	Unused11, 	   // input
	Unused1, 	   // unused
	Count,		   // special
};
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

static_assert(static_cast<decltype(HIGH)>(i960Pinout::Count) == 32);

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

uint32_t
getAddress() noexcept {
	auto lower16Addr = static_cast<uint32_t>(lower16.readGPIOs());
	auto upper16Addr = static_cast<uint32_t>(upper16.readGPIOs()) << 16;
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

uint8_t getByteEnableBits() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b11);
}

uint8_t getBurstAddress() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b11100) >> 2;
}

bool isBurstLast() noexcept {
	return (extraMemoryCommit.readGPIOs() & 0b100000);
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
			i960Pinout::ALE,
			i960Pinout::AS_,
			i960Pinout::DT_R,
			i960Pinout::DEN_,
			i960Pinout::W_R,
			i960Pinout::HLDA);
	SPI.begin();
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
	/// wait two seconds to ensure that reset is successful
	delay(2000);
}

// the loop routine runs over and over again forever:
void loop() {
	digitalWrite(i960Pinout::Led, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);               // wait for a second
	digitalWrite(i960Pinout::Led, LOW);    // turn the LED off by making the voltage LOW
	delay(1000);               // wait for a second
}
