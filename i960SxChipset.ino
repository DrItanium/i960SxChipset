/*
   Blink
   Turns on an LED on for one second, then off for one second, repeatedly.

   This example code is in the public domain.
 */
#include <SPI.h>
#include <Wire.h>
#include <libbonuspin.h>
// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
enum class i960Pinout : decltype(A0) {
	Led = 0, 	  // output
	Reset960, // output
	ResetGPIO,    // output
	Ready,		  // output
	GPIOSelect,   // output
	MOSI,		  // reserved
	MISO,		  // reserved
	SCK, 		  // reserved
	TX0, 		  // reserved
	RX0, 		  // reserved
	ALE,		  // input
	AS_, 		  // input
	Lock_,		  // bidirectional but default to output
	Blast_,		  // input
	DT_R, 		  // input
	DEN_, 		  // input
	SCL,		  // reserved
	SDA, 		  // reserved
	W_R, 		  // input
	Hold,		  // output
	HLDA,         // input
	Int3_,        // output
	Int2, 		  // output
	Int1,		  // output
	Int0_,		  // output
	ByteEnable0_, // input
	ByteEnable1_, // input
	BurstAddress1, // input
	BurstAddress2, // input
	BurstAddress3, // input
	Unused0, 	   // unused
	Unused1, 	   // unused
	Count,		   // special
};
enum class IOExpanderAddress : byte {
	DataLines = 0b000,
	Lower16Lines,
	Upper16Lines,
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

IOExpander<IOExpanderAddress::DataLines> dataLines;
IOExpander<IOExpanderAddress::Lower16Lines> lower16;
IOExpander<IOExpanderAddress::Upper16Lines> upper16;

// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(9600);
	Serial.println("80960Sx Chipset Starting up...");
	setupPins(OUTPUT, 
			i960Pinout::ResetGPIO, 
			i960Pinout::Reset960);
	HoldPinLow<i960Pinout::Reset960> holdi960InReset;
	HoldPinLow<i960Pinout::ResetGPIO> gpioReset;
	setupPins(OUTPUT,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Lock_,
			i960Pinout::Int3_,
			i960Pinout::Int0_,
			i960Pinout::Hold,
			i960Pinout::Int2,
			i960Pinout::Int1);
	digitalWriteBlock(HIGH,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Lock_,
			i960Pinout::Int3_,
			i960Pinout::Int0_);
	digitalWrite(i960Pinout::Hold, LOW);
	setupPins(INPUT,
			i960Pinout::ALE,
			i960Pinout::AS_,
			i960Pinout::Blast_,
			i960Pinout::DT_R,
			i960Pinout::DEN_,
			i960Pinout::W_R,
			i960Pinout::HLDA,
			i960Pinout::ByteEnable0_,
			i960Pinout::ByteEnable1_,
			i960Pinout::BurstAddress1,
			i960Pinout::BurstAddress2,
			i960Pinout::BurstAddress3);
	SPI.begin();
	dataLines.begin();
	lower16.begin();
	upper16.begin();
	dataLines.reset();
	lower16.reset();
	upper16.reset();
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
