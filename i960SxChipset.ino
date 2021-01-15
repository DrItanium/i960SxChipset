/*
   Blink
   Turns on an LED on for one second, then off for one second, repeatedly.

   This example code is in the public domain.
 */
#include <SPI.h>
#include <Wire.h>
// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
constexpr auto led = 0 ; // technically unused
constexpr auto reset960 = 1;
constexpr auto resetGPIO = 2;
constexpr auto readyPin = 3;
constexpr auto gpioSelect = 4; // data capture spi
constexpr auto mosi = 5;       // data capture spi
constexpr auto miso = 6;	   // spi
constexpr auto sck = 7;		   // spi
constexpr auto tx0 = 8;  // reserved for serial
constexpr auto rx0 = 9;  // reserved for serial
constexpr auto ale = 10; // input
constexpr auto _as = 11; // input
constexpr auto _lock = 12; // bidirectional so make it output
constexpr auto _blast = 13; // input
constexpr auto dt_r = 14; // input
constexpr auto _den = 15; // input
constexpr auto scl = 16; // reserved
constexpr auto sda = 17; // reserved
constexpr auto w_r = 18; // input
constexpr auto hold = 19; // output
constexpr auto hlda = 20; // input
constexpr auto _int3 = 21; // output
constexpr auto int2 = 22; // output
constexpr auto int1 = 23; // output
constexpr auto _int0 = 24; // output
constexpr auto _be0 = 25; // input
constexpr auto _be1 = 26; // input
constexpr auto _a1 = 27; // input
constexpr auto _a2 = 28; // input
constexpr auto _a3 = 29; // input
constexpr auto unused0 = 30; // unused
constexpr auto unused1 = 31; // unused


// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(9600);
	pinMode(reset960, OUTPUT);
	digitalWrite(reset960, LOW);
	pinMode(resetGPIO, OUTPUT);
	digitalWrite(resetGPIO, LOW);
	SPI.begin();
	digitalWrite(resetGPIO, HIGH);
	pinMode(led, OUTPUT);
	pinMode(readyPin, OUTPUT);
	pinMode(gpioSelect, OUTPUT);
	digitalWrite(reset960, HIGH);
}

// the loop routine runs over and over again forever:
void loop() {
	digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);               // wait for a second
	digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
	delay(1000);               // wait for a second
}
