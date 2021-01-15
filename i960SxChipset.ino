/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */
#include <SPI.h>
// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
constexpr auto led = 0 ;
constexpr auto reset960 = 1;
constexpr auto resetGPIO = 2;
constexpr auto unused0 = 3;
constexpr auto gpioSelect = 4;
// the setup routine runs once when you press reset:
void setup() {
  pinMode(reset960, OUTPUT);
  digitalWrite(reset960, LOW);
  pinMode(resetGPIO, OUTPUT);
  digitalWrite(resetGPIO, LOW);
  SPI.begin();
  digitalWrite(resetGPIO, HIGH);
  pinMode(gpioSelect, OUTPUT);
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  digitalWrite(reset960, HIGH);
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
}
