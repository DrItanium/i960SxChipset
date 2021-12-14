#include <SD.h>
#include <SPI.h>
/*
 * A simple grand central m4 sketch to prototype the i960 chipset interface
 */
constexpr auto PSRAMEN0 = A0;
constexpr auto PSRAMEN1 = A1;
constexpr auto PSRAMEN2 = A2;
constexpr auto PSRAMEN3 = A3;
constexpr auto GPIO_CS = 53;

void setup() {
    Serial.begin(115200);
    while(!Serial) {
        delay(100);
    }
    while (!SD.begin(SDCARD_SS_PIN)) {
        Serial.println("NO SD CARD FOUND...TRYING AGAIN IN 1 SECOND!");
        delay(1000);
    }
    SPI.begin();
    // put your setup code here, to run once:
    pinMode(PSRAMEN0, OUTPUT);
    pinMode(PSRAMEN1, OUTPUT);
    pinMode(PSRAMEN2, OUTPUT);
    pinMode(PSRAMEN3, OUTPUT);
    pinMode(GPIO_CS, OUTPUT);
    digitalWrite(PSRAMEN0, HIGH);
    digitalWrite(PSRAMEN1, HIGH);
    digitalWrite(PSRAMEN2, HIGH);
    digitalWrite(PSRAMEN3, HIGH);
    digitalWrite(GPIO_CS, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}
