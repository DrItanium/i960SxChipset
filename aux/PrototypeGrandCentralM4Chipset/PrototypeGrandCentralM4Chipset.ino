#include <SD.h>
#include <SPI.h>
/*
 * A simple grand central m4 sketch to prototype the i960 chipset interface
 */
// inputs
constexpr auto MCU_FAIL = 26;
constexpr auto BLAST = 27;
constexpr auto DEN = 28;
constexpr auto WR = 23;
constexpr auto BE0 = 37;
constexpr auto BE1 = 36;
constexpr auto LINE_INT0 = 35;
constexpr auto LINE_INT1 = 34;
constexpr auto LINE_INT2 = 33;
constexpr auto LINE_INT3 = 32;
// outputs
constexpr auto MCU_READY = 16;
constexpr auto RESET960 = 17;
constexpr auto GPIO_CS = 53;
constexpr auto PSRAMEN0 = A0;
constexpr auto PSRAMEN1 = A1;
constexpr auto PSRAMEN2 = A2;
constexpr auto PSRAMEN3 = A3;
inline void outputPin(byte value) noexcept {
    pinMode(value, OUTPUT);
}
inline void inputPin(byte value) noexcept {
    pinMode(value, INPUT);
}
void setup() {
    outputPin(RESET960);
    digitalWrite(RESET960, LOW);
    Serial.begin(115200);
    while(!Serial) {
        delay(100);
    }
    while (!SD.begin(SDCARD_SS_PIN)) {
        Serial.println("NO SD CARD FOUND...TRYING AGAIN IN 1 SECOND!");
        delay(1000);
    }
    SPI.begin();
    inputPin(MCU_FAIL);
    inputPin(BLAST);
    inputPin(DEN);
    inputPin(WR);
    inputPin(BE0);
    inputPin(BE1);
    inputPin(LINE_INT0);
    inputPin(LINE_INT1);
    inputPin(LINE_INT2);
    inputPin(LINE_INT3);
    // put your setup code here, to run once:
    outputPin(PSRAMEN0);
    outputPin(PSRAMEN1);
    outputPin(PSRAMEN2);
    outputPin(PSRAMEN3);
    outputPin(GPIO_CS);
    outputPin(MCU_READY);

    digitalWrite(PSRAMEN0, HIGH);
    digitalWrite(PSRAMEN1, HIGH);
    digitalWrite(PSRAMEN2, HIGH);
    digitalWrite(PSRAMEN3, HIGH);
    digitalWrite(GPIO_CS, HIGH);
    digitalWrite(MCU_READY, HIGH);

    digitalWrite(RESET960, HIGH);
    // i960 is active after this point
}

void loop() {
  // put your main code here, to run repeatedly:

}
