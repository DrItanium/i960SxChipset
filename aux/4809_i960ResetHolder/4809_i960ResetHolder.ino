// Target Chip: Atmega 4809 (DIP40)
// Target Platform: MegaCoreX
// Clockspeed: 20MHz internal oscillator
// PF6 is reset
// No bootloader
// UPDI is connected to Serial1's alternate pinout
// this sketch just holds the i960 in reset, sets up the aux pins and then releases the reset pin 2 seconds after finishing everything else.
// the serial console is totally optional and could be easily removed if needed. 
//
// The objective of this program is to eliminate pins
constexpr auto Reset960 = PIN_PF0;
constexpr auto LOCK960 = PIN_PF1;
constexpr auto HLDA960 = PIN_PF2;
constexpr auto HOLD960 = PIN_PF3;
// this pin is responsible for allowing the 
constexpr auto WAITBOOT960 = PIN_PF4;
constexpr auto INT0_960 = PIN_PE0;
constexpr auto INT1_960 = PIN_PE1;
constexpr auto INT2_960 = PIN_PE2;
constexpr auto INT3_960 = PIN_PE3;
void setup() {
 
  // setup all of the pins that can affect the i960 itself
  CCP = 0xD8;
  CLKCTRL.MCLKCTRLA |= 0x3; // enable the external 20mhz clock source just to be on the safe side
  CCP = 0xD8;
  // always pull this low to start
  pinMode(Reset960, OUTPUT);
  digitalWrite(Reset960, LOW); 
  pinMode(WAITBOOT960, INPUT_PULLUP);  
  Serial1.swap(1);
  Serial1.begin(9600);
  delay(2000); 
  // wait two seconds to give the other device time to set stuff up
  pinMode(LOCK960, OUTPUT);
  digitalWrite(LOCK960, HIGH);
  pinMode(HLDA960, INPUT);
  pinMode(HOLD960, OUTPUT);
  digitalWrite(HOLD960, LOW);
  pinMode(INT0_960, OUTPUT);
  digitalWrite(INT0_960, HIGH);
  pinMode(INT1_960, OUTPUT);
  digitalWrite(INT1_960, LOW);
  pinMode(INT2_960, OUTPUT);
  digitalWrite(INT2_960, LOW);
  pinMode(INT3_960, OUTPUT);
  digitalWrite(INT3_960, HIGH);
  Serial1.println("WAITING FOR SIGNAL CHANGE!");
  // just poll until we are let through (aka this value is high)
  while (digitalRead(WAITBOOT960) == LOW);
  Serial1.println("SIGNAL CHANGED!");
  
  delay(2000); // always wait 2 seconds at the minimum
  // rest of the logic goes here
  digitalWrite(Reset960, HIGH);
}


void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}
