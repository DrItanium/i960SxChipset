#include <SD.h>
#include <SPI.h>
constexpr unsigned long long int operator "" _KB(unsigned long long int value) noexcept { return value * 1024; }
constexpr unsigned long long int operator "" _MB(unsigned long long int value) noexcept { return value * 1024 * 1024; }
constexpr unsigned long long int operator "" _KHz(unsigned long long int value) noexcept { return value * 1000; }
constexpr unsigned long long int operator "" _MHz(unsigned long long int value) noexcept { return value * 1000 * 1000; }
static_assert(2_KHz == 2000);
static_assert(2_MHz == 2000000);
static_assert(20_MHz == 20000000);
static_assert(2_KB == 2048);
static_assert(8_KB == 8192);
/*
 * A simple grand central m4 sketch to prototype the i960 chipset interface
 */
using Address = uint32_t;
using DataValue = uint16_t;
// inputs
constexpr auto MCU_FAIL = 27;
constexpr auto BLAST = 28;
constexpr auto DEN = 23;
constexpr auto WR = 37;
constexpr auto BE0 = 36;
constexpr auto BE1 = 35;
constexpr auto LINE_INT0 = 34;
constexpr auto LINE_INT1 = 33;
constexpr auto LINE_INT2 = 32;
constexpr auto LINE_INT3 = 31;
// outputs
constexpr auto MCU_READY = 16;
constexpr auto RESET960 = 17;
constexpr auto GPIO_CS = 53;
constexpr auto PSRAMEN0 = A0;
constexpr auto PSRAMEN1 = A1;
constexpr auto PSRAMEN2 = A2;
constexpr auto PSRAMEN3 = A3;
constexpr auto RESET4809 = 5;
inline void outputPin(byte value) noexcept {
    pinMode(value, OUTPUT);
}
inline void inputPin(byte value) noexcept {
    pinMode(value, INPUT);
}
void checksumFailureHappened() noexcept {
    Serial.println("CHECKSUM FAIL!");
    while (true) {
        delay(1000);
    }
}
inline void checkForFailPin() noexcept {
    if (digitalRead(MCU_FAIL) == LOW) {
        checksumFailureHappened();
    }
}
void checksumFailInterrupt() {
    // even though this is in interrupt context, if we get here then just hang everything
    checksumFailureHappened();
}
inline void waitForDataRequest() noexcept {
    while (digitalRead(DEN) != LOW);
}
void setupConsole() noexcept {
    Serial.begin(115200);
    while(!Serial) {
        delay(100);
    }
}
void setupSDCard() noexcept {
    while (!SD.begin(SDCARD_SS_PIN)) {
        Serial.println("NO SD CARD FOUND...TRYING AGAIN IN 1 SECOND!");
        delay(1000);
    }
    Serial.println("SD CARD FOUND!");
}
struct SplitWord16 {
    constexpr explicit SplitWord16(uint16_t value = 0) noexcept : value_(value) { }
    constexpr SplitWord16(byte lower, byte upper) noexcept : bytes_{lower, upper} {}
    constexpr uint16_t getValue() const noexcept { return value_; }
    constexpr byte getUpperHalf() const noexcept { return bytes_[0]; }
    constexpr byte getLowerHalf() const noexcept { return bytes_[1]; }
    void setLowerHalf(byte value) noexcept { bytes_[0] = value; }
    void setUpperHalf(byte value) noexcept { bytes_[1] = value; }
    void setValue(uint16_t value) noexcept { value_ = value; }
private:
    union
    {
        uint16_t value_;
        byte bytes_[sizeof(uint16_t)];
    };
};
struct IOExpander final {
    IOExpander() = delete;
    ~IOExpander() = delete;
    IOExpander(IOExpander const&) = delete;
    IOExpander(IOExpander&&) = delete;
    IOExpander& operator=(IOExpander const&) = delete;
    IOExpander& operator=(IOExpander &&) = delete;
public:
    enum class MCP23x17Registers : byte {
        IODIRA = 0,
        IODIRB,
        IPOLA,
        IPOLB,
        GPINTENA,
        GPINTENB,
        DEFVALA,
        DEFVALB,
        INTCONA,
        INTCONB,
        _IOCONA,
        _IOCONB,
        GPPUA,
        GPPUB,
        INTFA,
        INTFB,
        INTCAPA,
        INTCAPB,
        GPIOA,
        GPIOB,
        OLATA,
        OLATB,
        OLAT = OLATA,
        GPIO = GPIOA,
        IOCON = _IOCONA,
        IODIR = IODIRA,
        INTCAP = INTCAPA,
        INTF = INTFA,
        GPPU = GPPUA,
        INTCON = INTCONA,
        DEFVAL = DEFVALA,
        GPINTEN = GPINTENA,
        IPOL = IPOLA,
    };
    enum class IOExpanderAddress : byte {
        DataLines = 0b0000,
        Lower16Lines = 0b0010,
        Upper16Lines = 0b0100,
        MemoryCommitExtras = 0b0110,
        OtherDevice0 = 0b1000,
        OtherDevice1 = 0b1010,
        OtherDevice2 = 0b1100,
        OtherDevice3 = 0b1110,
    };
    static constexpr byte generateReadOpcode(IOExpanderAddress address) noexcept {
        return 0b01000001 | static_cast<uint8_t>(address);
    }
    static constexpr byte generateWriteOpcode(IOExpanderAddress address) noexcept {
        return 0b01000000 | static_cast<uint8_t>(address);
    }
    static uint16_t read16(IOExpanderAddress addr, MCP23x17Registers opcode) noexcept {
        SplitWord16 ret{0};
        SPI.beginTransaction(SPISettings{10_MHz, MSBFIRST, SPI_MODE0});
        digitalWrite(GPIO_CS, LOW);
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        ret.setLowerHalf(SPI.transfer(0));
        ret.setUpperHalf(SPI.transfer(0));
        digitalWrite(GPIO_CS, HIGH);
        SPI.endTransaction();
        return ret.getValue();
    }
    static uint8_t read8(IOExpanderAddress addr, MCP23x17Registers opcode) noexcept {
        SPI.beginTransaction(SPISettings{10_MHz, MSBFIRST, SPI_MODE0});
        digitalWrite(GPIO_CS, LOW);
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        auto result = SPI.transfer(0);
        digitalWrite(GPIO_CS, HIGH);
        SPI.endTransaction();
        return result;
    }
private:
    static void write16(IOExpanderAddress address, MCP23x17Registers opcode, SplitWord16 value) noexcept {
        SPI.beginTransaction(SPISettings{10_MHz, MSBFIRST, SPI_MODE0});
        digitalWrite(GPIO_CS, LOW);
        SPI.transfer(generateWriteOpcode(address));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(value.getLowerHalf());
        SPI.transfer(value.getUpperHalf());
        digitalWrite(GPIO_CS, HIGH);
        SPI.endTransaction();
    }
public:
    static void write16(IOExpanderAddress address, MCP23x17Registers opcode, uint16_t value) noexcept {
        write16(address, opcode, SplitWord16{value});
    }
    static void write8(IOExpanderAddress address, MCP23x17Registers opcode, uint8_t value) noexcept {
        SPI.beginTransaction(SPISettings{10_MHz, MSBFIRST, SPI_MODE0});
        digitalWrite(GPIO_CS, LOW);
        SPI.transfer(generateWriteOpcode(address));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(value);
        digitalWrite(GPIO_CS, HIGH);
        SPI.endTransaction();
    }

    static uint16_t getDataLines() noexcept {
        return read16(IOExpanderAddress::DataLines, MCP23x17Registers::GPIO);
    }
    static void setDataLines(uint16_t value) noexcept {
        write16(IOExpanderAddress::DataLines, MCP23x17Registers::GPIO, value);
        // do nothing right now
    }
    static void setupDataLinesForWrite() noexcept {
        // setup the data lines for input since we are writing a value to main memory
        write16(IOExpanderAddress::DataLines, MCP23x17Registers::IODIR, 0xFFFF);
    }

    static void setupDataLinesForRead() noexcept {
        // setup the data lines for output since we are reading a value from main memory
        write16(IOExpanderAddress::DataLines, MCP23x17Registers::IODIR, 0);
    }
    static void setDirection(IOExpanderAddress addr, uint16_t directionBits) noexcept {
        write16(addr, MCP23x17Registers::IODIR, directionBits);
    }
    static void begin() noexcept {
        outputPin(GPIO_CS);
        digitalWrite(GPIO_CS, HIGH);
        SPI.beginTransaction(SPISettings{10_MHz, MSBFIRST, SPI_MODE0});
        // first, set the hardware enable bits to make sure all of the others are set correctly
        write8(IOExpanderAddress::DataLines, MCP23x17Registers::IOCON, 0x08);
        write8(IOExpanderAddress::Upper16Lines, MCP23x17Registers::IOCON, 0x08);
        write8(IOExpanderAddress::Lower16Lines, MCP23x17Registers::IOCON, 0x08);
        setDirection(IOExpanderAddress::DataLines, 0xFFFF);
        setDirection(IOExpanderAddress::Lower16Lines, 0xFFFF);
        setDirection(IOExpanderAddress::Upper16Lines, 0xFFFF);
        write16(IOExpanderAddress::DataLines, MCP23x17Registers::OLAT, 0);
        // enable address lookup bits
        write16(IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPINTEN, 0xFFFF);
        write16(IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPINTEN, 0xFFFF);
        write16(IOExpanderAddress::Upper16Lines, MCP23x17Registers::INTCON, 0);
        write16(IOExpanderAddress::Lower16Lines, MCP23x17Registers::INTCON, 0);
        SPI.endTransaction();
    }
    static Address getAddress() noexcept {
        auto lowerHalf = static_cast<uint32_t>(read16(IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPIO));
        auto upperHalf = static_cast<uint32_t>(read16(IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPIO)) << 16;
        return lowerHalf | upperHalf;
    }
};

inline bool isReadOperation() noexcept {
    return digitalRead(WR) == LOW;
}
void pulse(byte index, decltype(LOW) to = LOW, decltype(HIGH) from = HIGH) noexcept {
    digitalWrite(index, to);
    //delay(1);
    digitalWrite(index, from);
}
bool transactionComplete() noexcept {
    auto done = digitalRead(BLAST) == LOW;
    pulse(MCU_READY);
    return done;
}
enum class ByteEnablePattern : byte {
    Full16 = 0b00,
    Upper8 = 0b01,
    Lower8 = 0b10,
    Error = 0b11,
};
ByteEnablePattern getByteEnablePattern() noexcept {
    return static_cast<ByteEnablePattern>(
            static_cast<byte>(digitalRead(BE0) == HIGH ? 0b01 : 0b00) |
            static_cast<byte>(digitalRead(BE1) == HIGH ? 0b10 : 0b00)
            );
}
void setup() {
    // hold the 4809 in reset as the first thing we do!
    outputPin(RESET4809);
    digitalWrite(RESET4809, LOW);
    outputPin(RESET960);
    digitalWrite(RESET960, LOW);
    digitalWrite(RESET4809, HIGH);
    SPI.begin();
    setupConsole();
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
    outputPin(MCU_READY);

    digitalWrite(PSRAMEN0, HIGH);
    digitalWrite(PSRAMEN1, HIGH);
    digitalWrite(PSRAMEN2, HIGH);
    digitalWrite(PSRAMEN3, HIGH);
    digitalWrite(MCU_READY, HIGH);
    IOExpander::begin();
    setupSDCard();
    /// @todo insert code for setting up other devices here
    delay(1000);
    digitalWrite(RESET960, HIGH);
    //delay(10000);
    // i960 is active after this point
    while (digitalRead(MCU_FAIL) == LOW);
    Serial.println("BOOTED!");
    // attach an interrupt
    attachInterrupt(digitalPinToInterrupt(MCU_FAIL), checksumFailInterrupt, LOW);
}

void loop() {
    //checkForFailPin();
    waitForDataRequest();
    auto targetAddress = IOExpander::getAddress();
    //Serial.print("Target Address: 0x");
    //Serial.println(targetAddress, HEX);
    if (isReadOperation()) {
        // is a read operation
        IOExpander::setupDataLinesForRead();
        do {
            auto pattern = getByteEnablePattern();
            /// @todo get value from a data source
            IOExpander::setDataLines(0);
            if (transactionComplete()) {
                break;
            } else {
                targetAddress += 2;
                //Serial.print("Address is now: 0x");
                //Serial.println(targetAddress, HEX);
                // until I come up with a better synchronization mechanism, this will have to work.
                delayMicroseconds(2);
            }
        } while (true);
        //delay(1);
    } else {
        // is a write operation
        IOExpander::setupDataLinesForWrite();
        do {
            /// @todo set value to data source
            auto pattern = getByteEnablePattern();
            auto valueToCommit = IOExpander::getDataLines();
            if (transactionComplete()) {
                break;
            } else {
                targetAddress += 2;
                //Serial.print("Address is now: 0x");
                //Serial.println(targetAddress, HEX);
                delayMicroseconds(2);
            }
        } while (true);
    }
}
