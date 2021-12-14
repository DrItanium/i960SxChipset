#include <SD.h>
#include <SPI.h>
/*
 * A simple grand central m4 sketch to prototype the i960 chipset interface
 */
using Address = uint32_t;
using DataValue = uint16_t;
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
inline void waitForDataRequest() noexcept {
    while (digitalRead(DEN) != LOW);
}
inline void checkForFailPin() noexcept {
    if (digitalRead(MCU_FAIL) == HIGH) {
        Serial.println("CHECKSUM FAIL!");
        while (true) {
            delay(1000);
        }
    }
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
        digitalWrite(GPIO_CS, LOW);
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        ret.setLowerHalf(SPI.transfer(0));
        ret.setUpperHalf(SPI.transfer(0));
        digitalWrite(GPIO_CS, HIGH);
        return ret.getValue();
    }
    static uint8_t read8(IOExpanderAddress addr, MCP23x17Registers opcode) noexcept {
        digitalWrite(GPIO_CS, LOW);
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        auto result = SPI.transfer(0);
        digitalWrite(GPIO_CS, HIGH);
        return result;
    }
private:
    static void write16(IOExpanderAddress address, MCP23x17Registers opcode, SplitWord16 value) noexcept {
        digitalWrite(GPIO_CS, LOW);
        SPI.transfer(generateWriteOpcode(address));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(value.getLowerHalf());
        SPI.transfer(value.getUpperHalf());
        digitalWrite(GPIO_CS, HIGH);
    }
public:
    static void write16(IOExpanderAddress address, MCP23x17Registers opcode, uint16_t value) noexcept {
        write16(address, opcode, SplitWord16{value});
    }
    static void write8(IOExpanderAddress address, MCP23x17Registers opcode, uint8_t value) noexcept {
        digitalWrite(GPIO_CS, LOW);
        SPI.transfer(generateWriteOpcode(address));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(value);
        digitalWrite(GPIO_CS, HIGH);
    }

    static uint16_t getDataLines() noexcept {
        return read16(IOExpanderAddress::DataLines, MCP23x17Registers::GPIO);
    }
    static void setDataLines(uint16_t value) noexcept {
        write16(IOExpanderAddress::DataLines, MCP23x17Registers::GPIO, value);
        // do nothing right now
    }
};
void
setupIOExpanders() noexcept {

}
void
setupDataLinesForWrite() noexcept {
    // setup the data lines for input since we are writing a value to main memory
}

void
setupDataLinesForRead() noexcept {
    // setup the data lines for output since we are reading a valeu from main memory
}

Address
getAddress() noexcept {
    /// @todo unstub this
    return 0;
}
inline bool isReadOperation() noexcept {
    return digitalRead(WR) == LOW;
}
void pulse(byte index, decltype(LOW) to = LOW, decltype(HIGH) from = HIGH) noexcept {
    digitalWrite(index, to);
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
    outputPin(RESET960);
    digitalWrite(RESET960, LOW);
    SPI.begin();
    setupConsole();
    setupSDCard();
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
    setupIOExpanders();

    digitalWrite(RESET960, HIGH);
    // i960 is active after this point
    while (digitalRead(MCU_FAIL) == LOW) {
        if (digitalRead(DEN) == LOW) {
            break;
        }
    }
    while (digitalRead(MCU_FAIL) == HIGH) {
        if (digitalRead(DEN) == LOW) {
            break;
        }
    }
}

void loop() {
    checkForFailPin();
    waitForDataRequest();
    auto targetAddress = getAddress();
    if (isReadOperation()) {
        // is a read operation
        setupDataLinesForRead();
        do {
            auto pattern = getByteEnablePattern();
            /// @todo get value from a data source
            IOExpander::setDataLines(0);
            if (transactionComplete()) {
                break;
            } else {
                targetAddress += 2;
            }
        } while (true);
    } else {
        // is a write operation
        setupDataLinesForWrite();
        do {
            /// @todo set value to data source
            auto pattern = getByteEnablePattern();
            auto valueToCommit = IOExpander::getDataLines();
            if (transactionComplete()) {
                break;
            } else {
                targetAddress += 2;
            }
        } while (true);
    }
}
