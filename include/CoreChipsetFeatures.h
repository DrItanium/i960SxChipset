//
// Created by jwscoggins on 6/21/21.
//

#ifndef I960SXCHIPSET_CORECHIPSETFEATURES_H
#define I960SXCHIPSET_CORECHIPSETFEATURES_H
#include "MemoryThing.h"
#include "ProcessorSerializer.h"
class CoreChipsetFeatures : public IOSpaceThing {
public:
    static constexpr auto AllowDebuggingStatements = true;
    enum class Registers : uint32_t {
        Led, // one byte
        DisplayMemoryReadsAndWrites,
        DisplayCacheLineUpdates,
        PortZGPIO = 0x10, // one byte wide
        PortZGPIODirection, // one byte wide
        PortZGPIOPolarity,
        PortZGPIOPullup,
        ConsoleFlush = 0x20, // 2 bytes for alignment purposes
        ConsoleFlushUpper, // 2 bytes for alignment purposes
#define TwoByteEntry(Prefix) Prefix, Prefix ## Upper
#define FourByteEntry(Prefix) Prefix ## LowerHalf, Prefix ## UpperLowerHalf, Prefix ## UpperHalf, Prefix ## UpperUpperHalf
        TwoByteEntry(ConsoleAvailable),
        TwoByteEntry(ConsoleAvailableForWrite),
        TwoByteEntry(ConsoleIO),
        /// @todo implement console buffer
        FourByteEntry(ConsoleBufferAddress),
        ConsoleBufferLength, // up to 256 bytes in length
        ConsoleBufferDoorbell, // read from this to do a buffered read, write to this to do a write from memory to console
#undef FourByteEntry
#undef TwoByteEntry
    };
    explicit CoreChipsetFeatures(Address offsetFromIOBase = 0) : IOSpaceThing(offsetFromIOBase, offsetFromIOBase + 0x100) {
        consoleBufferBaseAddress_.full = 0;

    }
    ~CoreChipsetFeatures() override = default;
private:
    uint8_t buffer_[256] = { 0 };
    uint8_t
    readFromStream(Stream& aStream, Address baseAddress, uint8_t length) noexcept {
        uint8_t count = 0;
        if (auto thing = getThing(baseAddress, LoadStoreStyle::Lower8); thing) {
            // force the i960sx to wait
            count = aStream.readBytes(buffer_, length);
            thing->write(thing->makeAddressRelative(baseAddress), buffer_, length);
        }
        return count;
    }
    uint8_t
    writeToStream(Stream& aStream, Address baseAddress, uint8_t length) noexcept {
        uint8_t count = 0;
        if (auto thing = getThing(baseAddress, LoadStoreStyle::Lower8); thing) {
            thing->read(thing->makeAddressRelative(baseAddress), buffer_, length);
            count = aStream.write(buffer_, length);
        }
        return count;
    }
public:
    [[nodiscard]] uint8_t read8(Address address) noexcept override {
            switch (static_cast<Registers>(address)) {
                case Registers::Led:
                    return readLed();
                case Registers::DisplayMemoryReadsAndWrites:
                    return displayMemoryReadsAndWrites();
                case Registers::DisplayCacheLineUpdates:
                    return displayCacheLineUpdates();
                case Registers::PortZGPIO:
                    return ProcessorInterface::getInterface().readPortZGPIORegister();
                case Registers::PortZGPIODirection:
                    return ProcessorInterface::getInterface().getPortZDirectionRegister();
                case Registers::PortZGPIOPolarity:
                    return ProcessorInterface::getInterface().getPortZPolarityRegister();
                case Registers::PortZGPIOPullup:
                    return ProcessorInterface::getInterface().getPortZPullupResistorRegister();
                case Registers::ConsoleBufferLength:
                    return consoleBufferLength_;
                case Registers::ConsoleBufferDoorbell:
                    return readFromStream(Serial, consoleBufferBaseAddress_.full, consoleBufferLength_);
                default:
                    return 0;
            }
    }
    [[nodiscard]] uint16_t read16(Address address) noexcept override {
            switch (static_cast<Registers>(address)) {
                case Registers::ConsoleIO: return Serial.read();
                case Registers::ConsoleAvailable: return Serial.available();
                case Registers::ConsoleAvailableForWrite: return Serial.availableForWrite();
                case Registers::ConsoleBufferAddressLowerHalf: return consoleBufferBaseAddress_.halves[0];
                case Registers::ConsoleBufferAddressUpperHalf: return consoleBufferBaseAddress_.halves[1];
                default: return 0;
            }
    }
    void write16(Address address, uint16_t value) noexcept override {
            switch (static_cast<Registers>(address)) {
                case Registers::ConsoleFlush:
                    Serial.flush();
                break;
                case Registers::ConsoleIO:
                    Serial.write(static_cast<char>(value));
                break;
                case Registers::ConsoleBufferAddressLowerHalf:
                    consoleBufferBaseAddress_.halves[0] = value;
                break;
                case Registers::ConsoleBufferAddressUpperHalf:
                    consoleBufferBaseAddress_.halves[1] = value;
                break;
                default:
                    break;
            }

    }
    void write8(Address address, uint8_t value) noexcept override {
            switch (static_cast<Registers>(address)) {
                case Registers::Led:
                    writeLed(value);
                break;
                case Registers::DisplayMemoryReadsAndWrites:
                    setDisplayMemoryReadsAndWrites(value != 0);
                break;
                case Registers::DisplayCacheLineUpdates:
                    setDisplayCacheLineUpdates(value != 0);
                break;
                case Registers::PortZGPIO:
                    ProcessorInterface::getInterface().writePortZGPIORegister(value);
                break;
                case Registers::PortZGPIOPullup:
                    ProcessorInterface::getInterface().setPortZPullupResistorRegister(value);
                break;
                case Registers::PortZGPIOPolarity:
                    ProcessorInterface::getInterface().setPortZPolarityRegister(value);
                break;
                case Registers::PortZGPIODirection:
                    ProcessorInterface::getInterface().setPortZDirectionRegister(value);
                break;
                case Registers::ConsoleBufferLength:
                    consoleBufferLength_ = value;
                break;
                case Registers::ConsoleBufferDoorbell:
                    (void)writeToStream(Serial, consoleBufferBaseAddress_.full, consoleBufferLength_);
                break;
                default:
                    break;
            }
    }
    [[nodiscard]] constexpr bool displayMemoryReadsAndWrites() const noexcept { return AllowDebuggingStatements && displayMemoryReadsAndWrites_; }
    [[nodiscard]] constexpr bool displayCacheLineUpdates() const noexcept { return AllowDebuggingStatements && displayCacheLineUpdates_; }
    void setDisplayMemoryReadsAndWrites(bool value) noexcept {
        if constexpr (AllowDebuggingStatements) {
            displayMemoryReadsAndWrites_ = value;
        }
    }
    void setDisplayCacheLineUpdates(bool value) noexcept {
        if constexpr (AllowDebuggingStatements) {
            displayCacheLineUpdates_ = value;
        }
    }
    [[nodiscard]] constexpr bool debuggingActive() const noexcept { return AllowDebuggingStatements; }
    void begin() noexcept override {
    }
private:
    static void
    writeLed(uint8_t value) noexcept {
        digitalWrite(i960Pinout::Led, value > 0 ? HIGH : LOW);
    }
    static uint8_t
    readLed() noexcept {
        return static_cast<uint8_t>(digitalRead(i960Pinout::Led));
    }
    bool displayMemoryReadsAndWrites_ = false;
    bool displayCacheLineUpdates_ = false;
    union {
        uint32_t full = 0;
        uint16_t halves[2];
    } consoleBufferBaseAddress_;
    uint8_t consoleBufferLength_ = 0;
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
