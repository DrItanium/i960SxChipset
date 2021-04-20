/**
 * @file 
 * header only implementation for interfacing with the mcp23s17 digital io
 * expander chip which operates over spi
 * @copyright
 * copyright (c) 2019 joshua scoggins 
 * 
 * this software is provided 'as-is', without any express or implied
 * warranty. in no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. the origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. if you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. this notice may not be removed or altered from any source distribution.
 */
#ifndef LIB_ICS_MCP23S17_H__
#define LIB_ICS_MCP23S17_H__
#include "Arduino.h"
#include "../core/concepts.h"
#include <SPI.h>
namespace bonuspin 
{
template<byte address, int resetPin = -1>
class MCP23x17 {
    public:
        static SPISettings& getSPISettings() noexcept {
            static SPISettings theSettings(10000000, MSBFIRST, SPI_MODE0);
            return theSettings;
        }
        static_assert((address & 0b111) == address, "Provided address is too large!");
    private:
        static constexpr auto generateByte(bool a, bool b, bool c, bool d, bool e, bool f, bool g, bool h) noexcept {
            byte output = 0;
            output |= (b ? 0b0000'0001 : 0);
            output |= (b ? 0b0000'0010 : 0);
            output |= (c ? 0b0000'0100 : 0);
            output |= (d ? 0b0000'1000 : 0);
            output |= (e ? 0b0001'0000 : 0);
            output |= (f ? 0b0010'0000 : 0);
            output |= (g ? 0b0100'0000 : 0);
            output |= (h ? 0b1000'0000 : 0);
            return output;
        }
        static constexpr auto generateIOConByte(bool intPolarity, bool odr, bool haen, bool disslw, bool seqop, bool mirror, bool bank) noexcept {
            return generateByte(false, intPolarity, odr, haen, disslw, seqop, mirror, bank);
        }
    public:
        using Self = MCP23x17<address, resetPin>;
        static constexpr auto BusAddress = address;
        static constexpr auto ResetPin = resetPin;
        static constexpr auto HasResetPin = (ResetPin >= 0);
        constexpr auto getSPIAddress() const noexcept { return _hardwareAddressPinsEnabled ? BusAddress : 0b000; }
        constexpr auto getResetPin() const noexcept { return ResetPin; }
        constexpr auto hasResetPin() const noexcept { return ResetPin >= 0; }
    public:
        MCP23x17() = default;
        // ugh, arduino doesn't implement delete(void*, unsigned int) so I get
        // an error because of this line. I'll disable it for now, these
        // objects should never _ever_ go out of scope
        // This is very gross as we should have a virtual destructor!
        virtual ~MCP23x17() = default;
        Self& operator=(const Self&) = delete; 
        Self& operator=(Self&&) = delete; 
        MCP23x17(const Self&) = delete;
        MCP23x17(Self&&) = delete;
        virtual void enableCS() noexcept = 0;
        virtual void disableCS() noexcept = 0;
        virtual void begin() noexcept {
            if constexpr (HasResetPin) {
                pinMode(ResetPin, OUTPUT);
                digitalWrite(ResetPin, HIGH);
            }
            // on startup registers are sequential, you must actually change
            // the iocon register. Polarity is also active low for interrupt
            // lines
        }
    private:
        class ReadOperation final { };
        class WriteOperation final { };
        constexpr byte generateOpcode(ReadOperation) const noexcept {
            return 0b0100'0000 | (getSPIAddress() << 1) | 1;
        }
        constexpr byte generateOpcode(WriteOperation) const noexcept {
            return 0b0100'0000 | (getSPIAddress() << 1);
        }

        byte read(byte registerAddress) noexcept {
            SPI.beginTransaction(getSPISettings());
            enableCS();
            SPI.transfer(static_cast<uint8_t>(generateOpcode(ReadOperation{})));
            SPI.transfer(static_cast<uint8_t>(registerAddress));
            auto result = SPI.transfer(0x00);
            disableCS();
            SPI.endTransaction();
            return result;
        }
        void write(byte registerAddress, byte value) noexcept {
            SPI.beginTransaction(getSPISettings());
            enableCS();
            SPI.transfer(static_cast<uint8_t>(generateOpcode(WriteOperation{})));
            SPI.transfer(static_cast<uint8_t>(registerAddress));
            SPI.transfer(static_cast<uint8_t>(value));
            disableCS();
            SPI.endTransaction();
        }
        void write16(byte registerAddressA, byte registerAddressB, uint16_t value) noexcept {
            write(registerAddressA, static_cast<byte>(value & 0xFF));
            write(registerAddressB, static_cast<byte>((value & 0xFF00) >> 8));
        }
        uint16_t read16(byte registerAddressA, byte registerAddressB) noexcept {
            return static_cast<uint16_t>(read(registerAddressA)) |
                   (static_cast<uint16_t>(read(registerAddressB)) << 8);
        }
        template<byte seq, byte banked>
        constexpr byte chooseAddress() const noexcept {
            return registersAreSequential() ? seq : banked;
        }
        constexpr auto getIODIRAAddress()   const noexcept { return 0x00; }
        constexpr auto getIODIRBAddress()   const noexcept { return chooseAddress<0x01, 0x10>(); }
        constexpr auto getIOPOLAAddress()   const noexcept { return chooseAddress<0x02, 0x01>(); }
        constexpr auto getIOPOLBAddress()   const noexcept { return chooseAddress<0x03, 0x11>(); }
        constexpr auto getGPINTENAAddress() const noexcept { return chooseAddress<0x04, 0x02>(); }
        constexpr auto getGPINTENBAddress() const noexcept { return chooseAddress<0x05, 0x12>(); }
        constexpr auto getDEFVALAAddress()  const noexcept { return chooseAddress<0x06, 0x03>(); }
        constexpr auto getDEFVALBAddress()  const noexcept { return chooseAddress<0x07, 0x13>(); }
        constexpr auto getIntConAAddress()  const noexcept { return chooseAddress<0x08, 0x04>(); }
        constexpr auto getIntConBAddress()  const noexcept { return chooseAddress<0x09, 0x14>(); }
        constexpr auto getIOConAddress()    const noexcept { return chooseAddress<0x0A, 0x05>(); }
        constexpr auto getGPPUAAddress()    const noexcept { return chooseAddress<0x0C, 0x06>(); }
        constexpr auto getGPPUBAddress()    const noexcept { return chooseAddress<0x0D, 0x16>(); }
        constexpr auto getINTFAAddress()    const noexcept { return chooseAddress<0x0E, 0x07>(); }
        constexpr auto getINTFBAddress()    const noexcept { return chooseAddress<0x0F, 0x17>(); }
        constexpr auto getINTCAPAAddress()  const noexcept { return chooseAddress<0x10, 0x08>(); }
        constexpr auto getINTCAPBAddress()  const noexcept { return chooseAddress<0x11, 0x18>(); }
        constexpr auto getGPIOAAddress()    const noexcept { return chooseAddress<0x12, 0x09>(); }
        constexpr auto getGPIOBAddress()    const noexcept { return chooseAddress<0x13, 0x19>(); }
        constexpr auto getOLATAAddress()    const noexcept { return chooseAddress<0x14, 0x0A>(); }
        constexpr auto getOLATBAddress()    const noexcept { return chooseAddress<0x15, 0x1A>(); }
    public:
        constexpr bool registersAreInSeparateBanks() const noexcept { return !_registersAreSequential; }
        constexpr bool registersAreSequential() const noexcept { return _registersAreSequential; }
        constexpr bool interruptPinsAreActiveLow() const noexcept { return _polarityIsActiveLow; }
        constexpr bool interruptPinsAreActiveHigh() const noexcept { return !_polarityIsActiveLow; }
        constexpr bool hardwareAddressEnabled() const noexcept { return _hardwareAddressPinsEnabled; }
        constexpr bool hardwareAddressDisabled() const noexcept { return !_hardwareAddressPinsEnabled; }
        void refreshIOCon() noexcept {
            auto result = getIOCon();
            _registersAreSequential = ((result & 0b1000'0000) == 0);
            _polarityIsActiveLow = ((result & 0b0000'0010) == 0);
            _hardwareAddressPinsEnabled = ((result & 0b0000'1000) != 0);
        }
        byte getIOCon() noexcept { return read(getIOConAddress()); }
        void setIOCon(byte value) noexcept {
            write(getIOConAddress(), value);
            refreshIOCon();
        }
        void makeRegistersSequential() noexcept {
            if (!_registersAreSequential) {
                setIOCon(getIOCon() & 0b0111'1110);
            }
        }
        void makeRegistersBanked() noexcept {
            if (_registersAreSequential) {
                setIOCon(getIOCon() | 0b1000'0000);
            }
        }
        void makeInterruptOutputLinesActiveLow() noexcept {
            if (!_polarityIsActiveLow) {
                setIOCon(getIOCon() & 0b0111'1100);
            }
        }
        void makeInterruptOutputLinesActiveHigh() noexcept {
            if (_polarityIsActiveLow) {
                setIOCon(getIOCon() | 0b0000'0010);
            }
        }
        void reset() noexcept {
            // always delay for 2 microseconds even if reset is not actually
            // connected to a pin for consistency
            volatile HoldPinLow<resetPin> holder;
            delayMicroseconds(2);
        }
        uint16_t readGPIOs() noexcept { return read16(getGPIOAAddress(), getGPIOBAddress()); }
        void writeGPIOs(uint16_t pattern) noexcept { write16(getGPIOAAddress(), getGPIOBAddress(), pattern); }

        uint16_t readGPIOsDirection() noexcept { return read16(getIODIRAAddress(), getIODIRBAddress()); }
        void writeGPIOsDirection(uint16_t pattern) noexcept { write16(getIODIRAAddress(), getIODIRBAddress(), pattern); }

        uint16_t readGPIOPolarity() noexcept { return read16(getIOPOLAAddress(), getIOPOLBAddress()); }
        void writeGPIOPolarity(uint16_t pattern) noexcept { write16(getIOPOLAAddress(), getIOPOLBAddress(), pattern); }

        uint16_t readGPIOInterruptEnable() noexcept { return read16(getGPINTENAAddress(), getGPINTENBAddress()); }
        void writeGPIOInterruptEnable(uint16_t pattern) noexcept { write16(getGPINTENAAddress(), getGPINTENBAddress(), pattern); }

        uint16_t readDefaultCompareRegisterForInterruptOnChange() noexcept { return read16(getDEFVALAAddress(), getDEFVALBAddress()); }
        void writeDefaultCompareRegisterForInterruptOnChange(uint16_t pattern) noexcept { write16(getDEFVALAAddress(), getDEFVALBAddress(), pattern); }

        uint16_t readInterruptOnChangeControlRegister() noexcept { return read16(getIntConAAddress(), getIntConBAddress()); }
        void writeInterruptOnChangeControlRegister(uint16_t pattern) noexcept { write16(getIntConAAddress(), getIntConBAddress(), pattern); }

        uint16_t readGPIOPullup() noexcept { return read16(getGPPUAAddress(), getGPPUBAddress()); }
        void writeGPIOPullup(uint16_t pattern) noexcept { write16(getGPPUAAddress(), getGPPUBAddress(), pattern); }
        uint16_t readGPIOInterruptFlags() noexcept { return read16(getINTFAAddress(), getINTFBAddress()); }
        uint16_t readGPIOInterruptCapturedRegister() noexcept { return read16(getINTCAPAAddress(), getINTCAPBAddress()); }
        uint16_t readOutputLatch() noexcept { return read16(getOLATAAddress(), getOLATBAddress()); }
        void writeOutputLatch(uint16_t pattern) noexcept { return write16(getOLATAAddress(), getOLATBAddress(), pattern); }

        void enableHardwareAddressPins() noexcept {
            if (!_hardwareAddressPinsEnabled) {
                setIOCon(getIOCon() | 0b0000'1000);
            }
        }
        void disableHardwareAddressPins() noexcept {
            if (_hardwareAddressPinsEnabled) {
                setIOCon(getIOCon() & 0b1111'0110);
            }
        }
        void interruptPinsAreMirrored() noexcept {
            setIOCon(getIOCon() | 0b0100'0000);
        }
        void interruptPinsAreIndependent() noexcept {
            setIOCon(getIOCon() & 0b1011'1110);
        }
        static constexpr uint16_t BitMasks[] = {
            1,
            1 << 1,
            1 << 2,
            1 << 3,
            1 << 4,
            1 << 5,
            1 << 6,
            1 << 7,
            1 << 8,
            1 << 9,
            1 << 10,
            1 << 11,
            1 << 12,
            1 << 13,
            1 << 14,
            1 << 15,
        };
        void digitalWrite(uint8_t pin, uint8_t value) noexcept {
            if (pin > 15) {
                return;
            }
            uint16_t toWriteBack = readGPIOs();
            if (auto pinMask = BitMasks[pin]; value == LOW) {
                toWriteBack &= ~pinMask;
            } else {
                toWriteBack |= pinMask;
            }
            writeGPIOs(toWriteBack);
        }
        int digitalRead(uint8_t pin) {
            if (pin > 15) {
                return -1;
            }  else {
                return (readGPIOs() & BitMasks[pin]) ? HIGH : LOW;
            }
        }
        void pinMode(uint8_t pin, decltype(INPUT) kind) {
            auto maskedValue = BitMasks[pin];
            auto dirmask = readGPIOsDirection();
            if (kind == INPUT_PULLUP) {
                auto pullups = readGPIOPullup();
                writeGPIOPullup(pullups | dirmask);
                writeGPIOsDirection(maskedValue | dirmask);
            } else if (kind == INPUT) {
                writeGPIOsDirection(maskedValue | dirmask);
            } else {
                writeGPIOsDirection((~maskedValue) & dirmask);
            }
        }
        void writePortB(uint8_t value) {
            write(getGPIOBAddress(), value);
        }
    private:
        bool _registersAreSequential = true;
        bool _polarityIsActiveLow = true;
        bool _hardwareAddressPinsEnabled = false;
};

template<byte address, int chipEnable, int resetPin = -1>
class MCP23S17 : public MCP23x17<address, resetPin> {
    public:
        using Parent = MCP23x17<address, resetPin>;
        using Self = MCP23S17<address, chipEnable, resetPin>;
        Self& operator=(const Self&) = delete; 
        Self& operator=(Self&&) = delete; 
        MCP23S17(const Self&) = delete;
        MCP23S17(Self&&) = delete;
        static_assert(chipEnable >= 0, "Must bind the chip enable to a real pin!");
        static constexpr auto ChipEnablePin = chipEnable;
        constexpr auto getChipEnablePin() const noexcept { return ChipEnablePin; }
        MCP23S17() = default;
        ~MCP23S17() override = default;
        void enableCS() noexcept override {
            digitalWrite(ChipEnablePin, LOW);
        }
        void disableCS() noexcept override {
            digitalWrite(ChipEnablePin, HIGH);
        }
        void begin() noexcept override {
            Parent::begin();
            pinMode(ChipEnablePin, OUTPUT);
            digitalWrite(ChipEnablePin, HIGH);
        }
};



} // end namespace bonuspin

template<byte address, int resetPin = -1>
void digitalWrite(uint8_t pin, uint8_t value, bonuspin::MCP23x17<address, resetPin>& mcp) noexcept {
    mcp.digitalWrite(pin, value);
}

template<byte address, int resetPin = -1>
auto digitalRead(uint8_t pin, bonuspin::MCP23x17<address, resetPin>& mcp) noexcept {
    return mcp.digitalRead(pin);
}

template<byte address, int resetPin = -1>
void pinMode(uint8_t pin, decltype(INPUT) kind, bonuspin::MCP23x17<address, resetPin>& mcp) noexcept {
    mcp.pinMode(pin, kind);
}


#endif // end LIB_ICS_MCP23S17_H__
