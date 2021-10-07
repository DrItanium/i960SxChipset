//
// Created by jwscoggins on 7/31/21.
//

#ifndef I960SXCHIPSET_PSRAMCHIP_H
#define I960SXCHIPSET_PSRAMCHIP_H
#include <Arduino.h>
#include <SPI.h>
#include "MCUPlatform.h"
#include "Pinout.h"
#include "MemoryThing.h"
#include "TaggedCacheAddress.h"
/**
 * @brief Interface to the memory connected to the chipset
 * @tparam enablePin The pin that is used to signal chip usage
 * @tparam sel0 The lowest pin used to select the target device
 * @tparam sel1 The middle pin used to select the target device
 * @tparam sel2 The upper pin used to select the target device
 */
template<i960Pinout enablePin>
class MemoryBlock {
public:
    static constexpr auto EnablePin = enablePin;
#ifdef CHIPSET_TYPE1
    static constexpr auto Select0 = i960Pinout::SPI_OFFSET0;
    static constexpr auto Select1 = i960Pinout::SPI_OFFSET1;
    static constexpr auto Select2 = i960Pinout::SPI_OFFSET2;
    static constexpr auto NumChips = 8;
    static_assert ((EnablePin != Select0) && (EnablePin != Select1) && (EnablePin != Select2), "The enable pin must be different from all select pins");
    static_assert ((Select0 != Select1) && (Select0 != Select2) && (Select1 != Select2), "All three select pins must point to a different physical pin");
#elif defined(CHIPSET_TYPE3)
    // in hardware, we bind two psrams to the same enable pin across two spi busses
    // externally, we present this as a single class
    static constexpr auto NumChips = 2;
#endif
public:
    MemoryBlock() = delete;
    ~MemoryBlock() = delete;
    union PSRAMBlockAddress {
        constexpr explicit PSRAMBlockAddress(Address value = 0) : base(value) { }
        constexpr auto getAddress() const noexcept { return base; }
        constexpr auto getOffset() const noexcept { return offset; }
        constexpr auto getIndex() const noexcept { return index; }
        Address base;
        struct {
            Address offset : 23;
            byte index : 3;
        };
        byte bytes_[4];
    };
private:
    enum class InterleavedMultibyteTransmissionStyle : byte {
        Mirrored = 0,
        SPI0_SPI1,
        SPI1_SPI0,
        Count, // must be last
    };
    template<InterleavedMultibyteTransmissionStyle style>
    static constexpr bool ValidTransmissionStyle_v = static_cast<byte>(style) < static_cast<byte>(InterleavedMultibyteTransmissionStyle::Count);
    static inline void transmitByte(byte value) noexcept {
        SPDR = value;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
    }
    static inline byte mspimTransfer(byte value) noexcept {
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = value;
        while (!(UCSR1A & (1 << RXC1)));
        return UDR1;
    }
    static inline void transmitByte_Bus2(byte value) noexcept {
       UDR1 = value;
       asm volatile ("nop");
       while (!(UCSR1A & (1 << UDRE1)));
    }
    static inline void transmitByte_Interleaved(byte value) noexcept {
        UDR1 = value;
        SPDR = value;
        asm volatile ("nop");
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
    }
    static inline void transmitTwoByte(uint8_t first, uint8_t second) noexcept {
        SPDR = first;
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF)));
        SPDR = second;
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF)));
    }
    static inline void transmitTwoByte_Bus2(uint8_t first, uint8_t second) noexcept {
        UDR1 = first;
        UDR1 = second;
        asm volatile ("nop");
        while (!(UCSR1A & (1 << UDRE1)));
    }
    template<InterleavedMultibyteTransmissionStyle style>
    static inline void transmitTwoByte_Interleaved(uint8_t first, uint8_t second) noexcept {
        static_assert(ValidTransmissionStyle_v<style>, "Unimplemented or illegal transmission style specified");
        if constexpr (style == InterleavedMultibyteTransmissionStyle::Mirrored) {
            UDR1 = first;
            SPDR = first;
            UDR1 = second;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF)));
            SPDR = second;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF)));
            while (!(UCSR1A & (1 << UDRE1)));
        } else if constexpr (style == InterleavedMultibyteTransmissionStyle::SPI0_SPI1) {
            SPDR = first;
            UDR1 = second;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF)));
            while (!(UCSR1A & (1 << UDRE1)));
        } else if constexpr (style == InterleavedMultibyteTransmissionStyle::SPI1_SPI0) {
            UDR1 = first;
            SPDR = second;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF)));
            while (!(UCSR1A & (1 << UDRE1)));
        } else {
            // do nothing
        }
    }
    enum class OperationKind {
        Write,
        Read,
        Generic,
    };

    template<byte opcode, OperationKind kind = OperationKind::Generic>
    inline static size_t genericReadWriteOperation(uint32_t address, byte* buf, size_t capacity) noexcept {
        if (capacity == 0) {
            return 0;
        }
        PSRAMBlockAddress curr(address);
        setChipId(curr.getIndex());
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        SPDR = opcode;
        asm volatile("nop");
        PSRAMBlockAddress end(address + capacity);
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = curr.bytes_[2];
        asm volatile("nop");
        auto numBytesToSecondChip = end.getOffset();
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = curr.bytes_[1];
        asm volatile("nop");
        auto localToASingleChip = curr.getIndex() == end.getIndex();
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = curr.bytes_[0];
        asm volatile("nop");
        auto numBytesToFirstChip = localToASingleChip ? capacity : (capacity - numBytesToSecondChip);
        while (!(SPSR & _BV(SPIF))) ; // wait
        if constexpr (kind == OperationKind::Write) {
            for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; ++i) {
                transmitByte(buf[i]);
            }
        } else if constexpr (kind == OperationKind::Read) {
            for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; ++i) {
                transmitByte(0);
                buf[i] = SPDR;
            }
        } else {
            SPI.transfer(buf, numBytesToFirstChip);
        }
        digitalWrite<EnablePin, HIGH>();
        if (!localToASingleChip && (numBytesToSecondChip > 0)) {
            // since size_t is 16-bits on AVR we can safely reduce the largest buffer size 64k, thus we can only ever span two psram chips at a time
            // thus we can actually convert this work into two separate spi transactions
            // start writing at the start of the next chip the remaining number of bytes
            setChipId(end.getIndex());
            // we start at address zero on this new chip always
            digitalWrite<EnablePin, LOW>();
            transmitByte(opcode);
            transmitByte(0);
            transmitByte(0);
            transmitByte(0);
            if constexpr (kind == OperationKind::Write) {
                auto count = numBytesToSecondChip;
                auto actualBuf = buf + numBytesToFirstChip;
                for (decltype(count) i = 0; i < count; ++i) {
                    transmitByte(actualBuf[i]);
                }
            } else if (kind == OperationKind::Read) {
                auto count = numBytesToSecondChip;
                auto actualBuf = buf + numBytesToFirstChip;
                for (decltype(count) i = 0; i < count; ++i) {
                    transmitByte(0);
                    actualBuf[i] = SPDR;
                }
            } else {
                SPI.transfer(buf + numBytesToFirstChip, numBytesToSecondChip);
            }
            digitalWrite<EnablePin, HIGH>();
        }
        SPI.endTransaction();
        return capacity;
    }
public:
    static void writeCacheLine(TaggedAddress address, const byte* buf) noexcept {
        //return genericCacheLineReadWriteOperation<0x02, OperationKind::Write>(address, buf);
        // unlike a generic read/write operation, tagged addresses will never actually span multiple devices so there is no
        // need to do the offset calculation
        setChipId(address.getPSRAMChipId());
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        transmitByte(0x02);
        transmitByte(address.getPSRAMAddress_High());
        transmitByte(address.getPSRAMAddress_Middle());
        transmitByte(address.getPSRAMAddress_Low());
        for (int i = 0; i < 16; ++i) {
            transmitByte(buf[i]);
        }
        digitalWrite<EnablePin, HIGH>();
        SPI.endTransaction();
    }
    static void readCacheLine(TaggedAddress address, byte* buf) noexcept {
        // unlike a generic read/write operation, tagged addresses will never actually span multiple devices so there is no
        // need to do the offset calculation
        setChipId(address.getPSRAMChipId());
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        transmitByte(0x03);
        transmitByte(address.getPSRAMAddress_High());
        transmitByte(address.getPSRAMAddress_Middle());
        transmitByte(address.getPSRAMAddress_Low());
        for (int i = 0; i < 16; ++i) {
            transmitByte(0);
            buf[i] = SPDR;
        }
        digitalWrite<EnablePin, HIGH>();
        SPI.endTransaction();
    }
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x02, OperationKind::Write>(address, buf, capacity);
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x03, OperationKind::Read>(address, buf, capacity);
    }
private:
    static void setChipId(byte index) noexcept {
#ifdef CHIPSET_TYPE1
        if (index != currentIndex_) {
            digitalWrite<Select0>((index & (1 << 0)) ? HIGH : LOW);
            digitalWrite<Select1>((index & (1 << 1)) ? HIGH : LOW);
            digitalWrite<Select2>((index & (1 << 2)) ? HIGH : LOW);
            currentIndex_ = index;
        }
#else
// do nothing
#endif
    }
public:
    static void begin() noexcept {
        static bool initialized_ = false;
        if (!initialized_) {
            initialized_ = true;
            setChipId(0);
            SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
            for (int i = 0; i < NumChips; ++i) {
                setChipId(i);
                delayMicroseconds(200); // give the psram enough time to come up regardless of where you call begin
                digitalWrite<enablePin, LOW>();
                SPI.transfer(0x66);
                digitalWrite<enablePin, HIGH>();
                asm volatile ("nop");
                asm volatile ("nop");
                digitalWrite<enablePin, LOW>();
                SPI.transfer(0x99);
                digitalWrite<enablePin, HIGH>();
            }
            SPI.endTransaction();
        }
    }
private:
    static inline byte currentIndex_ = 0xFF;
};

/**
 * @brief Controls two 8mb psram chips connected to two separate spi busses concurrently.
 * Since the memory interface operates exclusively on 16-byte cache lines, the memory usage is cut in half on each one.
 * A safe analogy would be like RAID-0/Striping where we fill up both chips concurrently. The way we store the data to
 * the psram chips is inconsequential to the external interface. The important part is making sure we take advantage of
 * both busses simultaneously.
 */
template<i960Pinout enablePin>
class DualChannelMemoryBlock {
public:
    static constexpr auto EnablePin = enablePin;
    // The real issue with the design is the fact that the design is going to turn one 16-byte (or whatever width) cache line into
    // two separate 8-byte (or n/2) cache lines that store at the same address. Doing some experiments, this seems to be as simple as
    // shifting right by 1.
public:
    DualChannelMemoryBlock() = delete;
    ~DualChannelMemoryBlock() = delete;
    /**
     * @brief Describes the block address decomposition for a dual spi bus setup.
     */
    union PSRAMBlockAddress {
        constexpr explicit PSRAMBlockAddress(Address value = 0) : base(value) { }
        constexpr auto getAddress() const noexcept { return base; }
        constexpr auto getOffset() const noexcept { return offset; }
        constexpr auto getIndex() const noexcept { return index; }
        /**
         * @brief Return the actual address that will be passed to the psram chips
         * @return The system address shifted right by 1
         */
        constexpr auto computeSingleChipAddress() const noexcept { return offset >> 1; }
        Address base;
        struct {
            Address offset : 24;
            byte index : 2; // currently type 3 allows up to 32-megabytes, but 64-megs is a good target
        };
        byte bytes_[4];
    };
public:
    static void writeCacheLine(TaggedAddress address, const byte* buf) noexcept {
        //return genericCacheLineReadWriteOperation<0x02, OperationKind::Write>(address, buf);
        // unlike a generic read/write operation, tagged addresses will never actually span multiple devices so there is no
        // need to do the offset calculation

        digitalWrite<EnablePin, LOW>();
        UDR1 = 0x02;
        SPDR = 0x02;
        TaggedAddress correctAddress(address.getAddress() >> 1);
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
        UDR1 = correctAddress.getPSRAMAddress_High();
        SPDR = correctAddress.getPSRAMAddress_High();
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
        UDR1 = correctAddress.getPSRAMAddress_Middle();
        SPDR = correctAddress.getPSRAMAddress_Middle();
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
        UDR1 = correctAddress.getPSRAMAddress_Low();
        SPDR = correctAddress.getPSRAMAddress_Low();
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
        for (int i = 0; i < 16; i+=2) {
            // okay this is the important part, we need to maintain consistency
            // bus 1 -> even addresses
            // bus 0 -> odd addresses
            UDR1 = buf[i];
            SPDR = buf[i+1];
            while (!(UCSR1A & (1 << UDRE1)));
            while (!(SPSR & (1 << SPIF)));
        }
        digitalWrite<EnablePin, HIGH>();
    }
    static void readCacheLine(TaggedAddress address, byte* buf) noexcept {
        // unlike a generic read/write operation, tagged addresses will never actually span multiple devices so there is no
        // need to do the offset calculation
        digitalWrite<EnablePin, LOW>();
        UDR1 = 0x03;
        SPDR = 0x03;
        TaggedAddress correctAddress(address.getAddress() >> 1);
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
        UDR1 = correctAddress.getPSRAMAddress_High();
        SPDR = correctAddress.getPSRAMAddress_High();
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
        UDR1 = correctAddress.getPSRAMAddress_Middle();
        SPDR = correctAddress.getPSRAMAddress_Middle();
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
        UDR1 = correctAddress.getPSRAMAddress_Low();
        SPDR = correctAddress.getPSRAMAddress_Low();
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
        for (int i = 0; i < 16; i+=2) {
            // okay this is the important part, we need to maintain consistency
            // bus 1 -> even addresses
            // bus 0 -> odd addresses
            UDR1 = 0;
            SPDR = 0;
            while (!(UCSR1A & (1 << UDRE1)));
            buf[i] = UDR1;
            while (!(SPSR & (1 << SPIF)));
            buf[i+1] = SPDR;
        }
        digitalWrite<EnablePin, HIGH>();
    }
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        //return genericReadWriteOperation<0x02, OperationKind::Write>(address, buf, capacity);
        // unlike with the single spi bus linear design, this class assumes that you can _only_ write to the
        // entire
        if (capacity == 0) {
            return 0;
        }
        PSRAMBlockAddress curr(address);
        digitalWrite<EnablePin, LOW>();
        UDR1 = 0x02;
        SPDR = 0x02;
        asm volatile("nop");
        PSRAMBlockAddress end(address + capacity);
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = curr.bytes_[2];
        SPDR = curr.bytes_[2];
        asm volatile("nop");
        auto numBytesToSecondChip = end.getOffset();
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = curr.bytes_[1];
        SPDR = curr.bytes_[1];
        asm volatile("nop");
        auto localToASingleChip = curr.getIndex() == end.getIndex();
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = curr.bytes_[0];
        SPDR = curr.bytes_[0];
        asm volatile("nop");
        auto numBytesToFirstChip = localToASingleChip ? capacity : (capacity - numBytesToSecondChip);
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        /// @todo fix
        for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; i+=2) {
            UDR1 = buf[i];
            SPDR = buf[i+1];
            asm volatile("nop");
            while (!(SPSR & _BV(SPIF))) ; // wait
            while (!(UCSR1A & (1 << UDRE1)));
        }
        digitalWrite<EnablePin, HIGH>();
        return numBytesToFirstChip;
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        //return genericReadWriteOperation<0x03, OperationKind::Read>(address, buf, capacity);
        // unlike with the single spi bus linear design, this class assumes that you can _only_ write to the
        // entire
        if (capacity == 0) {
            return 0;
        }
        PSRAMBlockAddress curr(address);
        digitalWrite<EnablePin, LOW>();
        UDR1 = 0x03;
        SPDR = 0x03;
        asm volatile("nop");
        PSRAMBlockAddress end(address + capacity);
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = curr.bytes_[2];
        SPDR = curr.bytes_[2];
        asm volatile("nop");
        auto numBytesToSecondChip = end.getOffset();
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = curr.bytes_[1];
        SPDR = curr.bytes_[1];
        asm volatile("nop");
        auto localToASingleChip = curr.getIndex() == end.getIndex();
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = curr.bytes_[0];
        SPDR = curr.bytes_[0];
        asm volatile("nop");
        auto numBytesToFirstChip = localToASingleChip ? capacity : (capacity - numBytesToSecondChip);
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        /// @todo fix
        for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; i+=2) {
            UDR1 = 0;
            SPDR = 0;
            asm volatile("nop");
            while (!(UCSR1A & (1 << UDRE1)));
            while (!(SPSR & _BV(SPIF))) ; // wait
            buf[i] = UDR1;
            buf[i+1] = SPDR;
        }
        digitalWrite<EnablePin, HIGH>();
        return numBytesToFirstChip;
    }
private:
public:
    static void begin() noexcept {
        static bool initialized_ = false;
        if (!initialized_) {
            initialized_ = true;
            delayMicroseconds(200); // give the psram enough time to come up regardless of where you call begin
            digitalWrite<EnablePin, LOW>();
            UDR1 = 0x66;
            SPDR = 0x66;
            asm volatile ("nop");
            while (!(UCSR1A & (1 << UDRE1)));
            while (!(SPSR & (1 << SPIF)));
            digitalWrite<EnablePin, HIGH>();
            digitalWrite<EnablePin, LOW>();
            SPI.transfer(0x99);
            UDR1 = 0x99;
            SPDR = 0x99;
            asm volatile ("nop");
            while (!(UCSR1A & (1 << UDRE1)));
            while (!(SPSR & (1 << SPIF)));
            digitalWrite<EnablePin, HIGH>();
        }
    }
};
class Type3MemoryInterface {
public:
// lower 16 megabytes
    using Lower16Megabytes = DualChannelMemoryBlock<i960Pinout::PSRAM_EN0>;
// upper 16 megabytes
    using Upper16Megabytes = DualChannelMemoryBlock<i960Pinout::PSRAM_EN1>;
    using PSRAMBlockAddress = Lower16Megabytes::PSRAMBlockAddress;
public:
    Type3MemoryInterface() = delete;
    ~Type3MemoryInterface() = delete;
    static void begin() noexcept {
        Lower16Megabytes::begin();
        Upper16Megabytes::begin();
    }
    static void writeCacheLine(TaggedAddress address, const byte* buf) noexcept {
        switch (address.getPSRAMChipId()) {
            case 0:
                Lower16Megabytes::writeCacheLine(address, buf);
                break;
            case 1:
                Upper16Megabytes::writeCacheLine(address, buf);
                break;
            default:
                break;
        }
    }
    static void readCacheLine(TaggedAddress address, byte* buf) noexcept {
        switch (address.getPSRAMChipId()) {
            case 0:
                Lower16Megabytes::readCacheLine(address, buf);
                break;
            case 1:
                Upper16Megabytes::readCacheLine(address, buf);
                break;
            default:
                break;
        }
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        if (PSRAMBlockAddress addr(address); addr.getIndex() == 0) {
            if (auto result = Lower16Megabytes ::read(address, buf, capacity); result != capacity) {
                auto difference = capacity - result;
                // transfer the remaining
                return Upper16Megabytes :: read(address + result, buf + result, difference) + result;
            } else {
                return result;
            }
        } else if (addr.getIndex() == 1) {
            // only read from mapped memory
            return Upper16Megabytes :: read(address, buf, capacity);
        } else {
            return 0;
        }
    }
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        if (PSRAMBlockAddress addr(address); addr.getIndex() == 0) {
            if (auto result = Lower16Megabytes ::write(address, buf, capacity); result != capacity) {
                auto difference = capacity - result;
                // transfer the remaining
                return Upper16Megabytes :: write(address + result, buf + result, difference) + result;
            } else {
                return result;
            }
        } else if (addr.getIndex() == 1) {
            // only read from mapped memory
            return Upper16Megabytes :: write(address, buf, capacity);
        } else {
            return 0;
        }
    }
};

#if defined(CHIPSET_TYPE1)
using OnboardPSRAMBlock = MemoryBlock<i960Pinout::PSRAM_EN>;
#elif defined(CHIPSET_TYPE3)

using OnboardPSRAMBlock = Type3MemoryInterface;
#endif
#endif //I960SXCHIPSET_PSRAMCHIP_H
