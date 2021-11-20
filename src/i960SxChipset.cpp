/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/// i960Sx chipset mcu, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include <SPI.h>
#include <SdFat.h>
#include "Pinout.h"

#include "CacheEntry.h"
#include "DirectMappedCacheWay.h"
#include "TwoWayLRUCacheEntry.h"
#include "ProcessorSerializer.h"
#include "MemoryThing.h"
#include "DisplayInterface.h"
#include "CoreChipsetFeatures.h"
#include "PSRAMChip.h"
#include "TaggedCacheAddress.h"
#include "RTCInterface.h"
#include "i960SxChipset.h"
#include "type_traits.h"

constexpr auto RTCBaseAddress = 0xFA00'0000;
constexpr auto Serial0BaseAddress = 0xFB00'0000;
constexpr auto DisplayBaseAddress = 0xFC00'0000;
constexpr auto SDBaseAddress = 0xFD00'0000;
constexpr auto MaximumNumberOfOpenFiles = 16;
constexpr auto CompileInAddressDebuggingSupport = false;
constexpr auto AddressDebuggingEnabledOnStartup = false;
using TheDisplayInterface = DisplayInterface<DisplayBaseAddress>;
using TheSDInterface = SDCardInterface<MaximumNumberOfOpenFiles, SDBaseAddress>;
using TheConsoleInterface = Serial0Interface<Serial0BaseAddress, CompileInAddressDebuggingSupport, AddressDebuggingEnabledOnStartup>;
using TheRTCInterface = RTCInterface<RTCBaseAddress>;
using ConfigurationSpace = CoreChipsetFeatures<TheConsoleInterface,
        TheSDInterface,
        TheDisplayInterface,
        TheRTCInterface>;






template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T>
class FourWayLRUCacheWay {
public:
    static constexpr auto NumberOfWays = 4;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
public:
    __attribute__((noinline)) CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i].matches(theAddress)) {
                updateFlags(i);
                return ways_[i];
            }
        }
        // find the inverse of the most recently used
        auto index = getLeastRecentlyUsed();
        updateFlags(index);
        ways_[index].reset(theAddress);
        return ways_[index];
    }
    void clear() noexcept {
        for (auto& way : ways_) {
            way.clear();
        }
        flags_ = 0;
    }
private:
    void updateFlags(byte index) noexcept {
        // we have to take the index and current value into account
        // bit 0: top
        // bit 1: left
        // bit 2: right
        static constexpr byte lookup[4] {
                _BV(0),
                _BV(1),
                _BV(2),
                _BV(3),
        };
        flags_ |= lookup[index];
        if (flags_ >= 0xF) {
            flags_ = lookup[index];
        }
    }
    [[nodiscard]] constexpr byte getLeastRecentlyUsed() const noexcept {
        return LRUTable[flags_];
    }
private:
    CacheEntry ways_[NumberOfWays];
    byte flags_ = 0;
    static constexpr byte LRUTable[16] {
            3, 3, 3, 3, 3, 3, 3, 3,
            2, 2, 2, 2,
            1, 1,
            0, 0,
    };

};

template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T>
class EightWayLRUCacheWay {
public:
    static constexpr auto NumberOfWays = 8;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
public:
    __attribute__((noinline)) CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i].matches(theAddress)) {
                updateFlags(i);
                return ways_[i];
            }
        }
        // find the inverse of the most recently used
        auto index = getLeastRecentlyUsed();
        updateFlags(index);
        ways_[index].reset(theAddress);
        return ways_[index];
    }
    void clear() noexcept {
        for (auto& way : ways_) {
            way.clear();
        }
        mruBits_ = 0;
    }
private:
    void updateFlags(byte index) noexcept {
        // cache the lookup
        static constexpr byte lookup[8] {
                _BV(0),
                _BV(1),
                _BV(2),
                _BV(3),
                _BV(4),
                _BV(5),
                _BV(6),
                _BV(7),
        };
        mruBits_ |= lookup[index];
        if (mruBits_ == 0xFF) {
            mruBits_ = lookup[index];
        }
    }
    [[nodiscard]] constexpr byte getLeastRecentlyUsed() const noexcept {
        return LRUTable[mruBits_];
    }
private:
    CacheEntry ways_[NumberOfWays];
    byte mruBits_ = 0;
    static constexpr byte LRUTable[256] {
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,

            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,

            5, 5, 5, 5, 5, 5, 5, 5,
            5, 5, 5, 5, 5, 5, 5, 5,
            5, 5, 5, 5, 5, 5, 5, 5,
            5, 5, 5, 5, 5, 5, 5, 5,

            4, 4, 4, 4, 4, 4, 4, 4,
            4, 4, 4, 4, 4, 4, 4, 4,

            3, 3, 3, 3, 3, 3, 3, 3,
            2, 2, 2, 2,
            1, 1,
            0, 0,
    };
};

template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T>
class SixteenWayLRUCacheWay {
public:
    static constexpr auto NumberOfWays = 16;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
public:
    __attribute__((noinline)) CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i].matches(theAddress)) {
                updateFlags(i);
                return ways_[i];
            }
        }
        // find the inverse of the most recently used
        auto index = getLeastRecentlyUsed();
        updateFlags(index);
        ways_[index].reset(theAddress);
        return ways_[index];
    }
    void clear() noexcept {
        for (auto& way : ways_) {
            way.clear();
        }
        mruBits_.wholeValue_ = 0;
    }
private:
    void updateFlags(byte index) noexcept {
        // cache the lookup
        static constexpr uint16_t lookup[NumberOfWays] {
                _BV(static_cast<uint16_t>(0)),
                _BV(static_cast<uint16_t>(1)),
                _BV(static_cast<uint16_t>(2)),
                _BV(static_cast<uint16_t>(3)),
                _BV(static_cast<uint16_t>(4)),
                _BV(static_cast<uint16_t>(5)),
                _BV(static_cast<uint16_t>(6)),
                _BV(static_cast<uint16_t>(7)),
                _BV(static_cast<uint16_t>(8)),
                _BV(static_cast<uint16_t>(9)),
                _BV(static_cast<uint16_t>(10)),
                _BV(static_cast<uint16_t>(11)),
                _BV(static_cast<uint16_t>(12)),
                _BV(static_cast<uint16_t>(13)),
                _BV(static_cast<uint16_t>(14)),
                _BV(static_cast<uint16_t>(15)),
        };
        mruBits_.wholeValue_ |= lookup[index];
        if (mruBits_.wholeValue_ == 0xFFFF) {
            mruBits_.wholeValue_ = lookup[index];
        }
        if (mruBits_.bytes[1] != 0xFF) {
            nextLRU_ = LRUTable2[mruBits_.bytes[1]]; // add eight to the position
        } else {
            nextLRU_ = LRUTable[mruBits_.bytes[0]];
        }
    }
    [[nodiscard]] constexpr byte getLeastRecentlyUsed() const noexcept {
        return nextLRU_;
    }
private:
    CacheEntry ways_[NumberOfWays];
    SplitWord16 mruBits_ {0};
    byte nextLRU_ = 0;
    static constexpr byte LRUTable[256] {
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,

            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,

            5, 5, 5, 5, 5, 5, 5, 5,
            5, 5, 5, 5, 5, 5, 5, 5,
            5, 5, 5, 5, 5, 5, 5, 5,
            5, 5, 5, 5, 5, 5, 5, 5,

            4, 4, 4, 4, 4, 4, 4, 4,
            4, 4, 4, 4, 4, 4, 4, 4,

            3, 3, 3, 3, 3, 3, 3, 3,
            2, 2, 2, 2,
            1, 1,
            0, 0,
    };
    static constexpr byte LRUTable2[256] {
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,

            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,

            13, 13, 13, 13, 13, 13, 13, 13,
            13, 13, 13, 13, 13, 13, 13, 13,
            13, 13, 13, 13, 13, 13, 13, 13,
            13, 13, 13, 13, 13, 13, 13, 13,

            12, 12, 12, 12, 12, 12, 12, 12,
            12, 12, 12, 12, 12, 12, 12, 12,

            11, 11, 11, 11, 11, 11, 11, 11,
            10, 10, 10, 10,
            9, 9,
            8, 0,
    };
};


template<template<auto, auto, auto, typename> typename C, uint16_t numEntries, byte numAddressBits, byte numOffsetBits, typename T>
class GenericCache {
private:
    using FakeCacheType = C<getNumberOfBitsForNumberOfEntries(numEntries), numAddressBits, numOffsetBits, T>;
public:
    static constexpr auto NumCacheWays = FakeCacheType::NumberOfWays;
    using CacheWay = C<getNumberOfBitsForNumberOfEntries(numEntries/NumCacheWays), numAddressBits, numOffsetBits, T>;
    static constexpr auto WayMask = CacheWay::WayMask;
    static constexpr auto MaximumNumberOfEntries = numEntries;
    using CacheEntry = typename CacheWay::CacheEntry;
    using TaggedAddress = typename CacheWay::TaggedAddress;
public:
    [[nodiscard]] CacheEntry& getLine() noexcept {
        // only align if we need to reset the chip
        TaggedAddress theAddress(ProcessorInterface::getAddress());
        return entries_[theAddress.getTagIndex()].getLine(theAddress);
    }
    void clear() {
        for (auto& a : entries_) {
            a.clear();
        }
    }
    byte* viewAsStorage() noexcept {
        return reinterpret_cast<byte*>(entries_);
    }
    void begin() noexcept {
        clear();
    }
    constexpr auto getCacheSize() const noexcept { return sizeof(entries_); }
private:
    CacheWay entries_[MaximumNumberOfEntries / CacheWay::NumberOfWays];
};
constexpr auto NumAddressBitsForPSRAMCache = 26;
constexpr auto NumAddressBits = NumAddressBitsForPSRAMCache;
constexpr auto NumEntries = 128;
constexpr auto NumOffsetBits = 6;
template<template<auto, auto, auto, typename> typename T>
using Cache_t = GenericCache<T, NumEntries, NumAddressBits, NumOffsetBits, OnboardPSRAMBlock>;
Cache_t<EightWayLRUCacheWay> theCache;


[[nodiscard]] bool informCPU() noexcept {
    // you must scan the BLAST_ pin before pulsing ready, the cpu will change blast for the next transaction
    auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
    pulse<i960Pinout::Ready>();
    return isBurstLast;
}
constexpr auto IncrementAddress = true;
constexpr auto LeaveAddressAlone = false;
// while the i960 does not allow going beyond 8 words, we can use the number of words cached in all cases to be safe
constexpr byte MaximumNumberOfWordsTransferrableInASingleTransaction = decltype(theCache)::CacheEntry::NumWordsCached;
inline void displayRequestedAddress() noexcept {
    auto address = ProcessorInterface::getAddress();
    Serial.print(F("ADDRESS: 0x"));
    Serial.println(address, HEX);
}

template<bool inDebugMode>
inline void fallbackBody() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    // fallback, be consistent to make sure we don't run faster than the i960
    if (ProcessorInterface::isReadOperation()) {
        for (;;) {
            // need to introduce some delay
            ProcessorInterface::setDataBits(0);
            if (informCPU()) {
                break;
            }
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    } else {
        for (;;) {
            // put four cycles worth of delay into this to make damn sure we are ready with the i960
            __builtin_avr_nops(4);
            // need to introduce some delay
            if (informCPU()) {
                break;
            }
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    }
}

template<bool inDebugMode>
inline void handleMemoryInterface() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    // okay we are dealing with the psram chips
    // now take the time to compute the cache offset entries
    if (auto& theEntry = theCache.getLine(); ProcessorInterface::isReadOperation()) {
        // when dealing with read operations, we can actually easily unroll the do while by starting at the cache offset entry and walking
        // forward until we either hit the end of the cache line or blast is asserted first (both are valid states)
        for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
            auto outcome = theEntry.get(i);
            if constexpr (inDebugMode) {
                Serial.print(F("\tOffset: 0x")) ;
                Serial.println(i, HEX);
                Serial.print(F("\tRead the value: 0x"));
                Serial.println(outcome, HEX);
            }
            ProcessorInterface::setDataBits(outcome);
            if (informCPU()) {
                break;
            }
            // so if I don't increment the address, I think we run too fast xD based on some experimentation
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    } else {
        // when dealing with writes to the cache line we are safe in just looping through from the start to at most 8 because that is as
        // far as we can go with how the Sx works!

        // Also the manual states that the processor cannot burst across 16-byte boundaries so :D.
        for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
            auto bits = ProcessorInterface::getDataBits();
            if constexpr (inDebugMode) {
                Serial.print(F("\tOffset: 0x")) ;
                Serial.println(i, HEX);
                Serial.print(F("\tWriting the value: 0x"));
                Serial.println(bits.getWholeValue(), HEX);
            }
            theEntry.set(i, ProcessorInterface::getStyle(), bits);
            if (informCPU()) {
                break;
            }
            // the manual doesn't state that the burst transaction will always have BE0 and BE1 pulled low and this is very true, you must
            // check the pins because it will do unaligned burst transactions but even that will never span multiple 16-byte entries
            // so if I don't increment the address, I think we run too fast xD based on some experimentation
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    }
}

template<bool inDebugMode, typename T>
inline void handleExternalDeviceRequest() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    // with burst transactions in the core chipset, we do not have access to a cache line to write into.
    // instead we need to do the old style infinite iteration design
    if (ProcessorInterface::isReadOperation()) {
        for(;;) {
            auto result = T::read(ProcessorInterface::getPageIndex(),
                                  ProcessorInterface::getPageOffset(),
                                  ProcessorInterface::getStyle());
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x")) ;
                Serial.println(ProcessorInterface::getPageIndex(), HEX);
                Serial.print(F("\tPage Offset: 0x")) ;
                Serial.println(ProcessorInterface::getPageOffset(), HEX);
                Serial.print(F("\tRead Value: 0x"));
                Serial.println(result, HEX);
            }
            ProcessorInterface::setDataBits(result);
            if (informCPU()) {
                break;
            }
            ProcessorInterface::burstNext<IncrementAddress>();
        }
    } else {
        for (;;) {
            auto dataBits = ProcessorInterface::getDataBits();
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x")) ;
                Serial.println(ProcessorInterface::getPageIndex(), HEX);
                Serial.print(F("\tPage Offset: 0x")) ;
                Serial.println(ProcessorInterface::getPageOffset(), HEX);
                Serial.print(F("\tData To Write: 0x"));
                Serial.println(dataBits.getWholeValue(), HEX);
            }
            T::write(ProcessorInterface::getPageIndex(),
                     ProcessorInterface::getPageOffset(),
                     ProcessorInterface::getStyle(),
                     dataBits);
            if (informCPU()) {
                break;
            }
            // be careful of querying i960 state at this point because the chipset runs at twice the frequency of the i960
            // so you may still be reading the previous i960 cycle state!
            ProcessorInterface::burstNext<IncrementAddress>();
        }
    }
}
template<bool inDebugMode>
inline void invocationBody() noexcept {
    // wait until AS goes from low to high
    // then wait until the DEN state is asserted
    while (DigitalPin<i960Pinout::DEN_>::isDeasserted());
    // keep processing data requests until we
    // when we do the transition, record the information we need
    // there are only two parts to this code, either we map into ram or chipset functions
    // we can just check if we are in ram, otherwise it is considered to be chipset. This means that everything not ram is chipset
    // and so we are actually continually mirroring the mapping for the sake of simplicity
    ProcessorInterface::newDataCycle<inDebugMode, decltype(theCache)::CacheEntry::CacheEntryMask>()();
}
template<bool allowAddressDebuggingCodePath>
void doInvocationBody() noexcept {
    if constexpr (allowAddressDebuggingCodePath) {
        if (TheConsoleInterface::addressDebuggingEnabled())  {
            invocationBody<true>();
        } else {
            invocationBody<false>();
        }
    } else {
        invocationBody<false>();
    }
}
void installBootImage() noexcept {

    // okay now we need to actually open boot.system and copy it into the ramBlock
    if (!SD.exists(const_cast<char*>("boot.sys"))) {
        // delete the file and start a new
        signalHaltState(F("Could not find file \"boot.sys\"!"));
    }
    if (auto theFile = SD.open("boot.sys", FILE_READ); !theFile) {
        signalHaltState(F("Could not open \"boot.sys\"! SD CARD may be corrupt?")) ;
    } else {
        // okay we were successful in opening the file, now copy the image into psram
        Address size = theFile.size();
        static constexpr auto CacheSize = theCache.getCacheSize();
        //static_assert(CacheSize >= (TargetBoard::cacheLineSize() * TargetBoard::numberOfCacheLines()), "The entry cache set is smaller than the requested cache size");
        // use the cache as a buffer since it won't be in use at this point in time
        auto* storage = theCache.viewAsStorage();
        Serial.println(F("TRANSFERRING BOOT.SYS TO PSRAM"));
        for (Address addr = 0; addr < size; addr += CacheSize) {
            // do a linear read from the start to the end of storage
            // wait around to make sure we don't run afoul of the sdcard itself
            while (theFile.isBusy());
            auto numRead = theFile.read(storage, CacheSize);
            if (numRead < 0) {
                // something wen't wrong so halt at this point
                SD.errorHalt();
            }
            (void)OnboardPSRAMBlock::write(addr, storage, numRead);
            Serial.print(F("."));
        }
        Serial.println();
        Serial.println(F("Transfer complete!"));
        // make sure we close the file before destruction
        theFile.close();
        theCache.clear();
    }
}

using DispatchTable = BodyFunction[256];
extern DispatchTable lookupTable;
extern DispatchTable lookupTable_Debug;
// the setup routine runs once when you press reset:
void setup() {
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    setupPins(OUTPUT,
              i960Pinout::PSRAM_EN,
              i960Pinout::SD_EN,
              i960Pinout::Reset960,
              i960Pinout::Ready,
              i960Pinout::GPIOSelect,
              i960Pinout::SPI_OFFSET0,
              i960Pinout::SPI_OFFSET1,
              i960Pinout::SPI_OFFSET2,
              i960Pinout::Reset960,
              i960Pinout::Int0_);
    digitalWrite<i960Pinout::Reset960, HIGH>();
    digitalWrite<i960Pinout::Int0_, HIGH>();
    {
        PinAsserter<i960Pinout::Reset960> holdi960InReset;
        // all of these pins need to be pulled high
        digitalWrite<i960Pinout::PSRAM_EN, HIGH>();
        digitalWrite<i960Pinout::SD_EN, HIGH>();
        digitalWrite<i960Pinout::Ready, HIGH>();
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        digitalWrite<i960Pinout::SPI_OFFSET0, LOW>();
        digitalWrite<i960Pinout::SPI_OFFSET1, LOW>();
        digitalWrite<i960Pinout::SPI_OFFSET2, LOW>();
        // setup the pins that could be attached to an io expander separately
        setupPins(INPUT,
                  i960Pinout::BA1,
                  i960Pinout::BA2,
                  i960Pinout::BA3,
                  i960Pinout::BE0,
                  i960Pinout::BE1,
                  i960Pinout::BLAST_,
                  i960Pinout::W_R_,
                  i960Pinout::DEN_,
                  i960Pinout::FAIL);
        //pinMode(i960Pinout::MISO, INPUT_PULLUP);
        theCache.begin();
        SPI.begin();
        // purge the cache pages
        ConfigurationSpace::begin();
        Serial.println(F("i960Sx chipset bringup"));
        {
            Serial.println(F("Setting up the initial lookup table"));
            for (auto& entry : lookupTable) {
                entry = fallbackBody<false>;
            }
            lookupTable[0] = handleMemoryInterface<false>;
            lookupTable[1] = handleMemoryInterface<false>;
            lookupTable[2] = handleMemoryInterface<false>;
            lookupTable[3] = handleMemoryInterface<false>;
            lookupTable[TheRTCInterface ::SectionID] = handleExternalDeviceRequest<false, TheRTCInterface >;
            lookupTable[TheDisplayInterface ::SectionID] = handleExternalDeviceRequest<false, TheDisplayInterface>;
            lookupTable[TheSDInterface ::SectionID] = handleExternalDeviceRequest<false, TheSDInterface>;
            lookupTable[TheConsoleInterface :: SectionID] = handleExternalDeviceRequest<false, TheConsoleInterface >;
            lookupTable[ConfigurationSpace :: SectionID] = handleExternalDeviceRequest<false, ConfigurationSpace >;
            if constexpr (CompileInAddressDebuggingSupport) {
                for (auto &entry: lookupTable_Debug) {
                    entry = fallbackBody<true>;
                }
                lookupTable_Debug[0] = handleMemoryInterface<true>;
                lookupTable_Debug[1] = handleMemoryInterface<true>;
                lookupTable_Debug[2] = handleMemoryInterface<true>;
                lookupTable_Debug[3] = handleMemoryInterface<true>;
                lookupTable_Debug[TheRTCInterface::SectionID] = handleExternalDeviceRequest<true, TheRTCInterface>;
                lookupTable_Debug[TheDisplayInterface::SectionID] = handleExternalDeviceRequest<true, TheDisplayInterface>;
                lookupTable_Debug[TheSDInterface::SectionID] = handleExternalDeviceRequest<true, TheSDInterface>;
                lookupTable_Debug[TheConsoleInterface::SectionID] = handleExternalDeviceRequest<true, TheConsoleInterface>;
                lookupTable_Debug[ConfigurationSpace::SectionID] = handleExternalDeviceRequest<true, ConfigurationSpace>;
            }
        }
        ProcessorInterface::begin();
        OnboardPSRAMBlock::begin();

        installBootImage();
        delay(100);
        Serial.println(F("i960Sx chipset brought up fully!"));

    }
    // at this point we have started execution of the i960
    // wait until we enter self test state
    while (DigitalPin<i960Pinout::FAIL>::isDeasserted()) {
        if (DigitalPin<i960Pinout::DEN_>::isAsserted()) {
            break;
        }
    }

    // now wait until we leave self test state
    while (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
        if (DigitalPin<i960Pinout::DEN_>::isAsserted()) {
            break;
        }
    }
    // at this point we are in idle so we are safe to loaf around a bit
    // at this point, the i960 will request 32-bytes to perform a boot check sum on.
    // If the checksum is successful then execution will continue as normal
    // first set of 16-byte request from memory
    doInvocationBody<CompileInAddressDebuggingSupport>();
    doInvocationBody<CompileInAddressDebuggingSupport>();
    if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
        signalHaltState(F("CHECKSUM FAILURE!"));
    }
    Serial.println(F("SYSTEM BOOT SUCCESSFUL!"));
}
// ----------------------------------------------------------------
// state machine
// ----------------------------------------------------------------
// The bootup process has a separate set of states
// TStart - Where we start
// TSystemTest - Processor performs self test
//
// TStart -> TSystemTest via FAIL being asserted
// TSystemTest -> Ti via FAIL being deasserted
//
// State machine will stay here for the duration
// State diagram based off of i960SA/SB Reference manual
// Basic Bus States
// Ti - Idle State
// Ta - Address State
// Td - Data State
// Tw - Wait State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in

// Ti -> Ti via no request
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Ti after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted
// NOTE: Tw may turn out to be synthetic

void loop() {
    // by default, arduino's main has a boolean check inside of the loop for the serial event run function check
    // this is expensive and something I don't use. The infinite loop is for this purpose. It shaves off around 0.1usec in the worst case
    // and doesn't seem to impact performance in burst transactions

    for (;;) {
        doInvocationBody<CompileInAddressDebuggingSupport>();
    }
}

[[noreturn]]
void
signalHaltState(const __FlashStringHelper* haltMsg) {
    Serial.print(F("CHIPSET HALT: "));
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}

DispatchTable lookupTable;
DispatchTable lookupTable_Debug;

BodyFunction getNonDebugBody(byte index) noexcept {
    return lookupTable[index];
}
BodyFunction getDebugBody(byte index) noexcept {
    return lookupTable_Debug[index];
}


SdFat SD;
