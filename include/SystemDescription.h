/*
i960SxChipset
Copyright (c) 2020-2022, Joshua Scoggins
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
//
// Created by jwscoggins on 3/5/22.
//

#ifndef SXCHIPSET_SYSTEMDESCRIPTION_H
#define SXCHIPSET_SYSTEMDESCRIPTION_H
#include "ConfigurationFlags.h"
#include "MCUPlatform.h"

#include "CacheEntry.h"
#include "DirectMappedCacheWay.h"
#include "TwoWayLRUCacheEntry.h"
#include "FourWayPseudoLRUEntry.h"
#include "EightWayPseudoLRUEntry.h"
#include "SixteenWayPseudoLRUEntry.h"
#include "EightWayRandPLRUEntry.h"
#include "EightWayTreePLRUEntry.h"
#include "SinglePoolCache.h"
#include "DisplayInterface.h"
#include "CoreChipsetFeatures.h"
#include "PSRAMChip.h"
#include "SDCardAsRam.h"
#include "TaggedCacheAddress.h"
#include "RTCInterface.h"

constexpr auto RTCBaseAddress = 0xFA00'0000;
constexpr auto Serial0BaseAddress = 0xFB00'0000;
constexpr auto DisplayBaseAddress = 0xFC00'0000;
constexpr auto SDBaseAddress = 0xFD00'0000;
constexpr auto MaximumNumberOfOpenFiles = 16;

using TheDisplayInterface = DisplayInterface<DisplayBaseAddress>;
using TheSDInterface = SDCardInterface<MaximumNumberOfOpenFiles, SDBaseAddress>;
using TheConsoleInterface = Serial0Interface<Serial0BaseAddress, CompileInAddressDebuggingSupport, AddressDebuggingEnabledOnStartup>;
using TheRTCInterface = RTCInterface<RTCBaseAddress>;
using ConfigurationSpace = CoreChipsetFeatures<TheConsoleInterface,
        TheSDInterface,
        TheDisplayInterface,
        TheRTCInterface>;
// define the backing memory storage classes via template specialization
// at this point in time, if no specialization is performed, use SDCard as ram backend
using FallbackMemory = SDCardAsRam<TheSDInterface >;
template<TargetMCU mcu> struct BackingMemoryStorage final {
    using Type = FallbackMemory;
};
template<> struct BackingMemoryStorage<TargetMCU::ATmega1284p_Type1> final {
    //using ActualType = OnboardPSRAMBlock;
    //using Type = SRAMDataContainer<ActualType>;
    using Type = OnboardPSRAMBlock;
};

using BackingMemoryStorage_t = BackingMemoryStorage<TargetBoard::getMCUTarget()>::Type;
constexpr auto NumAddressBitsForPSRAMCache = 26;
constexpr auto NumAddressBits = NumAddressBitsForPSRAMCache;
constexpr auto CacheLineSize = 6;
constexpr auto CacheSize = 8192;
constexpr auto UseSpecificTypesForDifferentAddressComponents = true;

using L1Cache = CacheInstance_t<EightWayRandPLRUCacheSet, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t, UseSpecificTypesForDifferentAddressComponents>;
extern L1Cache theCache;

#endif //SXCHIPSET_SYSTEMDESCRIPTION_H
