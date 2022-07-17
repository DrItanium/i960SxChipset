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

#ifndef SXCHIPSET_CACHEDESCRIPTION_H
#define SXCHIPSET_CACHEDESCRIPTION_H
#include "ConfigurationFlags.h"
#include "CacheEntry.h"
#include "DirectMappedCacheWay.h"
#include "TwoWayLRUCacheEntry.h"
#include "FourWayPseudoLRUEntry.h"
#include "HackedFourWayPseudoLRUEntry.h"
#include "EightWayPseudoLRUEntry.h"
#include "SixteenWayPseudoLRUEntry.h"
#include "EightWayRandPLRUEntry.h"
#include "EightWayTreePLRUEntry.h"
#include "TenWayRandPLRUEntry.h"
#include "TwelveWayRandPLRUEntry.h"
#include "FourteenWayRandPLRUEntry.h"
#include "SinglePoolCache.h"
#include "SDCardAsRam.h"
#include "PSRAMChip.h"
#include "DualPSRAMPool.h"

// define the backing memory storage classes via template specialization
// at this point in time, if no specialization is performed, use SDCard as ram backend
template<TargetMCU mcu> struct BackingMemoryStorage final {
    using Type = FallbackMemory;
};
template<> struct BackingMemoryStorage<TargetMCU::ATmega1284p_Type1> final {
    //using ActualType = OnboardPSRAMBlock;
    //using Type = SRAMDataContainer<ActualType>;
    //using Type = OnboardPSRAMBlock;
    //using Type = OnboardPSRAMBlock_Pool2;
    using Type = DualPoolMemoryBlock;
};

using BackingMemoryStorage_t = BackingMemoryStorage<TargetBoard::getMCUTarget()>::Type;

//using Cache1Config = CacheInstance_t<EightWayRandPLRUCacheSet, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t, UseSpecificTypesForDifferentAddressComponents>;
using Cache1Config = CacheInstance_t<HackedFourWayLRUCacheWay, CacheSize, NumAddressBits, 4, BackingMemoryStorage_t, UseSpecificTypesForDifferentAddressComponents>;
// unlike normal caches, we have to tune the number of entries based on available ram
constexpr auto NumberCache2Entries = 16;
using Cache2Config = Cache2Instance_t<TenWayRandPLRUCacheSet, NumberCache2Entries, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
using L1Cache = Cache1Config;
using CacheLine = L1Cache::CacheEntry;
extern L1Cache theCache;
#endif //SXCHIPSET_CACHEDESCRIPTION_H
