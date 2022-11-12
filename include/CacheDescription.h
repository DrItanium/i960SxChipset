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
#include "EightWayPseudoLRUEntry.h"
#include "SixteenWayPseudoLRUEntry.h"
#include "EightWayRandPLRUEntry.h"
#include "EightWayTreePLRUEntry.h"
#include "TenWayRandPLRUEntry.h"
#include "TwelveWayRandPLRUEntry.h"
#include "FourteenWayRandPLRUEntry.h"
#include "RoundRobinCacheSet.h"
#include "SinglePoolCache.h"
#include "PSRAMChip.h"
#include "DualPSRAMPool.h"


//using BackingMemoryStorage_t = DualPoolPSRAM;
using BackingMemoryStorage_t = OnboardPSRAMBlock_Pool2;

//using Cache1Config = CacheInstance_t<EightWayRandPLRUCacheSet, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t, UseSpecificTypesForDifferentAddressComponents>;
//using Cache1Config = CacheInstance_t<FourWayLRUCacheWay, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t, UseSpecificTypesForDifferentAddressComponents>;
using Cache1Config = CacheInstance_t<FourWayRoundRobinCacheWay, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t, UseSpecificTypesForDifferentAddressComponents>;
//constexpr auto NumberOfEntries = 128;
//using Cache1Config = Cache2Instance_t<FiveWayRoundRobinCacheWay, NumberOfEntries, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
// unlike normal caches, we have to tune the number of entries based on available ram
using L1Cache = Cache1Config;
using CacheLine = L1Cache::CacheEntry;
extern L1Cache theCache;
#endif //SXCHIPSET_CACHEDESCRIPTION_H
