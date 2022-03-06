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

#include "ProcessorSerializer.h"
#include "SystemDescription.h"
void
ProcessorInterface::readCacheLine_NonDebug() noexcept {
    performCacheRead<typename L1Cache::CacheEntry, false>(theCache.getLine());
}
void
ProcessorInterface::readCacheLine_Debug() noexcept {
    performCacheRead<typename L1Cache::CacheEntry, true>(theCache.getLine());
}
void
ProcessorInterface::writeCacheLine_NonDebug() noexcept {
    performCacheWrite<typename L1Cache::CacheEntry, false>(theCache.getLine());
}
void
ProcessorInterface::writeCacheLine_Debug() noexcept {
    performCacheWrite<typename L1Cache::CacheEntry, true>(theCache.getLine());
}
void
ProcessorInterface::setupDispatchTable() noexcept {
    Serial.println(F("Setting up the initial lookup table"));
    if constexpr (UseSpacePins) {
        for (int i = 0; i < 32; ++i) {
            ramSectionRead_[i] = performFallbackRead;
            ramSectionWrite_[i] = performFallbackWrite;
            ioSectionRead_[i] = performFallbackRead;
            ioSectionWrite_[i] = performFallbackWrite;
            if constexpr (CompileInAddressDebuggingSupport) {
                ramSectionRead_Debug_[i] = performFallbackRead;
                ramSectionWrite_Debug_[i] = performFallbackWrite;
                ioSectionRead_Debug_[i] = performFallbackRead;
                ioSectionWrite_Debug_[i] = performFallbackWrite;
            }
        }
        for (int i = 0;i < 4; ++i) {
            ramSectionRead_[i] = readCacheLine<false>;
            ramSectionWrite_[i] = writeCacheLine<false>;
            if constexpr (CompileInAddressDebuggingSupport) {
                ramSectionRead_Debug_[i] = readCacheLine<true>;
                ramSectionWrite_Debug_[i] = writeCacheLine<true>;
            }
        }
    } else {
        for (int i = 0; i < 256; ++i) {
            lookupTableRead[i] = performFallbackRead;
            lookupTableWrite[i] = performFallbackWrite;
            if constexpr (CompileInAddressDebuggingSupport) {
                lookupTableRead_Debug[i] = performFallbackRead;
                lookupTableWrite_Debug[i] = performFallbackWrite;
            }
        }
        for (int i = 0;i < 4; ++i) {
            lookupTableRead[i] = readCacheLine<false>;
            lookupTableWrite[i] = writeCacheLine<false>;
            if constexpr (CompileInAddressDebuggingSupport) {
                lookupTableRead_Debug[i] = readCacheLine<true>;
                lookupTableWrite_Debug[i] = writeCacheLine<true>;
            }
        }
    }
    registerExternalDeviceWithLookupTable<TheRTCInterface>();
    registerExternalDeviceWithLookupTable<TheDisplayInterface>();
    registerExternalDeviceWithLookupTable<TheSDInterface >();
    registerExternalDeviceWithLookupTable<TheConsoleInterface>();
    registerExternalDeviceWithLookupTable<ConfigurationSpace>();
    // now tell the ProcessorInterface to pull the appropriate functions
    setupMostRecentDispatchFunctions();
}

BodyFunction
ProcessorInterface::getDebugReadBody(byte index) noexcept {
    if constexpr (CompileInAddressDebuggingSupport) {
        if constexpr (UseSpacePins) {
            if (inIOSpace())  {
                return ioSectionRead_Debug_[index & 0b0001'1111];
            } else if (inRAMSpace()) {
                return ramSectionRead_Debug_[index & 0b0001'1111];
            } else {
                return ProcessorInterface::performFallbackRead;
            }
        } else {
            return lookupTableRead_Debug[index];
        }
    } else {
        return ProcessorInterface::performFallbackRead;
    }
}
BodyFunction
ProcessorInterface::getDebugWriteBody(byte index) noexcept {
    if constexpr (CompileInAddressDebuggingSupport) {
        if constexpr (UseSpacePins) {
            if (inIOSpace())  {
                return ioSectionWrite_Debug_[index & 0b0001'1111];
            } else if (inRAMSpace()) {
                return ramSectionWrite_Debug_[index & 0b0001'1111];
            } else {
                return ProcessorInterface::performFallbackWrite;
            }
        } else {
            return lookupTableWrite_Debug[index];
        }
    } else {
        return ProcessorInterface::performFallbackWrite;
    }
}
BodyFunction
ProcessorInterface::getNonDebugReadBody(byte index) noexcept {
    if constexpr (UseSpacePins) {
        if (inIOSpace()) {
            return ioSectionRead_[index & 0b0001'1111];
        } else if (inRAMSpace()) {
            return ramSectionRead_[index & 0b0001'1111];
        } else {
            return ProcessorInterface::performFallbackRead;
        }
    } else {
        return lookupTableRead[index];
    }
}
BodyFunction
ProcessorInterface::getNonDebugWriteBody(byte index) noexcept {
    if constexpr (UseSpacePins) {
        if (inIOSpace()) {
            return ioSectionWrite_[index & 0b0001'1111];
        } else if (inRAMSpace()) {
            return ramSectionWrite_[index & 0b0001'1111];
        } else {
            return ProcessorInterface::performFallbackWrite;
        }
    } else {
        return lookupTableWrite[index];
    }
}
