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
#include "CacheDescription.h"
void
ProcessorInterface::setupDispatchTable() noexcept {
    static_assert(BackingMemoryStorage_t::NumSections == 32, "Must have a 512 megabyte ram view");
    for (int i = 0; i < 32; ++i) {
        ioSectionRead_[i] = performFallbackRead;
        ioSectionWrite_[i] = performFallbackWrite;
        if constexpr (CompileInAddressDebuggingSupport) {
            ioSectionRead_Debug_[i] = performFallbackRead;
            ioSectionWrite_Debug_[i] = performFallbackWrite;
        }
    }
    registerExternalDeviceWithLookupTable<TheRTCInterface>();
    registerExternalDeviceWithLookupTable<TheDisplayInterface>();
    registerExternalDeviceWithLookupTable<TheSDInterface >();
    registerExternalDeviceWithLookupTable<TheConsoleInterface>();
    registerExternalDeviceWithLookupTable<ConfigurationSpace>();
    // now tell the ProcessorInterface to pull the appropriate functions
}

