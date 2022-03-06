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

#ifndef SXCHIPSET_CONFIGURATIONFLAGS_H
#define SXCHIPSET_CONFIGURATIONFLAGS_H

constexpr auto CompileInAddressDebuggingSupport = false;
constexpr auto AddressDebuggingEnabledOnStartup = false;
constexpr auto ValidateTransferDuringInstall = false;
/**
 * @brief When set to true, the interrupt lines the mcp23s17 provides are used to determine which bytes to read
 */
constexpr auto UseIOExpanderAddressLineInterrupts = true;
/**
 * @brief When set to true, the ram and io space pins will be queried when getting the function associated with a given address block. This will
 * allow a reduction in sram usage at the cost of some latency (128 total entries [256 with debug active]). When false, more ram will be used
 * to store a separate function for each 16 megabyte section of the space. It will be slightly faster too. However, most of the sram used by the
 * lookup table will go to waste.
 */
constexpr auto UseSpacePins = true;

/**
 * @brief Set to true (if safe) to allow bitfields to be the exact width needed instead of the integral type. This is safe on AVR and nowhere els
 */
constexpr auto UseSpecificTypesForDifferentAddressComponents = true;
/**
 * @brief When false, all chipset boot information over serial console (except for the ram upload) will be suppressed
 */
constexpr auto DisplayBootupInformation  = true;
/**
 * @brief If true, then the onboard flash chips are selected to load boot.sys from, otherwise the sdcard is used
 */
constexpr auto UploadBootImageFromFlash = false;
/**
 * @brief Maximum number of concurrently open files allowed, increasing this number will increase sram usage
 */
constexpr auto MaximumNumberOfOpenFiles = 16;

constexpr auto RTCBaseAddress = 0xFA00'0000;
constexpr auto Serial0BaseAddress = 0xFB00'0000;
constexpr auto DisplayBaseAddress = 0xFC00'0000;
constexpr auto SDBaseAddress = 0xFD00'0000;

// Cache configuration fields
/**
 * @brief Total number of bits used for cache address resolution. Reduction in bit count on AVR can translate to improved code density. Everywhere else this really does nothing
 */
constexpr auto NumAddressBitsForPSRAMCache = 27;
/**
 * @brief The total number of bits used for addresses overall, usually the same as NumAddressBitsForPSRAMCache
 */
constexpr auto NumAddressBits = NumAddressBitsForPSRAMCache;
/**
 * @brief Number of bits used to represent cache line size, defaults to 6 which translates to 64 bytes
 */
constexpr auto CacheLineSize = 6;
/**
 * @brief Total number of bytes that make up the cache itself (compiler will carve this up automatically based on configuration parameters provided)
 */
constexpr auto CacheSize = 8192;

#endif //SXCHIPSET_CONFIGURATIONFLAGS_H
