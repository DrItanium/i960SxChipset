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
//
// Created by jwscoggins on 6/21/21.
//

#ifndef I960SXCHIPSET_CORECHIPSETFEATURES_H
#define I960SXCHIPSET_CORECHIPSETFEATURES_H
#include "ProcessorSerializer.h"
#include "SDCardInterface.h"
#include "DisplayInterface.h"
#include "Serial0Interface.h"
template<typename TheConsoleInterface,
         typename TheSDInterface,
         typename TheDisplayInterface,
         typename TheRTCInterface>
class CoreChipsetFeatures /* : public IOSpaceThing */ {
public:
    static constexpr Address IOBaseAddress = 0xFE00'0000;
    static constexpr SplitWord32 IOBaseSplit { IOBaseAddress };
    static constexpr byte SectionID = IOBaseSplit.getMostSignificantByte();
    // each one of these 256 byte pages have a prescribed start and end
    static constexpr Address IOConfigurationSpaceStart = IOBaseAddress;
    static constexpr Address IOConfigurationSpaceEnd = IOConfigurationSpaceStart + (16 * 0x100);
    enum class IOConfigurationSpace0Registers : uint8_t {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)

        FourByteEntry(Serial0BaseAddress),
        FourByteEntry(SDCardInterfaceBaseAddress),
        FourByteEntry(SDCardFileBlock0BaseAddress),
        FourByteEntry(DisplayShieldBaseAddress),
        FourByteEntry(ST7735DisplayBaseAddress),
        FourByteEntry(RTCBaseAddress),
#undef FourByteEntry
#undef TwoByteEntry
        End,
        Serial0BaseAddressLower = Serial0BaseAddress00,
        Serial0BaseAddressUpper = Serial0BaseAddress10,
        SDCardInterfaceBaseAddressLower = SDCardInterfaceBaseAddress00,
        SDCardInterfaceBaseAddressUpper = SDCardInterfaceBaseAddress10,
        SDCardFileBlock0BaseAddressLower = SDCardFileBlock0BaseAddress00,
        SDCardFileBlock0BaseAddressUpper = SDCardFileBlock0BaseAddress10,
        DisplayShieldBaseAddressLower = DisplayShieldBaseAddress00,
        DisplayShieldBaseAddressUpper = DisplayShieldBaseAddress10,
        ST7735DisplayBaseAddressLower = ST7735DisplayBaseAddress00,
        ST7735DisplayBaseAddressUpper = ST7735DisplayBaseAddress10,
        RTCBaseAddressLower = RTCBaseAddress00,
        RTCBaseAddressUpper = RTCBaseAddress10,
    };
private:
    static inline uint32_t ConfigurationSpace_AddressTable [] PROGMEM1 = {
            TheConsoleInterface::StartAddress,
            TheSDInterface::ControlBaseAddress,
            TheSDInterface::FilesBaseAddress,
            TheDisplayInterface::SeesawSectionStart,
            TheDisplayInterface::DisplaySectionStart,
            TheRTCInterface::StartAddress,
    };


public:
    CoreChipsetFeatures() = delete;
    ~CoreChipsetFeatures() = delete;
    CoreChipsetFeatures(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures(CoreChipsetFeatures&&) = delete;
    CoreChipsetFeatures& operator=(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures& operator=(CoreChipsetFeatures&&) = delete;
    static void begin() noexcept {
        // console always comes first
        TheConsoleInterface::begin();
        TheDisplayInterface::begin();
        TheSDInterface::begin();
        TheRTCInterface::begin();
    }
private:

    static uint16_t readIOConfigurationSpace0(uint8_t offset, LoadStoreStyle) noexcept {
        return pgm_read_word_far(pgm_get_far_address(ConfigurationSpace_AddressTable) + offset);
    }

public:
    [[nodiscard]] static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        // force override the default implementation
        if (targetPage == 0) {
            return readIOConfigurationSpace0(offset, lss);
        } else {
            return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        // do nothing
    }
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
