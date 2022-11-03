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
// Created by jwscoggins on 8/28/22.
//

#ifndef SXCHIPSET_EBI_H
#define SXCHIPSET_EBI_H
#include <Arduino.h>
#include "MCUPlatform.h"
namespace EBI
{

    template<typename T>
    inline volatile T& getRegister() noexcept { return memory<T>(T::Address); }
    enum class Register8BitIndex : uint8_t {
        Bit0,
        Bit1,
        Bit2,
        Bit3,
        Bit4,
        Bit5,
        Bit6,
        Bit7,
    };
    constexpr uint8_t getMask(Register8BitIndex index) noexcept {
        return static_cast<uint8_t>(1u << static_cast<uint8_t>(index));
    }
    union GenericRegister8bit {
        uint8_t full;
        struct {
            uint8_t b0 : 1;
            uint8_t b1 : 1;
            uint8_t b2 : 1;
            uint8_t b3 : 1;
            uint8_t b4 : 1;
            uint8_t b5 : 1;
            uint8_t b6 : 1;
            uint8_t b7 : 1;
        } bits;
    };
    template<auto pin, size_t registerAddress, Register8BitIndex index>
    struct BackingDigitalPin {
        static constexpr auto Offset = index;
        static constexpr auto Mask = getMask(Offset);
        BackingDigitalPin() = delete;
        ~BackingDigitalPin() = delete;
        BackingDigitalPin(const BackingDigitalPin&) = delete;
        BackingDigitalPin(BackingDigitalPin&&) = delete;
        BackingDigitalPin& operator=(const BackingDigitalPin&) = delete;
        BackingDigitalPin& operator=(BackingDigitalPin&&) = delete;
        static decltype(HIGH) read() noexcept {
            switch (Offset) {
                case Register8BitIndex::Bit0: return memory<GenericRegister8bit>(registerAddress).bits.b0;
                case Register8BitIndex::Bit1: return memory<GenericRegister8bit>(registerAddress).bits.b1;
                case Register8BitIndex::Bit2: return memory<GenericRegister8bit>(registerAddress).bits.b2;
                case Register8BitIndex::Bit3: return memory<GenericRegister8bit>(registerAddress).bits.b3;
                case Register8BitIndex::Bit4: return memory<GenericRegister8bit>(registerAddress).bits.b4;
                case Register8BitIndex::Bit5: return memory<GenericRegister8bit>(registerAddress).bits.b5;
                case Register8BitIndex::Bit6: return memory<GenericRegister8bit>(registerAddress).bits.b6;
                case Register8BitIndex::Bit7: return memory<GenericRegister8bit>(registerAddress).bits.b7;
                default: return LOW;
            }
        }
        static void write(decltype(HIGH) value) noexcept {
            switch (Offset) {
                case Register8BitIndex::Bit0: memory<GenericRegister8bit>(registerAddress).bits.b0 = value; break;
                case Register8BitIndex::Bit1: memory<GenericRegister8bit>(registerAddress).bits.b1 = value; break;
                case Register8BitIndex::Bit2: memory<GenericRegister8bit>(registerAddress).bits.b2 = value; break;
                case Register8BitIndex::Bit3: memory<GenericRegister8bit>(registerAddress).bits.b3 = value; break;
                case Register8BitIndex::Bit4: memory<GenericRegister8bit>(registerAddress).bits.b4 = value; break;
                case Register8BitIndex::Bit5: memory<GenericRegister8bit>(registerAddress).bits.b5 = value; break;
                case Register8BitIndex::Bit6: memory<GenericRegister8bit>(registerAddress).bits.b6 = value; break;
                case Register8BitIndex::Bit7: memory<GenericRegister8bit>(registerAddress).bits.b7 = value; break;
                default: break;
            }
        }
        template<decltype(HIGH) value>
        static void write() noexcept { write(value); }
        static void configure(decltype(INPUT) = INPUT) { }
        static constexpr bool isInputPin() noexcept { return true; }
        static constexpr bool isOutputPin() noexcept { return true; }
        static constexpr decltype(INPUT) mode() noexcept { return OUTPUT; }
        static constexpr auto getPin() noexcept { return pin; }
        static constexpr auto valid() noexcept { return false; }
        template<decltype(HIGH) to = LOW>
        [[gnu::always_inline]] static void pulse() {
            write<to>();
            write<to == LOW ? HIGH : LOW>();
        }
    };
} // end namespace EBI
#endif //SXCHIPSET_EBI_H
