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

#include "IOExpanders.h"

IOExpander<IOExpanderAddress::DataLines> dataLines;
IOExpander<IOExpanderAddress::Lower16Lines> lower16;
IOExpander<IOExpanderAddress::Upper16Lines> upper16;
IOExpander<IOExpanderAddress::MemoryCommitExtras> extraMemoryCommit;
Address
getAddress() noexcept {
    auto lower16Addr = static_cast<Address>(lower16.readGPIOs());
    auto upper16Addr = static_cast<Address>(upper16.readGPIOs()) << 16;
    return lower16Addr | upper16Addr;
}
uint16_t
getDataBits() noexcept {
    dataLines.writeGPIOsDirection(0xFFFF);
    return static_cast<uint16_t>(dataLines.readGPIOs());
}

void
setDataBits(uint16_t value) noexcept {
    dataLines.writeGPIOsDirection(0);
    dataLines.writeGPIOs(value);
}



uint8_t getByteEnableBits() noexcept {
    return (extraMemoryCommit.readGPIOs() & 0b11000) >> 3;
}

decltype(LOW) getByteEnable0() noexcept {
    return (getByteEnableBits() & 1) == 0 ? LOW : HIGH;
}
decltype(LOW) getByteEnable1() noexcept {
    return (getByteEnableBits() & 0b10) == 0 ? LOW : HIGH;
}

uint8_t getBurstAddressBits() noexcept {
    return (extraMemoryCommit.readGPIOs() & 0b111) << 1;
}

Address getBurstAddress(Address base) noexcept {
    return getBurstAddress(base, static_cast<Address>(getBurstAddressBits()));
}
Address getBurstAddress() noexcept {
    return getBurstAddress(getAddress());
}

bool isReadOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isAsserted(); }
bool isWriteOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isDeasserted(); }
decltype(LOW) getBlastPin() noexcept { return DigitalPin<i960Pinout::BLAST_>::read(); }
void setHOLDPin(decltype(LOW) value) noexcept {
    digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), value, extraMemoryCommit);
}
void setLOCKPin(decltype(LOW) value) noexcept {
    digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), value, extraMemoryCommit);
}
