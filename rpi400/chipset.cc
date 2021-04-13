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
#include <cstdint>
#include <memory>
#include <boost/program_options.hpp>
using Short = uint16_t;
using Ordinal = uint32_t;
using Address = Ordinal;
/**
 * @brief A single memory cell exposed to the i960Sx processor
 */
union DataCell {
   uint8_t bytes[sizeof(Short)] ;
   Short word;
};

constexpr auto RAMSizeInBytes = 1024u * 1024u * 1024u;
constexpr auto RAMSizeInDataCells = RAMSizeInBytes / sizeof(Short);
std::unique_ptr<DataCell[]> ram;
void setupRAM() {
    ram = std::make_unique<DataCell[]>(RAMSizeInDataCells);
    for (auto i = 0u; i < RAMSizeInDataCells; ++i) {
        ram[i].word = 0;
    }
}
constexpr Address RAMStart = 0x8000'0000;
constexpr Address RAMMax = RAMStart + RAMSizeInBytes;
static_assert(RAMMax == 0xC000'0000);
struct Upper8Bits { };
struct Lower8Bits { };
struct Full16Bits { };
constexpr Address performRAMMask(Address value) noexcept {
    return (~RAMMax) & value;
}
static_assert(performRAMMask(0x8000'0000) == 0);
static_assert(performRAMMask(0x8000'FDED) == 0xFDED);
static_assert(performRAMMask(0x9000'0000) == 0x1000'0000);
static_assert(performRAMMask(0xA000'0000) == 0x2000'0000);
uint8_t loadRAM(Address value, Upper8Bits) noexcept { return ram[performRAMMask(value)].bytes[1]; }
uint8_t loadRAM(Address value, Lower8Bits) noexcept { return ram[performRAMMask(value)].bytes[0]; }
uint16_t loadRAM(Address value, Full16Bits) noexcept { return ram[performRAMMask(value)].word; }
void storeRAM(Address addr, uint8_t value, Upper8Bits) noexcept { ram[performRAMMask(addr)].bytes[1] = value; }
void storeRAM(Address addr, uint8_t value, Lower8Bits) noexcept { ram[performRAMMask(addr)].bytes[0] = value; }
void storeRAM(Address addr, uint16_t value, Full16Bits) noexcept { ram[performRAMMask(addr)].word = value; }
int main(int argc, char** argv) {
    setupRAM();
    return 0;
}
