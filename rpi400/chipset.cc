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
}
int main(int argc, char** argv) {
    setupRAM();
    for (auto i = 0u; i < RAMSizeInDataCells; ++i) {
        ram[i].word = 0;
    }
    return 0;
}
