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

//#ifndef SXCHIPSET_DMACHANNEL_H
//#define SXCHIPSET_DMACHANNEL_H
//#include <Arduino.h>
///**
// * @brief Provides a DMA transfer channel accessible by the i960
// */
//class DMAChannel {
//    public:
//        void claim(uint32_t src, uint32_t dest, uint32_t count, bool advanceSrc, bool advanceDest) noexcept;
//        void step() noexcept;
//        void cancel() noexcept;
//        void pause() noexcept;
//        void unpause() noexcept;
//        [[nodiscard]] constexpr bool paused() const noexcept { return paused_; }
//        [[nodiscard]] constexpr bool finished() const noexcept { return finished_; }
//        [[nodiscard]] constexpr auto getSourceAddress() const noexcept { return srcAddress_; }
//        [[nodiscard]] constexpr auto getDestinationAddress() const noexcept { return destAddress_; }
//        [[nodiscard]] constexpr auto getCount() const noexcept { return count_; }
//        [[nodiscard]] constexpr bool available() const noexcept { return free_; }
//        [[nodiscard]] constexpr bool incrementSrcOnStep() const noexcept { return advanceSrc_; }
//        [[nodiscard]] constexpr bool incrementDestOnStep() const noexcept { return advanceSrc_; }
//    private:
//        uint32_t srcAddress_ = 0;
//        uint32_t destAddress_ = 0;
//        uint32_t count_ = 0;
//        bool free_ = true;
//        bool finished_ = false;
//        bool advanceSrc_ = true;
//        bool advanceDest_ = true;
//        bool paused_ = false;
//};
//#endif //SXCHIPSET_DMACHANNEL_H
#include "DMAChannel.h"

void
DMAChannel::claim(uint32_t src, uint32_t dest, uint32_t count, bool advanceSrc, bool advanceDest) noexcept {

}

void
DMAChannel::step() noexcept {

}

void
DMAChannel::cancel() noexcept {
}

void
DMAChannel::pause() noexcept {
    paused_ = true;
}
void
DMAChannel::unpause() noexcept {
    paused_ = false;
}
