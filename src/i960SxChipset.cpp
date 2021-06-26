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

/// i960Sx chipset mcu, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include <SPI.h>
#include <Fsm.h>
#include <SdFat.h>

#include "Pinout.h"

#include "ProcessorSerializer.h"
#include "Device.h"
#include "MemoryMappedFileThing.h"
#include "SDCardFileSystemInterface.h"
#include "CoreChipsetFeatures.h"
#include "TFTDisplayThing.h"


bool displayReady = false;
/**
 * @brief Describes a single cache line which associates an address with 16 bytes of storage
 */
ProcessorInterface& processorInterface = ProcessorInterface::getInterface();
// ----------------------------------------------------------------
// Load/Store routines
// ----------------------------------------------------------------

CoreChipsetFeatures chipsetFunctions(0);



class RAMFile : public MemoryMappedFile {
public:
    static constexpr Address MaxRamSize = 32 * 0x0100'0000; // 32 Memory Spaces or 512 Megabytes
    static constexpr auto RamMask = MaxRamSize - 1;
    static constexpr Address RamStartingAddress = 0x8000'0000; // start this at 512 megabytes
    static constexpr auto RamEndingAddress = RamStartingAddress + MaxRamSize;
    using Parent = MemoryMappedFile;
    RAMFile() noexcept : Parent(RamStartingAddress, RamEndingAddress, MaxRamSize, "ram.bin", FILE_WRITE) { }
    ~RAMFile() override = default;
    [[nodiscard]] Address
    makeAddressRelative(Address input) const noexcept override {
        // in this case, we want relative offsets
        return input & RamMask;
    }
    void begin() noexcept override {
        Parent::begin();
        if (Parent::getFileSize() != MaxRamSize) {
            signalHaltState(F("RAM.BIN MUST BE 512 MEGS IN SIZE!"));
        }
    }
};

class ROMTextSection : public MemoryMappedFile {
public:
    static constexpr Address ROMStart = 0;
    static constexpr Address ROMEnd = 0x2000'0000;
    static constexpr Address ROMMask = ROMEnd - 1;
    using Parent = MemoryMappedFile;
public:
    ROMTextSection() noexcept : Parent(ROMStart, ROMEnd, ROMEnd - 1, "boot.rom", FILE_READ){ }
    ~ROMTextSection() override = default;
    [[nodiscard]] Address
    makeAddressRelative(Address input) const noexcept override {
        return input & ROMMask;
    }
    [[nodiscard]] bool
    respondsTo(Address address) const noexcept override {
        return address < Parent::getFileSize();
    }
    using Device::respondsTo;
};

/// @todo add support for the boot data section that needs to be copied into ram by the i960 on bootup
class ROMDataSection : public MemoryMappedFile {
public:
    // two clusters are held onto at a time
    static constexpr Address ROMStart = 0x2000'0000;
    static constexpr Address ROMEnd = 0x8000'0000;
    static constexpr Address DataSizeMax = ROMEnd - ROMStart;
    using Parent = MemoryMappedFile;
public:
    ROMDataSection() noexcept : Parent(ROMStart, ROMEnd, DataSizeMax, "boot.dat", FILE_READ) { }
    ~ROMDataSection() override = default;
};

TFTDisplayThing displayCommandSet(0x200);
// we want to only put a data cache on the text section
RAMFile ram;
ROMTextSection rom;
ROMDataSection dataRom;

SDCardFilesystemInterface fs(0x300);

// list of io memory devices to walk through
Device* things[] {
        &rom,
        &dataRom,
        &ram,
        &chipsetFunctions,
        &displayCommandSet,
        &fs,
};

Device*
getThing(Address address, LoadStoreStyle style) noexcept {
    for (auto* currentThing : things) {
        if (currentThing->respondsTo(address, style)) {
            return currentThing;
        }
    }
    return nullptr;
}

// ----------------------------------------------------------------
// state machine
// ----------------------------------------------------------------
// The bootup process has a separate set of states
// TStart - Where we start
// TSystemTest - Processor performs self test
//
// TStart -> TSystemTest via FAIL being asserted
// TSystemTest -> Ti via FAIL being deasserted
// 
// State machine will stay here for the duration
// State diagram based off of i960SA/SB Reference manual
// Basic Bus States
// Ti - Idle State
// Ta - Address State
// Td - Data State
// TChecksumFailure - Checksum Failure State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in 

// Ti -> Ti via no request
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Ti after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted

// We are adding extra states to the design when dealing with burst transactions on burst address aware devices
//
// Tbr -> Burst Read Action (supportsBlockTransfers = true)
// Tbw -> Burst Write Action (supportsBlockTransfers = true)
// Tcbr -> Cyclic Burst Read (supportsBlockTransfers = false)
// Tcbw -> Cyclic Burst Write (supportsBlockTransfers = false)
// Tnbr -> Non Burst Read Action
// Tnbw -> Non Burst Write Action
// Tur -> Unmapped Read Action
// Tuw -> Unmapped Write Action
// Td -> Tbr  via ToBurstReadTransaction
// Td -> Tbw  via ToBurstWriteTransaction
// Td -> Tnbr via ToNonBurstReadTransaction
// Td -> Tnbw via ToNonBurstWriteTransaction
// Td -> Tur via ToUnmappedReadTransaction
// Td -> Tuw via ToUnmappedWriteTransaction
// all of these new states transition to tR via ToBusRecovery
#ifdef ARDUINO_ARCH_SAMD
Adafruit_ZeroTimer burstTransactionTimer(3); // I'm not going to be using tone on the grand central
#endif
SplitWord16 burstCache[16 / sizeof(SplitWord16)] = { { 0 }  };
constexpr auto NoRequest = 0;
constexpr auto NewRequest = 1;
constexpr auto ReadyAndBurst = 2;
constexpr auto NotReady = 3;
constexpr auto ReadyAndNoBurst = 4;
constexpr auto RequestPending = 5;
constexpr auto ToDataState = 6;
constexpr auto PerformSelfTest = 7;
constexpr auto SelfTestComplete = 8;
constexpr auto ChecksumFailure = 9;
constexpr auto ToBurstReadTransaction = 10;
constexpr auto ToBurstWriteTransaction = 11;
constexpr auto ToNonBurstReadTransaction = 12;
constexpr auto ToNonBurstWriteTransaction = 13;
constexpr auto ToUnmappedReadTransaction = 14;
constexpr auto ToUnmappedWriteTransaction = 15;
constexpr auto ToBusRecovery = 16;
constexpr auto ToCyclicBurstReadTransaction = 17;
constexpr auto ToCyclicBurstWriteTransaction= 18;
constexpr auto ToCommitBurstTransaction = 19;
void startupState() noexcept;
void systemTestState() noexcept;
void idleState() noexcept;
void doAddressState() noexcept;
[[maybe_unused]] void processDataRequest() noexcept;
void dataCycleStart() noexcept;
[[maybe_unused]] void doRecoveryState() noexcept;
void enteringChecksumFailure() noexcept;
void performBurstRead() noexcept;
void performBurstWrite() noexcept;
void performNonBurstRead() noexcept;
void performNonBurstWrite() noexcept;
void unmappedRead() noexcept;
void unmappedWrite() noexcept;
void performCyclicBurstRead() noexcept;
void performCyclicBurstWrite() noexcept;
void commitBurstTransaction() noexcept;
State tStart(nullptr, startupState, nullptr);
State tSystemTest(nullptr, systemTestState, nullptr);
Fsm fsm(&tStart);
State tIdle(nullptr,
            idleState,
            nullptr);
State tAddr([]() { processorInterface.clearASTrigger(); },
            doAddressState,
            nullptr);
State tData(nullptr,
            dataCycleStart,
            nullptr);
#if 0
State tRecovery(nullptr,
                doRecoveryState,
                nullptr);
#endif
State tBurstRead(nullptr, performBurstRead, nullptr);
State tBurstWrite(nullptr, performBurstWrite, nullptr);
State tNonBurstRead(nullptr, performNonBurstRead, nullptr);
State tNonBurstWrite(nullptr, performNonBurstWrite, nullptr);
State tUnmappedRead(nullptr, unmappedRead, nullptr);
State tUnmappedWrite(nullptr, unmappedWrite, nullptr);

State tCyclicBurstRead(nullptr, performCyclicBurstRead, nullptr);
State tCyclicBurstWrite(nullptr, performCyclicBurstWrite, nullptr);
State tCommitBurstTransaction(nullptr, commitBurstTransaction, nullptr);


void startupState() noexcept {
    if (processorInterface.failTriggered()) {
        fsm.trigger(PerformSelfTest);
    }
}
void systemTestState() noexcept {
    if (!processorInterface.failTriggered()) {
        fsm.trigger(SelfTestComplete);
    }
}
void onASAsserted() {
    processorInterface.triggerAS();
}
void onDENAsserted() {
    processorInterface.triggerDEN();
}
void idleState() noexcept {
    if (processorInterface.failTriggered()) {
        signalHaltState(F("CHECKSUM FAILURE!"));
    } else {
        if (processorInterface.asTriggered()) {
            fsm.trigger(NewRequest);
        }
    }
}
void doAddressState() noexcept {
    if (processorInterface.denTriggered()) {
        fsm.trigger(ToDataState);
    }
}


Device* currentThing = nullptr;
void dataCycleStart() noexcept {
    processorInterface.newDataCycle();
    currentThing = getThing(processorInterface.get16ByteAlignedBaseAddress(), LoadStoreStyle::Full16);
    bool isReadOperation = processorInterface.isReadOperation();
    auto align16BaseAddress = processorInterface.get16ByteAlignedBaseAddress();
    currentThing = getThing(align16BaseAddress, LoadStoreStyle::Full16);
    if (currentThing) {
        if (!processorInterface.blastTriggered()) {
            if (currentThing->supportsBlockTransfers()) {
                currentThing->read(align16BaseAddress, reinterpret_cast<byte*>(burstCache), 16);
                // read into the burst cache as part of data cycle startup
                fsm.trigger(isReadOperation ? ToBurstReadTransaction : ToBurstWriteTransaction);
            } else {
                fsm.trigger(isReadOperation ? ToCyclicBurstReadTransaction : ToCyclicBurstWriteTransaction);
            }
        } else {
            fsm.trigger(isReadOperation ? ToNonBurstReadTransaction : ToNonBurstWriteTransaction);
        }
    } else {
        // unmapped space (including cpu internal)
        if (isReadOperation) {
            // force the data expander to be zero for unmapped reads
            processorInterface.setDataBits(0);
        }
        fsm.trigger(isReadOperation ? ToUnmappedReadTransaction : ToUnmappedWriteTransaction);
    }
}
void performBurstWrite() noexcept {
    processorInterface.updateDataCycle();
    auto offset = processorInterface.getBurstAddressIndex();
    auto& targetCell = burstCache[offset];
    SplitWord16 dataBits(processorInterface.getDataBits());
    // we've gotten the parts to perform the commit
    processorInterface.signalReady();
    switch (processorInterface.getStyle()) {
        case LoadStoreStyle::Full16:
            targetCell.wholeValue_ = dataBits.wholeValue_;
            break;
        case LoadStoreStyle::Lower8:
            targetCell.bytes[0] = dataBits.bytes[0];
            break;
        case LoadStoreStyle::Upper8:
            targetCell.bytes[1] = dataBits.bytes[1];
            break;
        default:
            break;
    }
    if (processorInterface.blastTriggered()) {
        fsm.trigger(ToCommitBurstTransaction);
    }
}
void commitBurstTransaction() noexcept {
    currentThing->write(processorInterface.get16ByteAlignedBaseAddress(),
                        reinterpret_cast<byte*>(burstCache),
                        16);
    fsm.trigger(ToBusRecovery);
}
void performBurstRead() noexcept {
    processorInterface.updateDataCycle();
    // just assign all 16-bits, the processor will choose which bits to care about
    processorInterface.setDataBits(burstCache[processorInterface.getBurstAddressIndex()].wholeValue_);
    processorInterface.signalReady();
    if (processorInterface.blastTriggered()) {
        // we do not need to write anything back
        fsm.trigger(ToBusRecovery);
    }
}
void performNonBurstRead() noexcept {
    processorInterface.updateDataCycle();
    auto result = currentThing->read(processorInterface.getAddress(),
                                     processorInterface.getStyle());
    processorInterface.setDataBits(result);
    processorInterface.signalReady();
    fsm.trigger(ToBusRecovery);
}

void performCyclicBurstRead() noexcept {
    processorInterface.updateDataCycle();
    auto result = currentThing->read(processorInterface.getAddress(),
                                     processorInterface.getStyle());
    processorInterface.setDataBits(result);
    processorInterface.signalReady();
    if (processorInterface.blastTriggered()) {
        fsm.trigger(ToBusRecovery);
    }
}
void performNonBurstWrite() noexcept {
    // write the given value right here and now
    processorInterface.updateDataCycle();
    auto address = processorInterface.getAddress();
    auto bits = processorInterface.getDataBits();
    auto style = processorInterface.getStyle();
    //we've pulled everything off of the bus so signal that we are ready to continue
    processorInterface.signalReady();
    // now call write
    currentThing->write(address, bits, style);
    // we not in burst mode
    fsm.trigger(ToBusRecovery);
}

void performCyclicBurstWrite() noexcept {
    // write the given value right here and now
    processorInterface.updateDataCycle();
    auto address = processorInterface.getAddress();
    auto bits = processorInterface.getDataBits();
    auto style = processorInterface.getStyle();
    processorInterface.signalReady();
    currentThing->write(address, bits, style);
    if (processorInterface.blastTriggered()) {
        // we not in burst mode
        fsm.trigger(ToBusRecovery);
    }
}
void unmappedWrite() noexcept {
    // this is used regardless of burst or non burst operations
    processorInterface.updateDataCycle();
    // we are just going to report the error
    Serial.print(F("UNMAPPED WRITE OF 0x"));
    // expensive but something has gone horribly wrong anyway so whatever!
    Serial.print(processorInterface.getDataBits(), HEX);
    Serial.print(F(" TO 0x"));
    Serial.println(processorInterface.getAddress(), HEX);
    signalHaltState(F("UNMAPPED WRITE!"));
    processorInterface.signalReady();
    if (processorInterface.blastTriggered()) {
        // we not in burst mode
        fsm.trigger(ToBusRecovery);
    }
}
void unmappedRead() noexcept {
    // this is used regardless of burst or non burst operations
    processorInterface.updateDataCycle();
    // we are just going to report the error
    Serial.print(F("UNMAPPED READ FROM 0x"));
    // expensive but something has gone horribly wrong anyway so whatever!
    Serial.println(processorInterface.getAddress(), HEX);
    signalHaltState(F("UNMAPPED READ!"));
    processorInterface.signalReady();
    if (processorInterface.blastTriggered()) {
        // we not in burst mode
        fsm.trigger(ToBusRecovery);
    }
}

// ----------------------------------------------------------------
// setup routines
// ----------------------------------------------------------------

void setupBusStateMachine() noexcept {
    fsm.add_transition(&tStart, &tSystemTest, PerformSelfTest, nullptr);
    fsm.add_transition(&tSystemTest, &tIdle, SelfTestComplete, nullptr);
    fsm.add_transition(&tIdle, &tAddr, NewRequest, nullptr);
    fsm.add_transition(&tAddr, &tData, ToDataState, nullptr);
    // we want to commit the burst transaction in a separate state
    fsm.add_transition(&tData, &tBurstWrite, ToBurstWriteTransaction, nullptr);
    fsm.add_transition(&tBurstWrite, &tCommitBurstTransaction, ToCommitBurstTransaction, nullptr);
    fsm.add_transition(&tCommitBurstTransaction, &tIdle, ToBusRecovery, nullptr);
    auto connectStateToDataAndRecovery = [](auto* state, auto toState) noexcept {
        fsm.add_transition(&tData, state, toState, nullptr);
        fsm.add_transition(state, &tIdle, ToBusRecovery, nullptr);
    };
    connectStateToDataAndRecovery(&tBurstRead, ToBurstReadTransaction);
    connectStateToDataAndRecovery(&tNonBurstRead, ToNonBurstReadTransaction);
    connectStateToDataAndRecovery(&tNonBurstWrite, ToNonBurstWriteTransaction);
    connectStateToDataAndRecovery(&tUnmappedRead, ToUnmappedReadTransaction);
    connectStateToDataAndRecovery(&tUnmappedWrite, ToUnmappedWriteTransaction);
    connectStateToDataAndRecovery(&tCyclicBurstRead, ToCyclicBurstReadTransaction);
    connectStateToDataAndRecovery(&tCyclicBurstWrite, ToCyclicBurstWriteTransaction);
}
void setupPeripherals() {
    displayCommandSet.begin();
    displayReady = true;
    rom.begin();
    dataRom.begin();
    ram.begin();
}
// the setup routine runs once when you press reset:
void setup() {

    Serial.begin(115200);
    while(!Serial) {
        delay(10);
    }
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    setupPins(OUTPUT,
              i960Pinout::SPI_BUS_EN,
              i960Pinout::DISPLAY_EN,
              i960Pinout::SD_EN,
              i960Pinout::Reset960,
              i960Pinout::Ready,
              i960Pinout::GPIOSelect,
              i960Pinout::Led,
              i960Pinout::Int0_);
    PinAsserter<i960Pinout::Reset960> holdi960InReset;
    // all of these pins need to be pulled high
    digitalWriteBlock(HIGH,
                      i960Pinout::SPI_BUS_EN,
                      i960Pinout::SD_EN,
                      i960Pinout::DISPLAY_EN,
                      i960Pinout::Ready,
                      i960Pinout::GPIOSelect,
                      i960Pinout::Int0_);
    setupPins(INPUT,
              i960Pinout::BLAST_,
              i960Pinout::AS_,
              i960Pinout::W_R_,
              i960Pinout::DEN_,
              i960Pinout::FAIL,
              i960Pinout::WriteReadCheck,
              i960Pinout::BurstAddress1,
              i960Pinout::BurstAddress2,
              i960Pinout::BurstAddress3,
              i960Pinout::ByteEnable0,
              i960Pinout::ByteEnable1,
              i960Pinout::BurstLast,
              i960Pinout::FAIL_INPUT);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::AS_)), onASAsserted, FALLING);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::DEN_)), onDENAsserted, FALLING);
    digitalWrite(i960Pinout::Led, LOW);
    fs.begin();
    chipsetFunctions.begin();
    SPI.begin();
    processorInterface.begin();
    // setup the CPU Interface
    setupBusStateMachine();
    setupPeripherals();
    delay(1000);
    // we want to jump into the code as soon as possible after this point
}
void loop() {
    fsm.run_machine();
}

[[noreturn]]
void
signalHaltState(const __FlashStringHelper* haltMsg) {
    if (displayReady) {
        displayCommandSet.clearScreen();
        displayCommandSet.setCursor(0, 0);
        displayCommandSet.setTextSize(2);
        displayCommandSet.println(haltMsg);
    }
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}
SdFat SD;
/// @todo Eliminate after MightyCore update
#if __cplusplus >= 201402L

void operator delete(void * ptr, size_t)
{
    ::operator delete(ptr);
}

void operator delete[](void * ptr, size_t)
{
    ::operator delete(ptr);
}

#endif // end language is C++14 or greater
