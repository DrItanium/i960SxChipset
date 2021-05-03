//
// Created by jwscoggins on 5/3/21.
//

#include "IODevice.h"
#include "ChipsetInteract.h"
IODevice::IODevice(uint8_t* baseAddress) : baseAddress_(baseAddress)
{

}

IODevice::IODevice(uint32_t byteOffset) : baseAddress_(computeIOAddress<uint8_t>(byteOffset)) {

}
IODevice::~IODevice()
{

}
