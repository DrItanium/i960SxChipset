//
// Created by jwscoggins on 5/3/21.
//

#ifndef I960SXCHIPSET_IODEVICE_H
#define I960SXCHIPSET_IODEVICE_H
#include <stdint.h>

class IODevice {
public:
    IODevice(uint8_t* baseAddress);
    IODevice(uint32_t byteOffset);
    virtual ~IODevice();
protected:
    uint8_t* baseAddress_;
};

#endif //I960SXCHIPSET_IODEVICE_H
