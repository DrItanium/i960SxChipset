//
// Created by jwscoggins on 5/3/21.
//

#ifndef I960SXCHIPSET_IODEVICE_H
#define I960SXCHIPSET_IODEVICE_H
#include <stdint.h>
#include "ChipsetInteract.h"

/**
 * @brief Manages the builtin led provided by the chipset
 */
class BuiltinLED {
public:
    bool getValue();
    void setValue(bool value);
    void toggle();
};
#endif //I960SXCHIPSET_IODEVICE_H
