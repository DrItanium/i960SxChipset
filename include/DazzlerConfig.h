//
// Created by jwscoggins on 10/7/21.
//

#ifndef SXCHIPSET_DAZZLERCONFIG_H
#define SXCHIPSET_DAZZLERCONFIG_H
#ifdef USE_DAZZLER
#include "Pinout.h"
#define SD_PIN static_cast<int>(i960Pinout::SD_EN)
#define DEFAULT_CS static_cast<int>(i960Pinout::GPU_EN)
#include <GD2.h>
#endif
#endif //SXCHIPSET_DAZZLERCONFIG_H
