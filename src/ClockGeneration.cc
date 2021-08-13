//
// Created by jwscoggins on 8/5/21.
//

#include <Arduino.h>
#include "Pinout.h"
#include "ClockGeneration.h"
#ifdef ARDUINO_ARCH_RP2040
#include "hardware/clocks.h"
#endif
void setupClockSource() noexcept {
#ifdef ARDUINO_GRAND_CENTRAL_M4
constexpr auto ClockSignal_20MHz = 6;
constexpr auto ClockSignal_24MHz = 5;
constexpr auto ClockSignal_30MHz = 4;
constexpr auto ClockSignal_40MHz = 3;
constexpr auto ClockSignal_60MHZ = 2; // THIS IS ALSO DAMN DANGEROUS
GCLK->GENCTRL[3].reg = GCLK_GENCTRL_DIV(ClockSignal_20MHz) |
                       GCLK_GENCTRL_IDC |
                       GCLK_GENCTRL_GENEN |
                       GCLK_GENCTRL_OE |
                       GCLK_GENCTRL_SRC_DPLL0;
while (GCLK->SYNCBUSY.bit.GENCTRL3);

constexpr auto pinIndex0 = static_cast<int>(i960Pinout::CLOCK_OUT);
PORT->Group[g_APinDescription[pinIndex0].ulPort].PINCFG[g_APinDescription[pinIndex0].ulPin].bit.PMUXEN = 1;
// enable on pin 36 or PA17
PORT->Group[g_APinDescription[pinIndex0].ulPort].PMUX[g_APinDescription[pinIndex0].ulPin >> 1].reg |= PORT_PMUX_PMUXO(MUX_PA17M_GCLK_IO3);
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
// divide 133mhz by 6 to get ~ 20MHz (really 20.8 or so)
    clock_gpio_init(static_cast<int>(i960Pinout::CLOCK_OUT), CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 7);
#elif defined(ARDUINO_AVR_ATmega1284)
    // clock source is configured using fuses
#else
#warning "setupClockSource is has a default implementation"
#endif
}