//
// Created by jwscoggins on 8/5/21.
//

#include "ClockGeneration.h"

void setupClockSource() noexcept {
#ifdef ARDUINO_GRAND_CENTRAL_M4
constexpr auto ClockDivider_10MHZ = 6;
constexpr auto ClockDivider_12MHZ = 5;
constexpr auto ClockDivider_15MHZ = 4;
constexpr auto ClockDivider_20MHZ = 3;
//constexpr auto ClockDivider_40MHZ = 2; // THIS IS ALSO DAMN DANGEROUS
//constexpr auto ClockDivider_80MHZ = 1; // DEAR GOD DO NOT USE THIS!!!!
GCLK->GENCTRL[3].reg = GCLK_GENCTRL_DIV(ClockDivider_10MHZ) |
                       GCLK_GENCTRL_IDC |
                       GCLK_GENCTRL_GENEN |
                       GCLK_GENCTRL_OE |
                       GCLK_GENCTRL_SRC_DPLL0;
while (GCLK->SYNCBUSY.bit.GENCTRL3);

constexpr auto pinIndex0 = static_cast<int>(i960Pinout::CLOCK_OUT);
PORT->Group[g_APinDescription[pinIndex0].ulPort].PINCFG[g_APinDescription[pinIndex0].ulPin].bit.PMUXEN = 1;
// enable on pin 36 or PA17
PORT->Group[g_APinDescription[pinIndex0].ulPort].PMUX[g_APinDescription[pinIndex0].ulPin >> 1].reg |= PORT_PMUX_PMUXO(MUX_PA17M_GCLK_IO3);
#endif
}