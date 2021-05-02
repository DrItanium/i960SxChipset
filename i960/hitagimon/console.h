//
// Created by jwscoggins on 5/2/21.
//

#ifndef I960SXCHIPSET_CONSOLE_H
#define I960SXCHIPSET_CONSOLE_H

/**
 * @brief Provide system services console io. This routine will be entered from 'calls 0'
 * in the supervisor table, thus allowing an application to execute and do I/O to
 * the serial device of this monitor through run-time binding
 * @param type
 * @param chr
 */
extern "C" int console_io(int type, int chr);

#endif //I960SXCHIPSET_CONSOLE_H
