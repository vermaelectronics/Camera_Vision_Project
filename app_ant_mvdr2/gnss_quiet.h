/***************************************************************************//**
 *   @file   gnss_quiet.h
 *   @brief  Keeps the MVDR2 engine modules quiet on the console.
 *
 *   The engine files (autostart, passthrough, l1, command) were written to
 *   print every step. With the LVDS console layer (gnss_ant.c) only the pages
 *   and short result lines are printed. Those files map printf/console_print
 *   to gnss_qprintf(): the text goes into a small ring buffer instead of the
 *   UART, and gnss_quiet_dump() prints it when something FAILED (so a failure
 *   still shows its full explanation). gnss_quiet_set(0) lets it through
 *   (e.g. help-level detail), gnss_quiet_set(1) is the default.
*******************************************************************************/
#ifndef GNSS_QUIET_H_
#define GNSS_QUIET_H_

#include <stdarg.h>
int  gnss_qprintf(const char *fmt, ...);
int  gnss_qvprintf(const char *fmt, va_list ap);
int  gnss_qputchar(int c);
void gnss_quiet_set(int on);         /* 1 = quiet (default), 0 = print       */
int  gnss_quiet_get(void);
void gnss_quiet_clear(void);         /* forget the captured text            */
void gnss_quiet_dump(int last_lines);/* print the last N captured lines      */

/* In an engine .c file, AFTER its #includes:
 *     #include "gnss_quiet.h"
 *     #define GNSS_QUIET_THIS_FILE
 * maps its printf / console_print to gnss_qprintf. */
#ifdef GNSS_QUIET_THIS_FILE
#undef  printf
#define printf        gnss_qprintf
#undef  console_print
#define console_print gnss_qprintf
#undef  vprintf
#define vprintf       gnss_qvprintf
#undef  putchar
#define putchar       gnss_qputchar
#endif

#endif
