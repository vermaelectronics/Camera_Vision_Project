/* Included in front of every firmware source (-include lx_force.h):
 * routes the firmware's console output through gnss_crpa's output layer
 * (local terminal, attached clients and the log). */
#ifndef LX_FORCE_H_
#define LX_FORCE_H_
#include <stdio.h>
#include <stdbool.h>
#include "lx_platform.h"
#define printf  lx_printf
#define putchar lx_putchar
#define puts    lx_puts
#define getchar lx_getchar
#endif
