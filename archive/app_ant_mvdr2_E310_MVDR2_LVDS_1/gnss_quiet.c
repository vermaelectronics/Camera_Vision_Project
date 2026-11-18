/***************************************************************************//**
 *   @file   gnss_quiet.c
 *   @brief  See gnss_quiet.h.
*******************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "gnss_quiet.h"

#define QBUF 4096
static char  qbuf[QBUF];
static unsigned qhead = 0, qlen = 0;
static int   quiet = 1;

void gnss_quiet_set(int on) { quiet = on ? 1 : 0; }
int  gnss_quiet_get(void)   { return quiet; }
void gnss_quiet_clear(void) { qhead = 0; qlen = 0; }

int gnss_qprintf(const char *fmt, ...)
{
	char line[512];
	va_list ap;
	int n, i;

	va_start(ap, fmt);
	n = vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	if (n < 0)
		return n;
	if (n >= (int)sizeof(line))
		n = (int)sizeof(line) - 1;
	if (!quiet) {
		fputs(line, stdout);
		return n;
	}
	for (i = 0; i < n; i++) {
		qbuf[(qhead + qlen) % QBUF] = line[i];
		if (qlen < QBUF)
			qlen++;
		else
			qhead = (qhead + 1) % QBUF;
	}
	return n;
}

void gnss_quiet_dump(int last_lines)
{
	unsigned i, start = 0, nl = 0;

	if (!qlen)
		return;
	/* find where the last N lines start */
	for (i = qlen; i > 0; i--) {
		if (qbuf[(qhead + i - 1) % QBUF] == '\n' && i != qlen) {
			if (++nl >= (unsigned)last_lines) {
				start = i;
				break;
			}
		}
	}
	printf("  ---- details ----\r\n");
	for (i = start; i < qlen; i++) {
		char c = qbuf[(qhead + i) % QBUF];
		if (c == '\n')
			fputs("\r\n", stdout);
		else if (c != '\r')
			fputc(c, stdout);
	}
	printf("  -----------------\r\n");
}
