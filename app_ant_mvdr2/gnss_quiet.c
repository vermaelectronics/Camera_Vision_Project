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

static void qput(const char *line, int n)
{
	int i;
	for (i = 0; i < n; i++) {
		qbuf[(qhead + qlen) % QBUF] = line[i];
		if (qlen < QBUF)
			qlen++;
		else
			qhead = (qhead + 1) % QBUF;
	}
}

/* MOD8: spool -- the periodic KPI line / report are captured here and sent
 * to the UART only as fast as it can take them (115200 baud ~ 11.5 bytes/ms,
 * at most one 64-byte UART FIFO per pass), so printing a 6 KB report never
 * holds the main loop (SMI solves, AGC) for the ~0.6 s it takes on the wire. */
#define SPOOL 16384
static char  spool[SPOOL];
static unsigned sp_len = 0, sp_pos = 0;
static int   spooling = 0, sp_cut = 0;
void gnss_spool_begin(void) { if (sp_pos >= sp_len) { sp_len = 0; sp_pos = 0; sp_cut = 0; } spooling = 1; }
void gnss_spool_end(void)   { spooling = 0; }
int  gnss_spool_busy(void)  { return sp_pos < sp_len; }
int  gnss_spool_drain(unsigned max_bytes)
{
	unsigned n = sp_len - sp_pos;
	if (!n)
		return 0;
	if (n > max_bytes)
		n = max_bytes;
	fwrite(spool + sp_pos, 1, n, stdout);
	fflush(stdout);
	sp_pos += n;
	if (sp_pos >= sp_len) { sp_len = 0; sp_pos = 0; sp_cut = 0; return 2; }   /* 2 = just finished */
	return 1;
}
static void spool_put(const char *s, int n)
{
	static const char cut[] = "\r\n[... cut: spool full]\r\n";
	if (n <= 0 || sp_cut) return;
	if (sp_len + (unsigned)n > SPOOL - sizeof(cut)) {              /* full: mark it once, drop the rest */
		memcpy(spool + sp_len, cut, sizeof(cut) - 1);
		sp_len += sizeof(cut) - 1;
		sp_cut = 1;
		return;
	}
	memcpy(spool + sp_len, s, (size_t)n);
	sp_len += (unsigned)n;
}

int gnss_qvprintf(const char *fmt, va_list ap)
{
	char line[512];
	int n = vsnprintf(line, sizeof(line), fmt, ap);

	if (n < 0)
		return n;
	if (n >= (int)sizeof(line))
		n = (int)sizeof(line) - 1;
	if (spooling) {
		spool_put(line, n);
		return n;
	}
	if (!quiet) {
		fputs(line, stdout);
		return n;
	}
	qput(line, n);
	return n;
}

int gnss_qprintf(const char *fmt, ...)
{
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = gnss_qvprintf(fmt, ap);
	va_end(ap);
	return n;
}

int gnss_qputchar(int c)
{
	char ch = (char)c;
	if (spooling) { spool_put(&ch, 1); return c; }
	if (!quiet)
		return fputc(c, stdout);
	qput(&ch, 1);
	return c;
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
