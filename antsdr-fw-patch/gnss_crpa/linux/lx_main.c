/* gnss_crpa - the SMI-PI v4.9.14 GNSS anti-jam firmware for the ANTSDR E310,
 * running under Linux (antsdr-fw-patch, design A).
 *
 *   gnss_crpa            run in this terminal, exactly like the bare-metal
 *                        firmware on the UART (only if no service is running)
 *   gnss_crpa -d         run as the background service (S95gnss_crpa at boot)
 *   gnss                 attach this terminal (PuTTY) to the service: type the
 *                        firmware's commands, see its [AJ] events; Ctrl-D or
 *                        "exit" detaches, the anti-jam keeps running
 *   gnss <command>       one command, e.g.  gnss gnss_band=7,1575.42,61.44,56,30,50
 *
 * The start-up sequence and the console loop are those of the firmware's
 * main.c (MOD-3 .. MOD-10 and SMI-PI v4.1 .. 4.9.14); the hardware set-up the
 * Linux kernel already did (AD9361 init, DMA, DAC core reset) is not repeated.
 * TX1 IS CONDUCTED, ATTENUATED COAX ONLY -- never an antenna.
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "lx_platform.h"
#include "ad9361_api.h"
#include "axi_dmac.h"
#include "console.h"
#include "command.h"
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "gnss_aj.h"
#include "gnss_log.h"
#include "gnss_info.h"

#define GNSS_CRPA_VERSION "4.9.14-linux1"
#define MARK '\x1e'               /* end of a one-shot command's output */

struct ad9361_rf_phy *ad9361_phy;
static struct axi_dmac rx_dmac_s = { .name = "rx_dmac", .base = LX_RX_DMAC_BASE,
				     .direction = DMA_DEV_TO_MEM };
static struct axi_dmac tx_dmac_s = { .name = "tx_dmac", .base = LX_TX_DMAC_BASE,
				     .direction = DMA_MEM_TO_DEV, .hw_cyclic = true };
struct axi_dmac *rx_dmac = &rx_dmac_s;
struct axi_dmac *tx_dmac = &tx_dmac_s;

extern command cmd_list[];
extern const char cmd_no;

/* ---- paths ------------------------------------------------------------------ */
static char sock_path[256], pid_path[256], log_path[256];

static void set_paths(void)
{
	const char *d = getenv("GNSS_CRPA_RUNDIR");
	const char *l = getenv("GNSS_CRPA_LOG");

	snprintf(sock_path, sizeof(sock_path), "%s/gnss_crpa.sock", (d && *d) ? d : "/var/run");
	if (strlen(sock_path) >= sizeof(((struct sockaddr_un *)0)->sun_path)) {
		fprintf(stderr, "gnss_crpa: socket path too long: %s\n", sock_path);
		exit(2);
	}
	snprintf(pid_path, sizeof(pid_path), "%s/gnss_crpa.pid", (d && *d) ? d : "/var/run");
	snprintf(log_path, sizeof(log_path), "%s", (l && *l) ? l : "/var/log/gnss_crpa.log");
}

/* ---- output -------------------------------------------------------------------- */
#define MAXC  8
#define HIST  16384
#define LOGMAX (1024 * 1024)

struct client { int fd; int once; int attached; char in[256]; size_t inlen; };
static struct client cl[MAXC];
static int daemon_mode, listen_fd = -1, log_fd = -1;
static long log_size;
static char hist[HIST];
static size_t hist_pos, hist_fill;
static char obuf[8192];
static size_t olen;

static void write_all(int fd, const char *s, size_t n)
{
	while (n) {
		ssize_t w = write(fd, s, n);
		if (w < 0 && errno == EINTR)
			continue;
		if (w <= 0)
			return;
		s += w;
		n -= (size_t)w;
	}
}

static void drop_client(int i);

static void deliver(const char *s, size_t n)
{
	int i;

	if (!daemon_mode) {
		write_all(1, s, n);
		return;
	}
	if (log_fd >= 0) {
		if (log_size + (long)n > LOGMAX) {
			char old[272];
			close(log_fd);
			snprintf(old, sizeof(old), "%s.1", log_path);
			rename(log_path, old);
			log_fd = open(log_path, O_WRONLY | O_CREAT | O_APPEND | O_TRUNC, 0644);
			log_size = 0;
		}
		if (log_fd >= 0) {
			write_all(log_fd, s, n);
			log_size += (long)n;
		}
	}
	for (i = 0; i < (int)n; i++) {
		hist[hist_pos] = s[i];
		hist_pos = (hist_pos + 1) % HIST;
	}
	hist_fill = (hist_fill + n > HIST) ? HIST : hist_fill + n;
	for (i = 0; i < MAXC; i++) {
		ssize_t w;
		if (cl[i].fd < 0 || !cl[i].attached)
			continue;
		w = send(cl[i].fd, s, n, MSG_DONTWAIT | MSG_NOSIGNAL);
		if (w != (ssize_t)n)
			drop_client(i);            /* gone, or not reading: do not stall the engine */
	}
}

void lx_flush(void)
{
	if (olen) {
		deliver(obuf, olen);
		olen = 0;
	}
}

void lx_out(const char *s, size_t n)
{
	int nl = memchr(s, '\n', n) != NULL;

	while (n) {
		size_t k = sizeof(obuf) - olen;
		if (k > n)
			k = n;
		memcpy(obuf + olen, s, k);
		olen += k;
		s += k;
		n -= k;
		if (olen == sizeof(obuf))
			lx_flush();
	}
	if (nl)
		lx_flush();
}

int lx_printf(const char *fmt, ...)
{
	char tmp[1024], *p = tmp;
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
	va_end(ap);
	if (n < 0)
		return n;
	if ((size_t)n >= sizeof(tmp)) {
		p = malloc((size_t)n + 1);
		if (!p)
			return -1;
		va_start(ap, fmt);
		vsnprintf(p, (size_t)n + 1, fmt, ap);
		va_end(ap);
	}
	lx_out(p, (size_t)n);
	if (p != tmp)
		free(p);
	return n;
}

int lx_putchar(int c)
{
	char ch = (char)c;

	lx_out(&ch, 1);
	return c;
}

int lx_puts(const char *s)
{
	lx_out(s, strlen(s));
	lx_out("\n", 1);
	return 1;
}

/* ---- input: one queue of whole lines, from the terminal or the clients ---- */
#define QLEN 32
#define LMAX (CONSOLE_CMD_MAX - 1)
static struct { char t[LMAX + 1]; int owner; } q[QLEN];
static int q_head, q_count;
static char cur[LMAX + 1];
static int cur_pos = -1, cur_owner = -1, mark_owner = -1;
static int sig_pipe[2] = { -1, -1 };
static volatile sig_atomic_t stop_sig;

static void enqueue(const char *line, size_t n, int owner)
{
	int slot;

	while (n && (line[n - 1] == '\r' || line[n - 1] == '\n'))
		n--;
	if (n > LMAX - 1)
		n = LMAX - 1;
	if (q_count == QLEN)
		return;
	slot = (q_head + q_count) % QLEN;
	memcpy(q[slot].t, line, n);
	q[slot].t[n] = '\n';
	q[slot].t[n + 1] = 0;
	q[slot].owner = owner;
	q_count++;
}

static void drop_client(int i)
{
	int k;

	if (cl[i].fd >= 0)
		close(cl[i].fd);
	cl[i].fd = -1;
	cl[i].inlen = 0;
	for (k = 0; k < q_count; k++)
		if (q[(q_head + k) % QLEN].owner == i)
			q[(q_head + k) % QLEN].owner = -1;
	if (cur_owner == i)
		cur_owner = -1;
	if (mark_owner == i)
		mark_owner = -1;
}

static void send_history(int i)
{
	size_t start = (hist_fill < HIST) ? 0 : hist_pos;
	size_t n1 = (hist_fill < HIST) ? hist_fill : HIST - hist_pos;

	if (n1)
		(void)send(cl[i].fd, hist + start, n1, MSG_NOSIGNAL);
	if (hist_fill == HIST && hist_pos)
		(void)send(cl[i].fd, hist, hist_pos, MSG_NOSIGNAL);
}

static void client_line(int i, char *line, size_t n)
{
	if (n && line[0] == '\x01') {
		if (!strncmp(line + 1, "once", 4)) {
			cl[i].once = 1;
			cl[i].attached = 1;
		} else if (!strncmp(line + 1, "attach", 6)) {
			cl[i].attached = 1;
			send_history(i);
			{
				static const char hello[] =
					"\n---- attached to gnss_crpa " GNSS_CRPA_VERSION " (the last lines are above).\n"
					"---- ? = menu, help? = every command, gnss_band=7,LO,rate,BW,att,gain = custom band.\n"
					"---- Ctrl-D or exit = detach (the anti-jam keeps running).\n";
				(void)send(cl[i].fd, hello, sizeof(hello) - 1, MSG_NOSIGNAL);
			}
		}
		return;
	}
	enqueue(line, n, i);
}

static void client_read(int i)
{
	ssize_t r = recv(cl[i].fd, cl[i].in + cl[i].inlen, sizeof(cl[i].in) - 1 - cl[i].inlen, 0);
	char *nl;

	if (r <= 0) {
		drop_client(i);
		return;
	}
	cl[i].inlen += (size_t)r;
	while ((nl = memchr(cl[i].in, '\n', cl[i].inlen))) {
		size_t n = (size_t)(nl - cl[i].in) + 1;
		client_line(i, cl[i].in, n);
		if (cl[i].fd < 0)
			return;
		memmove(cl[i].in, cl[i].in + n, cl[i].inlen - n);
		cl[i].inlen -= n;
	}
	if (cl[i].inlen == sizeof(cl[i].in) - 1) {   /* no newline in 255 bytes */
		client_line(i, cl[i].in, cl[i].inlen);
		cl[i].inlen = 0;
	}
}

static void stdin_read(void)
{
	static char b[256];
	static size_t bl;
	ssize_t r = read(0, b + bl, sizeof(b) - 1 - bl);
	char *nl;

	if (r <= 0) {                    /* EOF on the terminal: leave like Ctrl-C */
		stop_sig = 1;
		return;
	}
	bl += (size_t)r;
	while ((nl = memchr(b, '\n', bl)) || (nl = memchr(b, '\r', bl))) {
		size_t n = (size_t)(nl - b) + 1;
		enqueue(b, n, -1);
		memmove(b, b + n, bl - n);
		bl -= n;
	}
	if (bl == sizeof(b) - 1) {
		enqueue(b, bl, -1);
		bl = 0;
	}
}

static void shutdown_now(void);

int lx_getchar_wait(int timeout_ms)
{
	struct pollfd p[MAXC + 4];
	int np, i, who[MAXC + 4];

	lx_flush();
	for (;;) {
		if (stop_sig)
			shutdown_now();
		if (cur_pos >= 0) {
			char c = cur[cur_pos++];
			if (!cur[cur_pos])
				cur_pos = -1;            /* the line is finished: its command runs next */
			return (unsigned char)c;
		}
		if (mark_owner >= 0) {           /* the previous command has run: tell its client */
			int o = mark_owner;
			char m = MARK;
			mark_owner = -1;
			if (cl[o].fd >= 0) {
				(void)send(cl[o].fd, &m, 1, MSG_NOSIGNAL);
				if (cl[o].once)
					drop_client(o);
			}
		}
		if (q_count) {
			memcpy(cur, q[q_head].t, sizeof(cur));
			cur_owner = q[q_head].owner;
			mark_owner = cur_owner;
			q_head = (q_head + 1) % QLEN;
			q_count--;
			cur_pos = 0;
			continue;
		}

		np = 0;
		p[np].fd = sig_pipe[0]; p[np].events = POLLIN; who[np++] = -3;
		if (!daemon_mode) {
			p[np].fd = 0; p[np].events = POLLIN; who[np++] = -1;
		} else {
			p[np].fd = listen_fd; p[np].events = POLLIN; who[np++] = -2;
			for (i = 0; i < MAXC; i++)
				if (cl[i].fd >= 0) {
					p[np].fd = cl[i].fd; p[np].events = POLLIN; who[np++] = i;
				}
		}
		if (lx_capture_fd() >= 0) {      /* wake as soon as capture samples arrive */
			p[np].fd = lx_capture_fd(); p[np].events = POLLIN; who[np++] = -4;
		}
		if (poll(p, np, timeout_ms) <= 0)
			return -1;
		for (i = 0; i < np; i++) {
			if (!(p[i].revents & (POLLIN | POLLHUP | POLLERR)))
				continue;
			if (who[i] == -3) {
				char x;
				if (read(sig_pipe[0], &x, 1) < 0) { /* drained */ }
			} else if (who[i] == -1) {
				stdin_read();
			} else if (who[i] == -2) {
				int fd = accept4(listen_fd, NULL, NULL, SOCK_CLOEXEC), k;
				if (fd >= 0) {
					for (k = 0; k < MAXC && cl[k].fd >= 0; k++)
						;
					if (k == MAXC) {
						close(fd);
					} else {
						memset(&cl[k], 0, sizeof(cl[k]));
						cl[k].fd = fd;
					}
				}
			} else if (who[i] >= 0 && cl[who[i]].fd == p[i].fd) {
				client_read(who[i]);
			}
		}
		if (!q_count && cur_pos < 0 && !stop_sig)
			return -1;                   /* nothing typed: run the idle hook */
	}
}

int lx_getchar(void)
{
	int c;

	while ((c = lx_getchar_wait(-1)) < 0)
		;
	return c;
}

/* ---- start / stop -------------------------------------------------------------------- */
static void on_signal(int s)
{
	char x = (char)s;

	stop_sig = 1;
	if (sig_pipe[1] >= 0)
		if (write(sig_pipe[1], &x, 1) < 0) { /* full: a wake-up is already pending */ }
}

static void shutdown_now(void)
{
	stop_sig = 0;
	printf("\ngnss_crpa: stopping -- TX1 silenced (attenuation at maximum, output muted).\n");
	if (ad9361_phy) {
		gnss_pt_set_mute(1);
		(void)gnss_l1_set_tx_attenuation(ad9361_phy, GNSS_L1_TX_MAX_ATTEN_MDB);
		(void)gnss_l1_tx_silence(ad9361_phy);
	}
	lx_capture_close();
	lx_flush();
	if (daemon_mode) {
		unlink(sock_path);
		unlink(pid_path);
	}
	exit(0);
}

static int lock_fd = -1;

static int take_lock(void)
{
	char b[32];

	lock_fd = open(pid_path, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
	if (lock_fd < 0)
		return -1;
	if (flock(lock_fd, LOCK_EX | LOCK_NB))
		return -1;
	if (ftruncate(lock_fd, 0) == 0) {
		snprintf(b, sizeof(b), "%ld\n", (long)getpid());
		write_all(lock_fd, b, strlen(b));
	}
	return 0;
}

static int start_listen(void)
{
	struct sockaddr_un a;

	listen_fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
	if (listen_fd < 0)
		return -1;
	memset(&a, 0, sizeof(a));
	a.sun_family = AF_UNIX;
	snprintf(a.sun_path, sizeof(a.sun_path), "%s", sock_path);
	unlink(sock_path);
	if (bind(listen_fd, (struct sockaddr *)&a, sizeof(a)) || listen(listen_fd, 4))
		return -1;
	chmod(sock_path, 0600);
	return 0;
}

static void firmware(void)
{
	char received_cmd[CONSOLE_CMD_MAX];
	double param[CONSOLE_PARAM_MAX];
	char param_no;
	int cmd_type, invalid_cmd, b;
	unsigned char cmd;

	printf("\n=== ANTSDR E310 - GNSS CRPA, SMI-PI %s, Linux (gnss_crpa %s) ===\n",
	       "4.9.14", GNSS_CRPA_VERSION);
	printf("    TX1 IS CONDUCTED, ATTENUATED COAX ONLY -- never an antenna.\n");

	/* 4.9.13 / 4.9.14: TX2 at the quietest setting and its copy muted */
	ad9361_set_tx_attenuation(ad9361_phy, 1, GNSS_L1_TX2_ATTEN_MDB);
	gnss_pt_set_tx2(0);
	/* MOD-5: no DDS tone, nothing on TX1 until it is switched on deliberately */
	gnss_l1_tx_silence(ad9361_phy);

	/* MOD-3: self-test, not fatal */
	{
		int32_t st = gnss_l1_selftest(ad9361_phy);
		if (st != GNSS_L1_OK)
			printf("gnss: self-test did not pass (%ld). The console is still "
			       "available for investigation.\n", (long)st);
	}

	/* SMI-PI v4.1 profile; antsdr-fw-patch: band 7 values and the start-up band */
	gnss_band7_load();
	b = gnss_band_boot();
	if (gnss_band_set(b) != 0)
		printf("gnss_band: the AD9361 refused part of profile %d\n", b);
	gnss_band_print();

	gnss_info_menu();
	gnss_aj_init();
	gnss_aj_enable(1);
	console_print("\nANTI-JAM (SMI-PI " GNSS_AJ_VERSION_STR "): ON, bypassed until a jammer appears.\n"
		      "  Jammer events are printed as [AJ] lines with a time stamp. gnss_aj? = state,\n"
		      "  gnss_report? = everything (copy it into a report), gnss_aj_csv=500 = CSV log,\n"
		      "  Only jammer events are printed (gnss_aj_log=30 adds a status line every 30 s).\n"
		      "  TX1 is switched on by itself (4.9.5: gnss_tx=1 always, also after every gnss_band=).\n"
		      "  CONDUCTED, ATTENUATED COAX ONLY. gnss_tx=0 silences it.\n"
		      "  Band 7 = your own values: gnss_band=7,<LO MHz>,<MS/s>,<BW MHz>,<TX1 att dB>,<RX gain dB>\n");
	gnss_tx_always();
	gnss_log_init();
	console_set_idle_hook(gnss_idle);

	for (;;) {
		console_get_command(received_cmd);
		if (received_cmd[0] == '\r' || received_cmd[0] == '\n')
			continue;
		if (received_cmd[0] >= '0' && received_cmd[0] <= '9') {
			int menu_opt = 0, digit = 0;
			while (digit < 3 && received_cmd[digit] >= '0' && received_cmd[digit] <= '9') {
				menu_opt = menu_opt * 10 + (received_cmd[digit] - '0');
				digit++;
			}
			if (received_cmd[digit] == '\r' || received_cmd[digit] == '\n') {
				gnss_info_select(menu_opt, ad9361_phy);
				continue;
			}
		}
		invalid_cmd = 0;
		for (cmd = 0; cmd < (unsigned char)cmd_no; cmd++) {
			param_no = 0;
			cmd_type = console_check_commands(received_cmd, cmd_list[cmd].name, param, &param_no);
			if (cmd_type == UNKNOWN_CMD)
				invalid_cmd++;
			else
				cmd_list[cmd].function(param, param_no);
		}
		if (invalid_cmd == cmd_no)
			console_print("Invalid command!  Type  ?  for the menu, or  help?  for every command.\n");
	}
}

/* ---- client: `gnss` ----------------------------------------------------------------- */
static int client(int argc, char **argv)
{
	struct sockaddr_un a;
	int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
	char buf[4096];

	memset(&a, 0, sizeof(a));
	a.sun_family = AF_UNIX;
	snprintf(a.sun_path, sizeof(a.sun_path), "%s", sock_path);
	if (fd < 0 || connect(fd, (struct sockaddr *)&a, sizeof(a))) {
		fprintf(stderr, "gnss: the gnss_crpa service is not running.\n"
			"  start it:  /etc/init.d/S95gnss_crpa start\n"
			"  or run it in this terminal:  gnss_crpa\n");
		return 1;
	}
	signal(SIGPIPE, SIG_IGN);
	if (argc > 0) {                       /* one command */
		char line[512] = "\x01once\n";
		size_t n = strlen(line);
		int i;
		for (i = 0; i < argc && n < sizeof(line) - 2; i++)
			n += (size_t)snprintf(line + n, sizeof(line) - n, "%s%s", i ? " " : "", argv[i]);
		if (n > sizeof(line) - 2)
			n = sizeof(line) - 2;
		line[n++] = '\n';
		write_all(fd, line, n);
		for (;;) {
			struct pollfd p = { fd, POLLIN, 0 };
			ssize_t r;
			char *m;
			if (poll(&p, 1, 120000) <= 0) {
				fprintf(stderr, "gnss: no answer within 120 s\n");
				return 1;
			}
			r = recv(fd, buf, sizeof(buf), 0);
			if (r <= 0)
				return 0;
			m = memchr(buf, MARK, (size_t)r);
			write_all(1, buf, m ? (size_t)(m - buf) : (size_t)r);
			if (m)
				return 0;
		}
	}
	write_all(fd, "\x01" "attach\n", 8);
	for (;;) {
		struct pollfd p[2] = { { 0, POLLIN, 0 }, { fd, POLLIN, 0 } };
		ssize_t r;
		if (poll(p, 2, -1) < 0) {
			if (errno == EINTR)
				continue;
			return 1;
		}
		if (p[1].revents) {
			ssize_t k, o = 0;
			r = recv(fd, buf, sizeof(buf), 0);
			if (r <= 0) {
				fprintf(stderr, "\ngnss: the service stopped\n");
				return 1;
			}
			for (k = 0; k < r; k++)
				if (buf[k] != MARK)
					buf[o++] = buf[k];
			write_all(1, buf, (size_t)o);
		}
		if (p[0].revents) {
			r = read(0, buf, sizeof(buf));
			if (r <= 0)
				break;
			if ((r >= 4 && !strncmp(buf, "exit", 4) && (r == 4 || buf[4] == '\n' || buf[4] == '\r')) ||
			    (r >= 4 && !strncmp(buf, "quit", 4) && (r == 4 || buf[4] == '\n' || buf[4] == '\r')))
				break;
			write_all(fd, buf, (size_t)r);
		}
	}
	printf("\ngnss: detached; gnss_crpa keeps running (gnss = attach again).\n");
	return 0;
}

static void usage(void)
{
	fprintf(stderr,
		"gnss_crpa %s - SMI-PI 4.9.14 GNSS anti-jam for the ANTSDR E310 (Linux)\n"
		"  gnss_crpa        run in this terminal (no service running)\n"
		"  gnss_crpa -d     run as the background service (S95gnss_crpa)\n"
		"  gnss             attach to the service (Ctrl-D / exit detaches)\n"
		"  gnss <command>   one command, e.g.  gnss gnss_band?\n"
		"                   gnss gnss_band=7,1575.42,61.44,56,30,50\n"
		"Log of the service: %s\n", GNSS_CRPA_VERSION, log_path);
}

int main(int argc, char **argv)
{
	const char *me = strrchr(argv[0], '/');
	int i;

	me = me ? me + 1 : argv[0];
	set_paths();
	for (i = 0; i < MAXC; i++)
		cl[i].fd = -1;

	if (!strcmp(me, "gnss"))
		return client(argc - 1, argv + 1);
	if (argc > 1 && !strcmp(argv[1], "-c"))
		return client(argc - 2, argv + 2);
	if (argc > 1 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
		usage();
		return 0;
	}
	daemon_mode = (argc > 1 && !strcmp(argv[1], "-d"));
	if (argc > 1 && !daemon_mode) {
		usage();
		return 2;
	}

	if (take_lock()) {
		fprintf(stderr, "gnss_crpa is already running (%s). Talk to it with:  gnss\n", pid_path);
		return 1;
	}
	if (daemon_mode) {
		pid_t pid = fork();
		int nul;
		if (pid < 0)
			return 1;
		if (pid > 0)
			_exit(0);
		setsid();
		log_fd = open(log_path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
		if (log_fd >= 0)
			log_size = lseek(log_fd, 0, SEEK_END);
		if (start_listen())     /* before chdir: a relative GNSS_CRPA_RUNDIR (tests) still works */
			printf("gnss_crpa: cannot listen on %s: %s\n", sock_path, strerror(errno));
		if (sock_path[0] == '/' && chdir("/")) { /* a relative run dir (tests) keeps the cwd */ }
		nul = open("/dev/null", O_RDWR);
		if (nul >= 0) {
			dup2(nul, 0);
			dup2(nul, 1);
			dup2(nul, 2);
			if (nul > 2)
				close(nul);
		}
		if (ftruncate(lock_fd, 0) == 0) {
			char b[32];
			snprintf(b, sizeof(b), "%ld\n", (long)getpid());
			if (pwrite(lock_fd, b, strlen(b), 0) < 0) { /* the lock is what counts */ }
		}
	}
	if (pipe2(sig_pipe, O_CLOEXEC | O_NONBLOCK))
		sig_pipe[0] = sig_pipe[1] = -1;
	signal(SIGPIPE, SIG_IGN);
	signal(SIGTERM, on_signal);
	signal(SIGINT, on_signal);
	signal(SIGHUP, on_signal);

	if (lx_io_init() || lx_ad9361_open(&ad9361_phy)) {
		printf("gnss_crpa: hardware not available -- is this the antsdr-fw-patch 0003 (SMI-PI) bitstream?\n");
		lx_flush();
		return 1;
	}
	if (lx_capture_open())
		printf("gnss_crpa: WARNING no RX capture: the anti-jam cannot compute weights "
		       "(the RX1 -> TX1 path itself still works).\n");
	firmware();
	return 0;
}
