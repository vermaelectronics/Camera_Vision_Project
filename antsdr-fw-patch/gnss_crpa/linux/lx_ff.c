/* gnss_crpa - FatFs calls of gnss_log.c mapped to a directory (see ff.h). */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>
#include "ff.h"

static char base[256];

const char *lx_ff_base(void) { return base; }

static const char *map(const char *p, char *out, size_t n)
{
	if (!strncmp(p, "0:", 2))
		p += 2;
	snprintf(out, n, "%s%s%s", base, (*p == '/') ? "" : "/", p);
	return out;
}

static int mounted_at(const char *dev, char *mnt, size_t n)
{
	FILE *f = fopen("/proc/mounts", "r");
	char d[128], m[256];
	int found = 0;

	if (!f)
		return 0;
	while (fscanf(f, "%127s %255s %*[^\n]", d, m) == 2)
		if (!strcmp(d, dev)) {
			snprintf(mnt, n, "%s", m);
			found = 1;
			break;
		}
	fclose(f);
	return found;
}

FRESULT f_mount(FATFS *fs, const char *path, int opt)
{
	const char *e = getenv("GNSS_CRPA_LOGDIR");
	struct stat st;

	(void)path; (void)opt;
	if (e && *e) {
		snprintf(base, sizeof(base), "%s", e);
		mkdir(base, 0755);
	} else if (!mounted_at("/dev/mmcblk0p1", base, sizeof(base))) {
		if (stat("/dev/mmcblk0p1", &st))
			return FR_NOT_READY;
		mkdir("/mnt/sd", 0755);
		if (mount("/dev/mmcblk0p1", "/mnt/sd", "vfat", MS_NOATIME, "flush"))
			return FR_NOT_READY;
		snprintf(base, sizeof(base), "/mnt/sd");
	}
	if (stat(base, &st) || !S_ISDIR(st.st_mode) || access(base, W_OK))
		return FR_DENIED;
	fs->mounted = 1;
	return FR_OK;
}

FRESULT f_mkdir(const char *path)
{
	char p[512];

	if (mkdir(map(path, p, sizeof(p)), 0755) && errno != EEXIST)
		return FR_DENIED;
	return FR_OK;
}

FRESULT f_open(FIL *fp, const char *path, int mode)
{
	char p[512];

	(void)mode;
	fp->f = fopen(map(path, p, sizeof(p)), "w");
	return fp->f ? FR_OK : FR_DENIED;
}

FRESULT f_write(FIL *fp, const void *buf, UINT btw, UINT *bw)
{
	*bw = (UINT)fwrite(buf, 1, btw, fp->f);
	return (*bw == btw) ? FR_OK : FR_DISK_ERR;
}

FRESULT f_close(FIL *fp)
{
	int r;

	fflush(fp->f);
	fsync(fileno(fp->f));
	r = fclose(fp->f);
	fp->f = NULL;
	return r ? FR_DISK_ERR : FR_OK;
}
