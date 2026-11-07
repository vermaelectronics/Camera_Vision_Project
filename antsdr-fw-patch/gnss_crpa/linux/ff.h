/* FatFs API used by gnss_log.c (GNSS_LOG_SD), on Linux: "0:/" is the SD
 * card's FAT boot partition (/dev/mmcblk0p1, mounted on /mnt/sd if it is not
 * mounted yet) or the directory in GNSS_CRPA_LOGDIR. Implemented in lx_ff.c. */
#ifndef FF_H_LX
#define FF_H_LX
#include <stdio.h>
typedef unsigned int UINT;
typedef struct { int mounted; } FATFS;
typedef struct { FILE *f; } FIL;
typedef enum { FR_OK = 0, FR_DISK_ERR, FR_NOT_READY, FR_NO_FILE, FR_DENIED, FR_EXIST } FRESULT;
#define FA_WRITE          0x02
#define FA_CREATE_ALWAYS  0x08
FRESULT f_mount(FATFS *fs, const char *path, int opt);
FRESULT f_mkdir(const char *path);
FRESULT f_open(FIL *fp, const char *path, int mode);
FRESULT f_write(FIL *fp, const void *buf, UINT btw, UINT *bw);
FRESULT f_close(FIL *fp);
const char *lx_ff_base(void);
#endif
