#ifndef XIL_CACHE_H_LX
#define XIL_CACHE_H_LX
/* The DMA buffers are handled by the kernel's IIO buffer: nothing to do. */
#define Xil_DCacheFlushRange(a, n)       do { (void)(a); (void)(n); } while (0)
#define Xil_DCacheInvalidateRange(a, n)  do { (void)(a); (void)(n); } while (0)
#define Xil_DCacheFlush()                do { } while (0)
#define Xil_DCacheEnable()               do { } while (0)
#define Xil_DCacheDisable()              do { } while (0)
#define Xil_ICacheEnable()               do { } while (0)
#define Xil_ICacheDisable()              do { } while (0)
#endif
