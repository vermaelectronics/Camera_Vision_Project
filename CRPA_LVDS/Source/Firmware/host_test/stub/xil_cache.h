/* host test stub: only what the anti-jam engine and the driver use */
#include <stdint.h>
void Xil_DCacheInvalidateRange(uintptr_t a, uint32_t n);
void Xil_DCacheFlushRange(uintptr_t a, uint32_t n);
void Xil_DCacheEnable(void); void Xil_ICacheEnable(void); void Xil_DCacheDisable(void); void Xil_ICacheDisable(void);
