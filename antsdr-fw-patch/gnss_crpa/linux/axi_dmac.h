/* AXI DMAC (no-OS API) - emulated on Linux by lx_dmac.c.
 * Register map and types as in the firmware's axi_dmac.h. */
#ifndef AXI_DMAC_H_LX
#define AXI_DMAC_H_LX
#include <stdint.h>
#include <stdbool.h>
#include "no_os_util.h"

#define AXI_DMAC_REG_IRQ_MASK		0x80
#define AXI_DMAC_REG_IRQ_PENDING	0x84
#define AXI_DMAC_IRQ_SOT		NO_OS_BIT(0)
#define AXI_DMAC_IRQ_EOT		NO_OS_BIT(1)
#define AXI_DMAC_REG_INTF_DESC		0x010
#define AXI_DMAC_REG_CTRL		0x400
#define AXI_DMAC_CTRL_ENABLE		NO_OS_BIT(0)
#define AXI_DMAC_CTRL_DISABLE		0u
#define AXI_DMAC_CTRL_PAUSE		NO_OS_BIT(1)
#define AXI_DMAC_REG_TRANSFER_ID	0x404
#define AXI_DMAC_REG_TRANSFER_SUBMIT	0x408
#define AXI_DMAC_TRANSFER_SUBMIT	NO_OS_BIT(0)
#define AXI_DMAC_QUEUE_FULL		NO_OS_BIT(0)
#define AXI_DMAC_REG_FLAGS		0x40c
#define AXI_DMAC_REG_DEST_ADDRESS	0x410
#define AXI_DMAC_REG_SRC_ADDRESS	0x414
#define AXI_DMAC_REG_X_LENGTH		0x418
#define AXI_DMAC_REG_Y_LENGTH		0x41c
#define AXI_DMAC_REG_DEST_STRIDE	0x420
#define AXI_DMAC_REG_SRC_STRIDE		0x424
#define AXI_DMAC_REG_TRANSFER_DONE	0x428

enum use_irq { IRQ_DISABLED = 0, IRQ_ENABLED = 1 };
enum dma_direction { INVALID_DIR = 0, DMA_DEV_TO_MEM = 1, DMA_MEM_TO_DEV = 2, DMA_MEM_TO_MEM = 3 };
enum cyclic_transfer { NO = 0, CYCLIC = 1 };

struct axi_dma_transfer {
	uint32_t size;
	volatile bool transfer_done;
	enum cyclic_transfer cyclic;
	uint32_t src_addr;
	uint32_t dest_addr;
};

struct axi_dmac {
	const char *name;
	uint32_t base;
	enum use_irq irq_option;
	enum dma_direction direction;
	bool hw_cyclic;
	uint32_t max_length;
	volatile struct axi_dma_transfer transfer;
};

int32_t axi_dmac_read(struct axi_dmac *dmac, uint32_t reg_addr, uint32_t *reg_data);
int32_t axi_dmac_write(struct axi_dmac *dmac, uint32_t reg_addr, uint32_t reg_data);
int32_t axi_dmac_transfer_start(struct axi_dmac *dmac, struct axi_dma_transfer *dma_transfer);
int32_t axi_dmac_transfer_wait_completion(struct axi_dmac *dmac, uint32_t timeout_ms);
void    axi_dmac_transfer_stop(struct axi_dmac *dmac);
#endif
