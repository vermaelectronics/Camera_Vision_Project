/******************************************************************************
 *  xparam_compat.h
 *  Compatibility shim between the 2023-era Analog Devices no-OS firmware and
 *  the BSP that Vitis 2026.1 generates.
 *
 *  Original work for this project.
 *
 *  ---------------------------------------------------------------------------
 *  WHAT CHANGED IN THE TOOLS
 *
 *  Vitis 2026.1 generates its BSP in AMD's System Device Tree (SDT) flow. Two
 *  things the older no-OS code depends on are gone:
 *
 *    1. Device IDs. xparameters.h no longer defines XPAR_*_DEVICE_ID at all
 *       (the only survivor is the unrelated string XPAR_DEVICE_ID "7z020").
 *       In the SDT flow the standalone drivers take a BASE ADDRESS instead:
 *
 *         #ifndef SDT
 *         XSpiPs_Config *XSpiPs_LookupConfig(u16 DeviceId);
 *         #else
 *         XSpiPs_Config *XSpiPs_LookupConfig(UINTPTR BaseAddress);
 *         #endif
 *
 *       So the legacy *_DEVICE_ID macros are mapped to the corresponding
 *       *_BASEADDR values here. That is not a fudge: the base address IS the
 *       correct argument for LookupConfig in this flow.
 *
 *    2. Peripheral naming. The PS7-prefixed names were replaced:
 *         XPAR_PS7_SPI_0_SPI_CLK_FREQ_HZ -> XPAR_XSPIPS_0_SPI_CLK_FREQ_HZ
 *         XPAR_PS7_GPIO_0_DEVICE_ID      -> XPAR_XGPIOPS_0_BASEADDR
 *         XPAR_PS7_SCUGIC_0_DEVICE_ID    -> XPAR_XSCUGIC_0_BASEADDR
 *
 *    3. The header guard _XPARAMETERS_PS_H_ is no longer defined. The vendor
 *       parameters.h keys its entire PS branch off that symbol, so without it
 *       the file silently fell through to the MicroBlaze/AXI branch and every
 *       PS definition -- including all eight RF band-select GPIO numbers --
 *       vanished. That was the actual cause of the first failed software build.
 *
 *  ---------------------------------------------------------------------------
 *  WHY A SHIM RATHER THAN EDITING THE DRIVERS
 *
 *  Everything here is a name mapping onto values the BSP genuinely provides.
 *  Keeping it in one file means the vendor no-OS sources stay unmodified and
 *  diffable against Vendor/MicroPhase_E310_V1/, and the whole tool-migration
 *  delta is visible in one place.
 *
 *  Every macro is guarded, so if a future BSP starts providing the legacy name
 *  again the real definition wins and this file becomes a no-op.
 *
 *  ---------------------------------------------------------------------------
 *  VERIFIED AGAINST
 *    Vitis 2026.1, platform e310_gnss_platform, domain standalone_ps7_cortexa9_0
 *    Every replacement symbol below was confirmed present in the generated
 *    xparameters.h before being used here. None is guessed (requirement 32).
 *****************************************************************************/
#ifndef XPARAM_COMPAT_H_
#define XPARAM_COMPAT_H_

#include <xparameters.h>

/* ---------------------------------------------------------------------------
 * 1. Re-assert the PS branch selector.
 *    Vendor parameters.h does "#ifdef _XPARAMETERS_PS_H_" to pick the Zynq PS
 *    definitions. Confirm we really are on a Zynq PS before defining it, so
 *    this cannot silently mis-select on some other architecture.
 * ------------------------------------------------------------------------- */
#if !defined(_XPARAMETERS_PS_H_) && defined(XPAR_XSCUGIC_0_BASEADDR)
#define _XPARAMETERS_PS_H_
#endif

/* ---------------------------------------------------------------------------
 * 2. Device identifiers -> base addresses (SDT flow).
 * ------------------------------------------------------------------------- */
#if !defined(XPAR_PS7_GPIO_0_DEVICE_ID) && defined(XPAR_XGPIOPS_0_BASEADDR)
#define XPAR_PS7_GPIO_0_DEVICE_ID       XPAR_XGPIOPS_0_BASEADDR
#endif

#if !defined(XPAR_PS7_SPI_0_DEVICE_ID) && defined(XPAR_XSPIPS_0_BASEADDR)
#define XPAR_PS7_SPI_0_DEVICE_ID        XPAR_XSPIPS_0_BASEADDR
#endif

#if !defined(XPAR_PS7_SPI_1_DEVICE_ID) && defined(XPAR_XSPIPS_1_BASEADDR)
#define XPAR_PS7_SPI_1_DEVICE_ID        XPAR_XSPIPS_1_BASEADDR
#endif

#if !defined(XPAR_PS7_SCUGIC_0_DEVICE_ID) && defined(XPAR_XSCUGIC_0_BASEADDR)
#define XPAR_PS7_SCUGIC_0_DEVICE_ID     XPAR_XSCUGIC_0_BASEADDR
#endif

#if !defined(XPAR_SCUGIC_SINGLE_DEVICE_ID) && defined(XPAR_XSCUGIC_0_BASEADDR)
#define XPAR_SCUGIC_SINGLE_DEVICE_ID    XPAR_XSCUGIC_0_BASEADDR
#endif

#if !defined(XPAR_XUARTPS_0_DEVICE_ID) && defined(XPAR_XUARTPS_0_BASEADDR)
#define XPAR_XUARTPS_0_DEVICE_ID        XPAR_XUARTPS_0_BASEADDR
#endif

/* ---------------------------------------------------------------------------
 * 3. SPI reference clock.
 *    xilinx_spi.c builds the name by token pasting:
 *      #define SPI_CLK_FREQ_HZ(dev) (XPAR_PS7_SPI_ ## dev ## _SPI_CLK_FREQ_HZ)
 *    so the legacy spelling has to exist as a macro, not just an equivalent.
 * ------------------------------------------------------------------------- */
#if !defined(XPAR_PS7_SPI_0_SPI_CLK_FREQ_HZ) && defined(XPAR_XSPIPS_0_SPI_CLK_FREQ_HZ)
#define XPAR_PS7_SPI_0_SPI_CLK_FREQ_HZ  XPAR_XSPIPS_0_SPI_CLK_FREQ_HZ
#endif

#if !defined(XPAR_PS7_SPI_1_SPI_CLK_FREQ_HZ) && defined(XPAR_XSPIPS_1_SPI_CLK_FREQ_HZ)
#define XPAR_PS7_SPI_1_SPI_CLK_FREQ_HZ  XPAR_XSPIPS_1_SPI_CLK_FREQ_HZ
#endif

/* ---------------------------------------------------------------------------
 * 3b. Peripheral instance counts.
 *
 *     The SDT BSP stops emitting XPAR_<driver>_NUM_INSTANCES for some
 *     peripherals -- XSPIPS is one of them, even though both SPI controllers
 *     are present and have base addresses.
 *
 *     This matters more than it looks. xilinx_spi.c does:
 *         #define SPI_NUM_INSTANCES XPAR_XSPIPS_NUM_INSTANCES   (else 0)
 *         ...
 *         switch (param->device_id) {
 *         #if (SPI_NUM_INSTANCES >= 1)
 *           case 0:  input_clock = SPI_CLK_FREQ_HZ(0); break;
 *         #endif
 *           default: goto ps_error;
 *         }
 *     With the count at 0 BOTH cases are preprocessed away, every call falls
 *     into default, and SPI init fails. The failure is not reported upward, so
 *     ad9361_spi_readm later dereferences an uninitialised descriptor and the
 *     CPU takes a PREFETCH ABORT. Symptom: a completely silent console with the
 *     core spinning in Xil_PrefetchAbortHandler.
 *
 *     Derive the count from the base addresses the BSP does provide.
 * ------------------------------------------------------------------------- */
#if !defined(XPAR_XSPIPS_NUM_INSTANCES)
#  if defined(XPAR_XSPIPS_1_BASEADDR)
#    define XPAR_XSPIPS_NUM_INSTANCES   2
#  elif defined(XPAR_XSPIPS_0_BASEADDR)
#    define XPAR_XSPIPS_NUM_INSTANCES   1
#  endif
#endif

#if !defined(XPAR_XGPIOPS_NUM_INSTANCES) && defined(XPAR_XGPIOPS_0_BASEADDR)
#define XPAR_XGPIOPS_NUM_INSTANCES      1
#endif

/* ---------------------------------------------------------------------------
 * 4. DDR base for the DMA capture buffers.
 *    The vendor code wants XPAR_DDR_MEM_BASEADDR. Vitis 2026.1 exposes the
 *    usable start of PS DDR as XPAR_PS7_DDR_0_BASEADDRESS (0x00100000 -- the
 *    first megabyte is reserved for vectors and the FSBL).
 *
 *    parameters.h then adds +0x800000 for the ADC buffer and +0xA000000 for the
 *    DAC buffer, so the highest address touched is about 0xA100000 (~161 MB),
 *    comfortably inside the E310's DDR.
 * ------------------------------------------------------------------------- */
#if !defined(XPAR_DDR_MEM_BASEADDR)
#  if defined(XPAR_PS7_DDR_0_BASEADDRESS)
#    define XPAR_DDR_MEM_BASEADDR       XPAR_PS7_DDR_0_BASEADDRESS
#  elif defined(XPAR_PSU_DDR_0_S_AXI_BASEADDR)
#    define XPAR_DDR_MEM_BASEADDR       XPAR_PSU_DDR_0_S_AXI_BASEADDR
#  else
#    error "No DDR base address found in xparameters.h. Do not guess one -- inspect the generated BSP and add the correct symbol here."
#  endif
#endif

#endif /* XPARAM_COMPAT_H_ */
