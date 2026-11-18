# 2026-11-11T18:46:42.209863
import vitis

client = vitis.create_client()
client.set_workspace(path="/home/nishant/AntSDR/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_MVDR_STAP/PASS_MVDR_STAP_Vitis")

platform = client.create_platform_component(name = "PASS_MVDR_STAP",hw = "/home/nishant/AntSDR/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_MVDR_STAP/PASS_MVDR_STAP.xsa",os = "standalone",cpu = "ps7_cortexa9_0")

comp = client.create_app_component(name="PASS_MVDR_STAP_test",platform = "/home/nishant/AntSDR/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_MVDR_STAP/PASS_MVDR_STAP_Vitis/PASS_MVDR_STAP/export/PASS_MVDR_STAP/PASS_MVDR_STAP.xpfm",domain = "standalone_ps7_cortexa9_0")

comp = client.get_component(name="PASS_MVDR_STAP_test")
status = comp.import_files(from_loc="/home/nishant/AntSDR/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_MVDR_STAP/source/E310_MVDR2_LVDS/app_ant_mvdr2", files=["ad9361.c", "ad9361.h", "ad9361_api.c", "ad9361_api.h", "ad9361_conv.c", "ad9361_util.c", "ad9361_util.h", "ant_platform.c", "ant_platform.h", "app_config.h", "axi_adc_core.c", "axi_adc_core.h", "axi_dac_core.c", "axi_dac_core.h", "axi_dmac.c", "axi_dmac.h", "axi_io.c", "command.c", "command.h", "common.h", "console.c", "console.h", "delay.c", "gnss_ant.c", "gnss_ant.h", "gnss_autostart.c", "gnss_autostart.h", "gnss_build_defines.h", "gnss_capture.c", "gnss_capture.h", "gnss_config.h", "gnss_dash.c", "gnss_dash.h", "gnss_info.c", "gnss_info.h", "gnss_l1.c", "gnss_l1.h", "gnss_mvdr_auto.c", "gnss_mvdr_auto.h", "gnss_mvdr_steer.c", "gnss_mvdr_steer.h", "gnss_passthrough.c", "gnss_passthrough.h", "gnss_report.c", "gnss_report.h", "gnss_smi.c", "gnss_smi.h", "gnss_txdma.c", "gnss_txdma.h", "gpio_extra.h", "irq_extra.h", "main.c", "no_os_axi_io.h", "no_os_delay.h", "no_os_error.h", "no_os_gpio.c", "no_os_gpio.h", "no_os_irq.c", "no_os_irq.h", "no_os_spi.c", "no_os_spi.h", "no_os_util.c", "no_os_util.h", "parameters.h", "README.md", "spi_extra.h", "uart_extra.h", "xilinx_gpio.c", "xilinx_irq.c", "xilinx_spi.c"], dest_dir_in_cmp = "src")

platform = client.get_platform_component(name="PASS_MVDR_STAP")
status = platform.build()

comp.build()

comp.build()

comp.build()

comp.build()

comp.build()

vitis.dispose()

