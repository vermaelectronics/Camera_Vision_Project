# Vendor/: third-party trees the build uses

| folder | what | where it comes from |
|---|---|---|
| `hls_headers/` | ap_int headers + hls_stream stand-in, for `csim_gcc.sh` | included |
| `adi_hdl/` | the Analog Devices HDL tree (antsdr-fw-patch/plutosdr-fw/hdl): `scripts/`, `projects/scripts/`, `library/` with axi_ad9361, axi_dmac, axi_tdd, util_pack, axi_vcxo_ctrl ... | **added on your PC** by `Automation/Linux/bundle_full.sh` |

When `Vendor/adi_hdl/` exists, `env.sh` uses it automatically, so no
`export ADI_HDL_DIR` is needed. Each build adds its Vivado project inside it,
as `projects/passthrough_lvds_<algo>`.

**Not included, and not includable:** Vivado, Vitis and Vitis HLS 2023.2. They
are AMD/Xilinx tools: licensed, many GB in size, and installed under
`/tools/Xilinx`.
