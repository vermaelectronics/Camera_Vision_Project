# ============================================================================
#  system_project.tcl  --  Vivado project for the SFAP-PI E310 V1 design
#  Copied by build_all.sh into $ADI_HDL_DIR/projects/gnss_sfap/e310 together
#  with Source/BlockDesign/system_bd.tcl, Source/HDL/system_top.v and the two
#  XDC files, then run by the ADI makefile (vivado -mode batch -source).
#  The part is given explicitly: the E310 is not one of ADI's carriers.
# ============================================================================
source ../../../scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

set p_name gnss_sfap_e310
if {[info exists ::env(SFAP_HDL_PRJ_NAME)]} { set p_name $::env(SFAP_HDL_PRJ_NAME) }

adi_project_create $p_name 0 {} "xc7z020clg400-2"
adi_project_files $p_name [list \
  "$ad_hdl_dir/library/common/ad_iobuf.v" \
  "system_top.v" \
  "system.xdc" \
  "gnss_passthrough_cdc.xdc" ]

adi_project_run $p_name
