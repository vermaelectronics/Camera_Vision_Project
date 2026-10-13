Packaged custom IP goes here. Create it (v1.2, with the ESPRIT DOA monitor) with:
  vivado -mode batch -source <LoopBack_Code>/Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl \
         -tclargs <LoopBack_Code>/Source <LoopBack_Code>/Build/ip_repo
The original v1.1 package was removed so that a build can never pick it up by mistake.
