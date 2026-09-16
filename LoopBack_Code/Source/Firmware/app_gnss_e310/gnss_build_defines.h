/******************************************************************************
 *  gnss_build_defines.h
 *  Build-variant defines, injected by the build script. GNSS-CRPA MOD-8.
 *
 *  THIS FILE IS A STUB AND IS OVERWRITTEN IN THE VITIS WORKSPACE COPY.
 *  Editing it here changes the DEFAULT for every build. To build a variant,
 *  pass -Defines to Build-Software.ps1 instead.
 *
 *  WHY THIS EXISTS INSTEAD OF -D ON THE COMPILER LINE
 *    The obvious mechanism is app.set_app_config(key="USER_COMPILE_FLAGS", ...)
 *    in the Vitis Python API. On Vitis 2026.1 that call FAILS:
 *
 *        get_config_info: Unable to get the config information
 *
 *    and build_software.py only printed a note and carried on. It went
 *    unnoticed for the whole project, because the two flags it was setting --
 *    XILINX_PLATFORM and ANTSDR_E310 -- are ALSO defined in app_config.h, so
 *    they were redundant. The failure only became visible when a define that
 *    was NOT redundant (GNSS_DEPLOY_AUTO_TX) silently failed to reach the
 *    compiler, producing a "deployment" ELF that did not deploy anything.
 *
 *    A generated header cannot fail that way: if the define is not here, it is
 *    visibly not here, and the file is in the workspace to be inspected.
 *
 *  Included first by app_config.h, so every translation unit that pulls in the
 *  application configuration sees these.
 *****************************************************************************/
#ifndef GNSS_BUILD_DEFINES_H_
#define GNSS_BUILD_DEFINES_H_

/* No build-variant defines. This is the normal JTAG development build:
 * TX is RF-silent at start-up and transmission requires a deliberate
 * gnss_tx=1 or gnss_ddr_tx=1 on the console. */

#endif /* GNSS_BUILD_DEFINES_H_ */
