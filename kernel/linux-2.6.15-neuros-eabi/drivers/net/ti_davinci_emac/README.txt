The DaVinci ethernet driver is derived from this reuse driver and a linux networking driver adaptation is provided to plug the cpmac driver core into linux.

This adaptation code was originally written for Linux Kernel 2.4 and then ported for Linux Kernel 2.6.


Driver Files:

[I] Linux Adaptation Files:
cpmacNetLx.c - Linux cpmac network driver file - registers itself as a networking driver
cpmacNetLx.h - Linux cpmac network driver header
cpmacNetLxCfg.h - Linux cpmac network driver configuration header
cpmacNetLxTxRx.c - Linux cpmac network driver packett transfer functions (uses ddc layer to actually do the packet transfer using CPMAC hardware)

[II] Driver Core files:
ddc_cpmac.c - CPMAC Driver core source - this source deals directly with the adapter and configures it for data transfer
ddc_cpmac.h - CPMAC Driver core header
ddc_cpmacTxRx.c - CPMAC Driver core sources for packet transfer
ddc_cpmacCfg.h - CPMAC Driver core configuration header
ddc_cpmacDrv.h - CPMAC Driver core internal data structures
ddc_cpmac_ioctl.h - CPMAC Driver core data structures for ioctl's (statistics etc)
ddc.h - TI Driver Core generic header
ddc_netdev.h - TI Network Driver Core header
ioctl_api.h - MIB and header file
mib_ioctl.h - MIB ioctl related
cslr_cpmac.h - CPMAC peripheral hardware registers definations
_tistdtypes.h - common driver headers
tistdtypes.h - common driver headers
cpmac_palOSCache.h - Cache abstraction for Driver Core (DDC)
cpmac_palOSMem.h - Memory abstraction for Driver Core (DDC)
cpmac_palProtect.h - Protection abstraction for Driver Core (DDC)

MDIO PHY related files:
cpswhalcommon_miimdio.c
cpswhalcommon_miimdio.h
cpswhalcommon_miimdio_regs.h
cpswhalcommon_stddef.h
