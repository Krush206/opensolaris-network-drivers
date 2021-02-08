#! /bin/csh -f
#set BASEDIR = /mnt
set BASEDIR = /
#
# Tested NE2000 campatible PCI cards
#  vendor-ID, device-ID
#  1106, 926		VIA 82C926 Amazon (works 16bit transfer only)
#  10ec, 8029		Realtek 8029 (32bit and 16bit transfer)
#
#
# Untested NE2000 campatible PCI cards
#  vendor-ID, device-ID
#  1050, 940		Winbond 89C940
#  1050, 5a5a		Winbond 89C940F
#  11f6, 1401		Compex ReadyLink 2000
#  8e2e, 3000		KTI ET32P2
#  4a14, 5000		NetVin NV5000
#  10bd, e34		SureCom NE34

set DEVLIST = ()
#  1106,926   VIA 82C926 Amazon (works 16bit transfer only)
set DEVLIST =  ($DEVLIST '"pci1106,926"')

#  10ec,8029  Realtek 8029 (32bit and 16bit transfer)
set DEVLIST = ($DEVLIST '"pci10ec,8029"')

#  1050,940	Winbond 89C940
set DEVLIST = ($DEVLIST '"pci1050,940"')

#  1050,5a5a	Winbond 89C940F
set DEVLIST = ($DEVLIST '"pci1050,5a5a"')

#  11f6,1401	Compex ReadyLink 2000
set DEVLIST = ($DEVLIST '"pci11f6,1401"')

#  8e2e,3000	KTI ET32P2
set DEVLIST = ($DEVLIST '"pci8e2e,3000"')

#  4a14,5000	NetVin NV5000
set DEVLIST = ($DEVLIST '"pci4a14,5000"')

#  10bd,e34	SureCom NE34
set DEVLIST = ($DEVLIST '"pci10bd,e34"')

#  pnpRTL,8019	Realtek 8019 (PnP mode)
set DEVLIST = ($DEVLIST '"pnpRTL,8019"')

#echo $DEVLIST

/usr/sbin/add_drv -b $BASEDIR -n -v -m '* 0600 root sys' -i "$DEVLIST" ni
sync
