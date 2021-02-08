#! /bin/csh -f
#
# SiS190/191 fast/giga ethernet controler driver
#
# PCI vendoro-id/device-id
#  SiS190     vid:1039, did:190		tested
#  SiS191     vid:1039, did:191		untested
#
set DEVLIST = ()
set DEVLIST = ($DEVLIST '"pci1039,190"')
set DEVLIST = ($DEVLIST '"pci1039,191"')
#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" sige
sync
