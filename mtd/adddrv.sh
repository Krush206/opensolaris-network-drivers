#! /bin/csh -f
#
# Myson mtd803 fast ethernet controler driver
#
# PCI vendoro-id/device-id
#
set DEVLIST = ()
set DEVLIST = ($DEVLIST '"pci1516,803"')	# myson mtd803
#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" mtd
sync
