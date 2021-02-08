#! /bin/csh -f
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pci14e4,4401"')	# BCM4401
set DEVLIST = ($DEVLIST '"pci14e4,4402"')	# BCM4401B0
set DEVLIST = ($DEVLIST '"pci14e4,170c"')	# BCM4401B0

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" bfe
sync
