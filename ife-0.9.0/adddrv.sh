#! /bin/csh -f
#
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pci8086,1029"')	#
set DEVLIST = ($DEVLIST '"pci8086,1030"')	#
set DEVLIST = ($DEVLIST '"pci8086,1229"')	#
set DEVLIST = ($DEVLIST '"pci8086,2449"')	#

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" ife
sync
