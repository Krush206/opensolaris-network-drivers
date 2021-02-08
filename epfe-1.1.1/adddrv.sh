#! /bin/csh -f
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pci10b8,5"')	# LAN83C170/171
set DEVLIST = ($DEVLIST '"pci10b8,6"')	# LAN83C173/175

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" epfe
sync
