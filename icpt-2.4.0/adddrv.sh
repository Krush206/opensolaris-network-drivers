#! /bin/csh -f
#
# IP1000A GbE mac driver
#
# Tested PCI cards
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pci13f0,1023"')	# IC plus ip1000a
#
# Untested PCI cards
#
set DEVLIST = ($DEVLIST '"pci1186,4020"')	# D-Link DL-2000

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" icpt
sync
