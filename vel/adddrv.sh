#! /bin/csh -f
#
# via velocity gigabit ethernet controller driver
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pci1106,3119"')	# velocity VT6122

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" vel
sync
