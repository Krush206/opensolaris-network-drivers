#! /bin/csh -f
#
# via rhine fast ethernet controler driver
#
set DEVLIST = ""
#set DEVLIST = ($DEVLIST '"pci1106,6100"')	# Rhine-I
set DEVLIST = ($DEVLIST '"pci1106,3043"')	# Rhine-I VT86C100A/DL10030A
set DEVLIST = ($DEVLIST '"pci1106,3065"')	# Rhine-II VT6102
set DEVLIST = ($DEVLIST '"pci1106,3106"')	# Rhine-III VT6105
set DEVLIST = ($DEVLIST '"pci1106,3053"')	# Rhine-III VT6105M

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" vfe
sync
