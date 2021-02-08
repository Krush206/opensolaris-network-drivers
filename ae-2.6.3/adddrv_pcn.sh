#! /bin/csh -f
#
# ae: AMD PCNET driver
set DEVLIST = ( )

set DEVLIST = ( $DEVLIST '"pci1022,2000"' )	# lance
set DEVLIST = ( $DEVLIST '"pci1022,2001"' )	# lance home
#set DEVLIST = ( $DEVLIST '"pci1023,2000"' )	# incorrect vendor id

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" pcn
sync
