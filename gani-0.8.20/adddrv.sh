#! /bin/csh -f
#
# gani: rtl8169/8110 driver
set DEVLIST = ( )

set DEVLIST = ( $DEVLIST '"pci10ec,8169"' )
set DEVLIST = ( $DEVLIST '"pci1259,c107"' )	# corega/Allied Telesyn

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" gani
sync
