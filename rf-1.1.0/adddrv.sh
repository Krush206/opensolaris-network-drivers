#! /bin/csh -f
#
# rf: rtl8129/8139 driver
set DEVLIST = ( )

#
# Tested PCI cards
#
#  NDC100TX-E, pci10ec,8129  (RTL8129)
set DEVLIST = ( $DEVLIST '"pci10ec,8129"' )

#  Corega Inc,     pci10ec,8139 (RTL8139C)
set DEVLIST = ( $DEVLIST '"pci10ec,8139"' )

#
# Untested PCI cards
#
#  RTL8138B
set DEVLIST = ( $DEVLIST '"pci10ec,8138"' )

#  DLink 530TX+
#  DLink 538TX
set DEVLIST = ( $DEVLIST '"pci1186,1300"' )

#  SMC1211TX
#  Accton MPX5030
set DEVLIST = ( $DEVLIST '"pci1113,1211"' )

#  LevelOne FPC-0106TX
set DEVLIST = ( $DEVLIST '"pci18a,106"' )

#  Compaq HNE-300
set DEVLIST = ( $DEVLIST '"pci21b,8139"' )

#  Corega Inc,
set DEVLIST = ( $DEVLIST '"pci1259,a11e"' )

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" rf
sync
