#! /bin/csh -f
#
# Tested NE2000 campatible PCMCIA cards
#set BASEDIR = /mnt
set BASEDIR = /
set DEVLIST = ()

# Planex communication Inc. FNW-3600-T/ENW3503-T pccard149,c1ab
set DEVLIST =  ($DEVLIST '"pccard149,c1ab"')

# IBM Credit Card Adapter Ethernet II pccarda4,2
set DEVLIST =  ($DEVLIST '"pccarda4,2"')
 

/usr/sbin/add_drv -b $BASEDIR -v -n -m '* 0600 root sys' -i "$DEVLIST" pcni
sync
