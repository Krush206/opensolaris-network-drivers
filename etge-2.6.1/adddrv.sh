#! /bin/csh -f
#
# lsilogic/agere ET1310
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pciex11c1,ed00"')	# ET1310 GbE
set DEVLIST = ($DEVLIST '"pciex11c1,ed01"')	# ET1310 fast

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" etge
sync
