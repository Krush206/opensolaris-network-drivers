#! /bin/csh -f
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pci14e4,4401"')	# BCM4401
set DEVLIST = ($DEVLIST '"pci14e4,4402"')	# BCM4401B0
set DEVLIST = ($DEVLIST '"pci14e4,170c"')	# BCM4401B0

#echo $DEVLIST

set DEVLIST2 = ( )
foreach i ($DEVLIST)
	set pcidev = `grep $i /etc/driver_aliases`
	echo $pcidev
	if ("$pcidev" == "") then
		set DEVLIST2 = ( $DEVLIST2 "$i" )
	endif
end

#echo $DEVLIST2
if ("$DEVLIST2" == "") then
	echo nothing to do.
	exit 1
endif

set existing = `grep "bfe " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" bfe
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" bfe
endif
sync
