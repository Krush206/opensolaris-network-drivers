#! /bin/csh -f
#
# attansic/atheros L1 series driver
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pciex1969,1048"')	# attansic L1
set DEVLIST = ($DEVLIST '"pciex1969,1026"')	# athros L1E/L2E
set DEVLIST = ($DEVLIST '"pciex1969,2048"')	# attansic L2
set DEVLIST = ($DEVLIST '"pciex1969,1063"')	# Atheros AR8131
set DEVLIST = ($DEVLIST '"pciex1969,1062"')	# Atheros AR8132
set DEVLIST = ($DEVLIST '"pciex1969,2060"')	# Atheros AR8152 v1.1
set DEVLIST = ($DEVLIST '"pciex1969,2062"')	# Atheros AR8152 v2.0
set DEVLIST = ($DEVLIST '"pciex1969,1073"')	# Atheros AR8151 v1.0
set DEVLIST = ($DEVLIST '"pciex1969,1083"')	# Atheros AR8151 v2.0

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

set existing = `grep "atge " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" atge
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" atge
endif
sync
