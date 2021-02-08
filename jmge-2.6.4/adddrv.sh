#! /bin/csh -f
#
# Jmicron PCIE ethernet controler Driver
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pciex197b,250"')	# JMC250
set DEVLIST = ($DEVLIST '"pciex197b,260"')	# JMC260

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

set existing = `grep "jmge " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" jmge
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" jmge
endif
sync
