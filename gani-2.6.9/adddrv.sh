#! /bin/csh -f
#
# gani: rtl8169/8110 rtl8168/810x driver
set DEVLIST = ( )

set DEVLIST = ( $DEVLIST '"pci10ec,8169"' )	# PCI 8169
set DEVLIST = ( $DEVLIST '"pci10ec,8167"' )	# PCI 8169SC
set DEVLIST = ( $DEVLIST '"pci1259,c107"' )	# corega/Allied Telesyn 8169
set DEVLIST = ( $DEVLIST '"pci1737,1032"' )	# Lynksy EG1032
set DEVLIST = ( $DEVLIST '"pci1186,4300"' )	# D-Link DGE-528T
set DEVLIST = ( $DEVLIST '"pciex10ec,8168"' )	# PCI-E 8168/811x
set DEVLIST = ( $DEVLIST '"pciex10ec,8136"' )	# PCI-E 810xE

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

set existing = `grep "gani " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" gani
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" gani
endif
sync
