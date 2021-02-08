#! /bin/csh -f
#
# TI ThunderLAN TNETE100A fast ethernet NIC driver
#
set DRV = tne
set DEVLIST = ( )
#
set DEVLIST = ($DEVLIST '"pcie11,ae32"')	# Compaq Netelligent 10/100 TX
set DEVLIST = ($DEVLIST '"pcie11,ae34"')	# Compaq Netelligent 10 T UTP
set DEVLIST = ($DEVLIST '"pcie11,ae35"')	# Compaq Integrated NetFlex-3/P 
set DEVLIST = ($DEVLIST '"pcie11,ae40"')	# Compaq Netelligent Dual10/100 TX UTP
set DEVLIST = ($DEVLIST '"pcie11,ae43"')	# Compaq Netelligent Integrated 10/100 TX UTP
set DEVLIST = ($DEVLIST '"pcie11,b011"')	# Compaq Netelligent 10/100 TX Embedded UTP
set DEVLIST = ($DEVLIST '"pcie11,b012"')	# Compaq Netelligent 10 T/2 PCI UTP/Coax
set DEVLIST = ($DEVLIST '"pcie11,b030"')	# Compaq Netelligent 10/100 TX UTP
set DEVLIST = ($DEVLIST '"pci108d,12"')		# Olicom OC-2325
set DEVLIST = ($DEVLIST '"pci108d,13"')		# Olicom OC-2183/2185
set DEVLIST = ($DEVLIST '"pci108d,14"')		# Olicom OC-2326
set DEVLIST = ($DEVLIST '"pci104c,500"')	# Generic ThunderLAN TNETE100E

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

set existing = `grep "${DRV} " /etc/driver_aliases`
echo $existing
if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" ${DRV}
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" ${DRV}
endif
sync
