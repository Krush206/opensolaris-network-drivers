#! /bin/csh -f
#
# axf ax88172 driver
set DEVLIST = ( )

#
# Tested USB devices
#
set DEVLIST = ( $DEVLIST '"usb7b8,420a"' )	# AXIS ax88172 generic
						# Planex UE2-100TX
						# Hawking UF200
						# TrendNet TU2-ET100
set DEVLIST = ( $DEVLIST '"usbb95,7720"' )	# planex UE2-200TX-G
set DEVLIST = ( $DEVLIST '"usbb95,772a"' )	# Logitec LAN-TX/U2H3
#
# Untested USB devices
#
set DEVLIST = ( $DEVLIST '"usb2001,1a00"' )	# DLINK DUBE100
set DEVLIST = ( $DEVLIST '"usb77b,2226"' )	# Linksys USB200M
set DEVLIST = ( $DEVLIST '"usb846,1040"' )	# Netgear FA120
set DEVLIST = ( $DEVLIST '"usbb95,1720"' )	# Intellinet, ST Lab USB Ethernet
set DEVLIST = ( $DEVLIST '"usb8dd,90ff"' )	# Billionton Systems, USB2AR 
set DEVLIST = ( $DEVLIST '"usb557,2009"' )	# ATEN UC210T
set DEVLIST = ( $DEVLIST '"usb411,3d"' )	# Buffalo LUA-U2-KTX
set DEVLIST = ( $DEVLIST '"usb6189,182d"' )	# Sitecom LN-029 "USB 2.0 10/100 Ethernet adapter"
set DEVLIST = ( $DEVLIST '"usb7aa,17"' )	# corega FEther USB2-TX
set DEVLIST = ( $DEVLIST '"usb1189,893"' )	# Surecom EP-1427X-2
set DEVLIST = ( $DEVLIST '"usb1631,6200"' )	# goodway corp usb gwusb2e
#
# AX88772
set DEVLIST = ( $DEVLIST '"usb13b1,18"' )	# Linksys USB200M
set DEVLIST = ( $DEVLIST '"usb1557,7720"' )	# 0Q0 cable ethernet
set DEVLIST = ( $DEVLIST '"usb7d1,3c05"' )	# DLink DUB E100
set DEVLIST = ( $DEVLIST '"usb2001,3c05"' )	# DLink DUB E100
set DEVLIST = ( $DEVLIST '"usb5ac,1402"' )	# Apple Ethernet USB adapter
#
# AX88178 (not supported yet)
# set DEVLIST = ( $DEVLIST '"usb1737,39"' )	# Linksys USB1000
# set DEVLIST = ( $DEVLIST '"usb411,6e"' )	# Buffalo LUA-U2-KGT/LUA-U2-GT
# set DEVLIST = ( $DEVLIST '"usb4bb,930"' )	# I/O DATA ETG-US2
# set DEVLIST = ( $DEVLIST '"usb50d,505"' )	# Belkin F5D5055


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

set existing = `grep "axf " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" axf
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" axf
endif
sync
