#! /bin/csh -f
#
set DEVLIST = ( )

#
# Tested USB devices
#
set DEVLIST = ( $DEVLIST '"usb789,164"' )	# Logitec LAN-W150N/U2PSP
set DEVLIST = ( $DEVLIST '"usb789,168"' )	# Logitec LAN-W150/U2BK
set DEVLIST = ( $DEVLIST '"usb2019,ab24"' )	# Planex GW-US300MiniS
set DEVLIST = ( $DEVLIST '"usb2019,ab25"' )	# Planex GW-USMini2N
set DEVLIST = ( $DEVLIST '"usb2019,ed14"' )	# Planex GW-USMicroN
set DEVLIST = ( $DEVLIST '"usb411,15d"' )	# Buffalo WLI-UC-GN
set DEVLIST = ( $DEVLIST '"usb411,1a2"' )	# Buffalo WLI-UC-GNM
set DEVLIST = ( $DEVLIST '"usb7aa,42"' )	# Corega CG-WLUSB300GNM

#
# Untested USB devices
#

#
# RT2870
#
set DEVLIST = ( $DEVLIST '"usb148f,2770"' )	# Ralink 
set DEVLIST = ( $DEVLIST '"usb148f,2870"' )	# Ralink 
set DEVLIST = ( $DEVLIST '"usb7b8,2870"' )	# AboCom 
set DEVLIST = ( $DEVLIST '"usb7b8,2770"' )	# AboCom 
set DEVLIST = ( $DEVLIST '"usb1482,3c09"' )	# Abocom 
set DEVLIST = ( $DEVLIST '"usb83a,6618"' )	# Accton 
set DEVLIST = ( $DEVLIST '"usb14b2,3c23"' )	# Airlink 
set DEVLIST = ( $DEVLIST '"usb14b2,3c27"' )	# Airlink 
set DEVLIST = ( $DEVLIST '"usb14b2,3c07"' )	# AL 
set DEVLIST = ( $DEVLIST '"usb14b2,3c09"' )	# Alpha 
set DEVLIST = ( $DEVLIST '"usb15c5,8"' )	# Amit 
set DEVLIST = ( $DEVLIST '"usb83a,7512"' )	# Arcadyan 2770 
set DEVLIST = ( $DEVLIST '"usb83a,7522"' )	# Arcadyan 
set DEVLIST = ( $DEVLIST '"usb83a,8522"' )	# Arcadyan 
set DEVLIST = ( $DEVLIST '"usbb05,1731"' )	# Asus 
set DEVLIST = ( $DEVLIST '"usbb05,1732"' )	# Asus 
set DEVLIST = ( $DEVLIST '"usbb05,1742"' )	# Asus 
set DEVLIST = ( $DEVLIST '"usb13d3,3247"' )	# AzureWave 
set DEVLIST = ( $DEVLIST '"usb411,e8"' )	# Buffalo WLI-UC-G300N 
#set DEVLIST = ( $DEVLIST '"usb411,1a2"' )	# Buffalo WLI-UC-GNM
set DEVLIST = ( $DEVLIST '"usb50d,8053"' )	# Belkin 
set DEVLIST = ( $DEVLIST '"usb50d,805c"' )
set DEVLIST = ( $DEVLIST '"usb50d,815c"' )	# Belkin F5D8053 
set DEVLIST = ( $DEVLIST '"usb14b2,3c06"' )	# Conceptronic 
set DEVLIST = ( $DEVLIST '"usb14b2,3c28"' )	# Conceptronic 
set DEVLIST = ( $DEVLIST '"usb7aa,2f"' )	# Corega 
set DEVLIST = ( $DEVLIST '"usb7aa,3c"' )	# Corega 
set DEVLIST = ( $DEVLIST '"usb7aa,3f"' )	# Corega 
set DEVLIST = ( $DEVLIST '"usb14b2,3c25"' )	# Draytek 
set DEVLIST = ( $DEVLIST '"usb7d1,3c09"' )	# D-Link 
set DEVLIST = ( $DEVLIST '"usb7d1,3c11"' )	# D-Link 
set DEVLIST = ( $DEVLIST '"usb1740,9701"' )	# EnGenius 
set DEVLIST = ( $DEVLIST '"usb1740,9702"' )	# EnGenius 
set DEVLIST = ( $DEVLIST '"usb7392,7718"' )	# Edimax
set DEVLIST = ( $DEVLIST '"usb7392,7717"' )	# Edimax
set DEVLIST = ( $DEVLIST '"usb1044,800b"' )	# Gigabyte 
set DEVLIST = ( $DEVLIST '"usbe66,1"' )		# Hawking 
set DEVLIST = ( $DEVLIST '"usbe66,3"' )		# Hawking 
set DEVLIST = ( $DEVLIST '"usb1737,70"' )	# Linksys WUSB100 
set DEVLIST = ( $DEVLIST '"usb1737,71"' )	# Linksys WUSB600N 
set DEVLIST = ( $DEVLIST '"usb789,162"' )	# Logitec 2870 
set DEVLIST = ( $DEVLIST '"usb789,163"' )	# Logitec 2870 
set DEVLIST = ( $DEVLIST '"usb177f,302"' )	# lsusb 
set DEVLIST = ( $DEVLIST '"usbdf6,39"' )	# Sitecom 2770 
set DEVLIST = ( $DEVLIST '"usbdf6,17"' )	# Sitecom 
set DEVLIST = ( $DEVLIST '"usbdf6,2b"' )	# Sitecom 
set DEVLIST = ( $DEVLIST '"usbdf6,2c"' )	# Sitecom 
set DEVLIST = ( $DEVLIST '"usbdf6,2d"' )	# Sitecom 
set DEVLIST = ( $DEVLIST '"usb471,200f"' )	# Philips 
set DEVLIST = ( $DEVLIST '"usb2019,ed06"' )	# Planex Communications, Inc. 
set DEVLIST = ( $DEVLIST '"usb4e8,2018"' )	# samsung 
set DEVLIST = ( $DEVLIST '"usb129b,1828"' )	# Siemens 
set DEVLIST = ( $DEVLIST '"usb83A,b522"' )	# SMC 
set DEVLIST = ( $DEVLIST '"usb83a,a618"' )	# SMC 
set DEVLIST = ( $DEVLIST '"usb15a9,6"' )	# Sparklan 
set DEVLIST = ( $DEVLIST '"usb157e,300e"' )	# U-Media 
set DEVLIST = ( $DEVLIST '"usbcde,22"' )	# ZCOM 
set DEVLIST = ( $DEVLIST '"usb5a57,280"' )	# Zinwell 
set DEVLIST = ( $DEVLIST '"usb5a57,282"' )	# Zinwell 
set DEVLIST = ( $DEVLIST '"usb586,3416"' )	# Zyxel 
set DEVLIST = ( $DEVLIST '"usbcde,25"' )	# Zyxel 
#
# RT307x
#
set DEVLIST = ( $DEVLIST '"usb148f,3070"' )	# Ralink 3070 
set DEVLIST = ( $DEVLIST '"usb148f,3071"' )	# Ralink 3071 
set DEVLIST = ( $DEVLIST '"usb148f,3072"' )	# Ralink 3072 
set DEVLIST = ( $DEVLIST '"usbdb0,3820"' )	# Ralink 3070 
set DEVLIST = ( $DEVLIST '"usb7b8,3070"' )	# AboCom 3070 
set DEVLIST = ( $DEVLIST '"usb7b8,3071"' )	# AboCom 3071 
set DEVLIST = ( $DEVLIST '"usb7b8,3072"' )	# Abocom 3072 
set DEVLIST = ( $DEVLIST '"usb1eda,2310"' )	# AirTies 3070 
set DEVLIST = ( $DEVLIST '"usb14b2,3c12"' )	# AL 3070 
set DEVLIST = ( $DEVLIST '"usb83a,7511"' )	# Arcadyan 3070 
set DEVLIST = ( $DEVLIST '"usb13d3,3273"' )	# AzureWave 3070 
set DEVLIST = ( $DEVLIST '"usb18c5,12"' )	# Corega 3070 
set DEVLIST = ( $DEVLIST '"usb7d1,3c0d"' )	# D-Link 3070 
set DEVLIST = ( $DEVLIST '"usb7d1,3c0e"' )	# D-Link 3070 
set DEVLIST = ( $DEVLIST '"usb7d1,3c0f"' )	# D-Link 3070 
set DEVLIST = ( $DEVLIST '"usb7d1,3c0a"' )	# D-Link 3072 
set DEVLIST = ( $DEVLIST '"usb2001,3c09"' )	# D-Link 
set DEVLIST = ( $DEVLIST '"usb2001,3c0a"' )	# D-Link 3072 
set DEVLIST = ( $DEVLIST '"usb7392,7711"' )	# Edimax 3070 
set DEVLIST = ( $DEVLIST '"usb203d,1480"' )	# Encore 3070 
set DEVLIST = ( $DEVLIST '"usb1740,9703"' )	# EnGenius 3070 
set DEVLIST = ( $DEVLIST '"usb1740,9705"' )	# EnGenius 3071 
set DEVLIST = ( $DEVLIST '"usb1740,9706"' )	# EnGenius 3072 
set DEVLIST = ( $DEVLIST '"usb1044,800d"' )	# Gigabyte GN-WB32L 3070 
set DEVLIST = ( $DEVLIST '"usb4bb,945"' )	# I-O DATA 3072 
set DEVLIST = ( $DEVLIST '"usb1737,77"' )	# Linksys WUSB54GC-EU v3 
#set DEVLIST = ( $DEVLIST '"usb789,164"' )	# Logitec LAN-W150N/U2PSP
#set DEVLIST = ( $DEVLIST '"usb789,168"' )	# Logitec LAN-W150/U2BK
set DEVLIST = ( $DEVLIST '"usb1d4d,c"' )	# Pegatron Corporation 3070 
set DEVLIST = ( $DEVLIST '"usb1d4d,e"' )	# Pegatron Corporation 3070 
#set DEVLIST = ( $DEVLIST '"usb2019,ab25"' )	# Planex Communications, Inc.
#set DEVLIST = ( $DEVLIST '"usb2019,ed14"' )	# Planex Communications, Inc. 
set DEVLIST = ( $DEVLIST '"usb1a32,304"' )	# Quanta 3070 
set DEVLIST = ( $DEVLIST '"usbdf6,3f"' )	# Sitecom WL-608 
set DEVLIST = ( $DEVLIST '"usbdf6,3e"' )	# Sitecom 3070 
set DEVLIST = ( $DEVLIST '"usbdf6,42"' )	# Sitecom 3072 
set DEVLIST = ( $DEVLIST '"usb5a57,5257"' )	# Zinwell 3070 
set DEVLIST = ( $DEVLIST '"usb5a57,283"' )	# Zinwell 3072 
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

set existing = `grep "run " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" run
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" run
endif
sync
