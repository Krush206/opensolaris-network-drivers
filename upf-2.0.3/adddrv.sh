#! /bin/csh -f
#
# upf: an986/adm8511/adm8513/adm8515 driver
set DEVLIST = ( )

#
# Tested USB devices
#
set DEVLIST = ( $DEVLIST '"usb7a6,8511"' )	#  ADMtek ADM8511 Pegasus II
set DEVLIST = ( $DEVLIST '"usb411,5"' )		#  MELCO/BUFFALO LUA-TX
set DEVLIST = ( $DEVLIST '"usb4bb,904"' )	#  IO DATA USB ET/TX
set DEVLIST = ( $DEVLIST '"usb4bb,93a"' )	#  IO DATA USB ETX2-US2
#
# Untested USB devices
#
set DEVLIST = ( $DEVLIST '"usb506,4601"' )	#  3Com USB Ethernet 3C460B
set DEVLIST = ( $DEVLIST '"usb557,2007"' )	#  ATEN USB Ethernet UC-110T
set DEVLIST = ( $DEVLIST '"usb7b8,110c"' )	#  USB HPNA/Ethernet
set DEVLIST = ( $DEVLIST '"usb7b8,4104"' )	#  USB HPNA/Ethernet
set DEVLIST = ( $DEVLIST '"usb7b8,4004"' )	#  USB HPNA/Ethernet
set DEVLIST = ( $DEVLIST '"usb7b8,4007"' )	#  USB HPNA/Ethernet
set DEVLIST = ( $DEVLIST '"usb7b8,4102"' )	#  USB 10/100 Fast Ethernet
set DEVLIST = ( $DEVLIST '"usb7b8,4002"' )	#  USB 10/100 Fast Ethernet
set DEVLIST = ( $DEVLIST '"usb7b8,400b"' )	#  USB 10/100 Fast Ethernet
set DEVLIST = ( $DEVLIST '"usb7b8,400c"' )	#  USB 10/100 Fast Ethernet
set DEVLIST = ( $DEVLIST '"usb7b8,abc1"' )	#  USB 10/100 Fast Ethernet
set DEVLIST = ( $DEVLIST '"usb7b8,200c"' )	#  USB 10/100 Fast Ethernet" 	
set DEVLIST = ( $DEVLIST '"usb83a,1046"' )	#  Accton USB 10/100
set DEVLIST = ( $DEVLIST '"usb83a,5046"' )	#  SpeedStream USB 10/100
set DEVLIST = ( $DEVLIST '"usb83a,b004"' )	#  Philips USB 10/100 Ethernet
set DEVLIST = ( $DEVLIST '"usb7a6,8513"' )	#  ADMtek ADM8513 Pegasus II
set DEVLIST = ( $DEVLIST '"usb7a6,8515"' )	#  ADMtek ADM8515 Pegasus II
set DEVLIST = ( $DEVLIST '"usb7a6,986"' )	#  ADMtek AN986 Pegasus USB
set DEVLIST = ( $DEVLIST '"usb7a6,1986"' )	#  AN986A USB MAC
set DEVLIST = ( $DEVLIST '"usb334,1701"' )	#  AEI USB Fast Ethernet Adapter
set DEVLIST = ( $DEVLIST '"usb7c9,b100"' )	#  Allied Telesyn Int. AT-USB100
set DEVLIST = ( $DEVLIST '"usb50d,121"' )	#  Belkin F5D5050 USB Ethernet
set DEVLIST = ( $DEVLIST '"usb8dd,986"' )	#  Billionton USB-100
set DEVLIST = ( $DEVLIST '"usb8dd,987"' )	#  Billionton USBLP-100
set DEVLIST = ( $DEVLIST '"usb8dd,988"' )	#  Billionton USBEL-100
set DEVLIST = ( $DEVLIST '"usb8dd,8511"' )	#  Billionton USBE-100
set DEVLIST = ( $DEVLIST '"usb49f,8511"' )	#  iPAQ Networking 10/100 USB
set DEVLIST = ( $DEVLIST '"usb7aa,4"' )		#  Corega FEter USB-TX
set DEVLIST = ( $DEVLIST '"usb7aa,d"' )		#  Corega FEter USB-TXS
set DEVLIST = ( $DEVLIST '"usb2001,4001"' )	#  D-Link DSB-650TX
set DEVLIST = ( $DEVLIST '"usb2001,4002"' )	#  D-Link DSB-650TX
set DEVLIST = ( $DEVLIST '"usb2001,4102"' )	#  D-Link DSB-650TX
set DEVLIST = ( $DEVLIST '"usb2001,400b"' )	#  D-Link DSB-650TX
set DEVLIST = ( $DEVLIST '"usb2001,200c"' )	#  D-Link DSB-650TX
set DEVLIST = ( $DEVLIST '"usb2001,4003"' )	#  D-Link DSB-650TX(PNA)
set DEVLIST = ( $DEVLIST '"usb2001,abc1"' )	#  D-Link DSB-650
set DEVLIST = ( $DEVLIST '"usbdb7,2"' )		#  GOLDPFEIL USB Adapter
set DEVLIST = ( $DEVLIST '"usb1342,304"' )	#  EasiDock Ethernet
set DEVLIST = ( $DEVLIST '"usb56e,4010"' )	#  ELECOM USB Ethernet LD-USB20
set DEVLIST = ( $DEVLIST '"usb5cc,3000"' )	#  Elsa Micolink USB2Ethernet
set DEVLIST = ( $DEVLIST '"usb1044,8002"' )	#  GIGABYTE GN-BR402W Wireless Router
set DEVLIST = ( $DEVLIST '"usbe66,400c"' )	#  Hawking UF100 10/100 Ethernet
set DEVLIST = ( $DEVLIST '"usb3f0,811c"' )	#  HP hn210c Ethernet USB
set DEVLIST = ( $DEVLIST '"usb4bb,913"' )	#  IO DATA USB ET/TX-S
set DEVLIST = ( $DEVLIST '"usb951,a"' )		#  Kingston KNU101TX Ethernet
set DEVLIST = ( $DEVLIST '"usb56e,4002"' )	#  LANEED USB Ethernet LD-USB/TX
set DEVLIST = ( $DEVLIST '"usb56e,4005"' )	#  LANEED USB Ethernet LD-USBL/TX
set DEVLIST = ( $DEVLIST '"usb56e,400b"' )	#  LANEED USB Ethernet LD-USB/TX
set DEVLIST = ( $DEVLIST '"usb56e,abc1"' )	#  LANEED USB Ethernet LD-USB/T
set DEVLIST = ( $DEVLIST '"usb56e,200c"' )	#  LANEED USB Ethernet LD-USB/TX
set DEVLIST = ( $DEVLIST '"usb66b,2202"' )	#  Linksys USB10TX
set DEVLIST = ( $DEVLIST '"usb66b,2203"' )	#  Linksys USB100TX
set DEVLIST = ( $DEVLIST '"usb66b,2204"' )	#  Linksys USB100TX
set DEVLIST = ( $DEVLIST '"usb66b,2206"' )	#  Linksys USB10T
set DEVLIST = ( $DEVLIST '"usb66b,8b4"' )	#  Linksys USBVPN1
set DEVLIST = ( $DEVLIST '"usb66b,400b"' )	#  Linksys USB USB100TX
set DEVLIST = ( $DEVLIST '"usb66b,200c"' )	#  Linksys USB10TX"	
set DEVLIST = ( $DEVLIST '"usb411,1"' )		#  MELCO/BUFFALO LUA-TX
set DEVLIST = ( $DEVLIST '"usb411,9"' )		#  MELCO/BUFFALO LUA2-TX
set DEVLIST = ( $DEVLIST '"usb45e,7a"' )	#  Microsoft MN-110
set DEVLIST = ( $DEVLIST '"usb846,1020"' )	#  NETGEAR FA101
set DEVLIST = ( $DEVLIST '"usbb3p,109"' )	#  OCT
set DEVLIST = ( $DEVLIST '"usbb3p,901"' )	#  OCT usb to Ethernet
set DEVLIST = ( $DEVLIST '"usb8d1,3"' )		#  smartNIC 2 PnP Adapter
set DEVLIST = ( $DEVLIST '"usb707,200"' )	#  SMC 202 USB Ethernet
set DEVLIST = ( $DEVLIST '"usb707,201"' )	#  SMC 2206 USB Ethernet
set DEVLIST = ( $DEVLIST '"usb15e8,9100"' )	#  SOHOware NUB100 Ethernet
set DEVLIST = ( $DEVLIST '"usb15e8,9110"' )	#  SOHOware NUB110 Ethernet
set DEVLIST = ( $DEVLIST '"usb67c,1001"' )	#  SpeedStream USB 10/100

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

set existing = `grep "upf " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" upf
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" upf
endif
sync
