#! /bin/csh -f
#
set DEVLIST = ( )

#
# Tested USB devices
#
set DEVLIST = ( $DEVLIST '"usb2019,ab2a"' )	# Planex GW-USNano2
set DEVLIST = ( $DEVLIST '"usb2019,ab2b"' )	# Planex GW-USEco300
set DEVLIST = ( $DEVLIST '"usb2019,ab2e"' )	# Oar Inc SW-WF02-AD15

#
# Untested USB devices
#
set DEVLIST = ( $DEVLIST '"usb2019,ed17"' )	# Planex GW-USWExtreme

set DEVLIST = ( $DEVLIST '"usbbda,8191"' )	# Realtek demoboard

						# 8188CU
set DEVLIST = ( $DEVLIST '"usbbda,8176"' )	# 8188cu 1*1 dongole
set DEVLIST = ( $DEVLIST '"usbbda,8177"' )	# 8188cu 1*1 dongole, (b/g mode only)
set DEVLIST = ( $DEVLIST '"usbbda,8170"' )	# 8188CE-VAU USB minCard
set DEVLIST = ( $DEVLIST '"usbbda,817a"' )	# 8188cu Slim Solo
set DEVLIST = ( $DEVLIST '"usbbda,817b"' )	# 8188cu Slim Combo
set DEVLIST = ( $DEVLIST '"usbbda,817d"' )	# 8188RU High-power USB Dongle
set DEVLIST = ( $DEVLIST '"usbbda,8754"' )	# 8188 Combo for BC4
set DEVLIST = ( $DEVLIST '"usbbda,817e"' )	# 8188CE-VAU USB minCard (b/g mode only)

						# 8192CU
#set DEVLIST = ( $DEVLIST '"usbbda,8177"' )	# 8191cu 1*2
set DEVLIST = ( $DEVLIST '"usbbda,8178"' )	# 8192cu 2*2
set DEVLIST = ( $DEVLIST '"usbbda,817c"' )	# 192CE-VAU USB minCard

						# 8188CU
set DEVLIST = ( $DEVLIST '"usb7b8,8189"' )	# Funai - Abocom
#set DEVLIST = ( $DEVLIST '"usb2019,ed17"' )	# PCI - Edimax
set DEVLIST = ( $DEVLIST '"usbdf6,52"' )	# Sitecom - Edimax
set DEVLIST = ( $DEVLIST '"usb7392,7811"' )	# Edimax - Edimax
set DEVLIST = ( $DEVLIST '"usb7b8,8188"' )	# Abocom - Abocom
set DEVLIST = ( $DEVLIST '"usbeb0,9071"' )	# NO Brand - Etop
set DEVLIST = ( $DEVLIST '"usb6f8,e033"' )	# Hercules - Edimax
set DEVLIST = ( $DEVLIST '"usb103c,1629"' )	# HP - Lite-On ,8188CUS Slim Combo
set DEVLIST = ( $DEVLIST '"usb2001,3308"' )	# D-Link - Alpha
set DEVLIST = ( $DEVLIST '"usb50d,1102"' )	# Belkin - Edimax
#set DEVLIST = ( $DEVLIST '"usb2019,ab2a"' )	# Planex - Abocom
set DEVLIST = ( $DEVLIST '"usb20f4,648b"' )	# TRENDnet - Cameo
set DEVLIST = ( $DEVLIST '"usb4855,90"' )	# Feixun

set DEVLIST = ( $DEVLIST '"usb3358,13d3"' )	# Azwave 8188CE-VAU
set DEVLIST = ( $DEVLIST '"usb3359,13d3"' )	# Azwave (8188CE-VAU  g mode)

						# 8192CU
#set DEVLIST = ( $DEVLIST '"usb7b8,8178"' )	# Funai -Abocom
set DEVLIST = ( $DEVLIST '"usb2001,3307"' )	# D-Link-Cameo
set DEVLIST = ( $DEVLIST '"usb2001,330a"' )	# D-Link-Alpha
set DEVLIST = ( $DEVLIST '"usb2001,3309"' )	# D-Link-Alpha
set DEVLIST = ( $DEVLIST '"usb0586,341f"' )	# Zyxel -Abocom
set DEVLIST = ( $DEVLIST '"usb7392,7822"' )	# Edimax -Edimax
#set DEVLIST = ( $DEVLIST '"usb2019,ab2b"' )	# Planex -Abocom
set DEVLIST = ( $DEVLIST '"usb7b8,8178"' )	# Abocom -Abocom
set DEVLIST = ( $DEVLIST '"usb7aa,56"' )	# ATKK-Gemtek
set DEVLIST = ( $DEVLIST '"usb4855,91"' )	# Feixun


set DEVLIST = ( $DEVLIST '"usb4f2,aff7"' )	# Chicony Electronics
set DEVLIST = ( $DEVLIST '"usb4f2,aff8"' )	# Chicony Electronics
set DEVLIST = ( $DEVLIST '"usb4f2,aff9"' )	# Chicony Electronics
set DEVLIST = ( $DEVLIST '"usb4f2,affa"' )	# Chicony Electronics
set DEVLIST = ( $DEVLIST '"usb4f2,affb"' )	# Chicony Electronics
set DEVLIST = ( $DEVLIST '"usb4f2,affc"' )	# Chicony Electronics
# set DEVLIST = ( $DEVLIST '"usb50d,1102"' )	# Belkin Components F7D1102 N150/Surf Micro Wireless Adapter v1000 [Realtek RTL8188CUS]
set DEVLIST = ( $DEVLIST '"usb50d,2102"' )	# Belkin Components (Device name unknown)
set DEVLIST = ( $DEVLIST '"usb50d,2103"' )	# Belkin Components F7D2102 802.11n N300 Micro Wireless Adapter v3000 [Realtek RTL8192CU]
#set DEVLIST = ( $DEVLIST '"usb586,341f"' )	# ZyXEL Communications Corp. NWD2205 802.11n Wireless N Adapter [Realtek RTL8192CU]
#set DEVLIST = ( $DEVLIST '"usb6f8,e033"' )	# Guillemot Corp. Hercules HWNUp-150 802.11n Wireless N Pico [Realtek RTL8188CUS]
#set DEVLIST = ( $DEVLIST '"usb7aa,56"' )	# Corega K.K. (Device name unknown)
#set DEVLIST = ( $DEVLIST '"usb7b8,8178"' )	# AboCom Systems Inc
#set DEVLIST = ( $DEVLIST '"usb7b8,8188"' )	# AboCom Systems Inc
#set DEVLIST = ( $DEVLIST '"usb7b8,8189"' )	# AboCom Systems Inc
set DEVLIST = ( $DEVLIST '"usb846,9021"' )	# NetGear, Inc.
set DEVLIST = ( $DEVLIST '"usb846,9041"' )	# NetGear, Inc. WNA1000M 802.11bgn [Realtek RTL8188CUS]
set DEVLIST = ( $DEVLIST '"usbb05,17ab"' )	# ASUSTek Computer, Inc. USB-N13 802.11n Network Adapter (rev. B1) [Realtek RTL8192CU]
set DEVLIST = ( $DEVLIST '"usbbda,18a"' )	# Realtek Semiconductor
set DEVLIST = ( $DEVLIST '"usbbda,317f"' )	# Realtek Semiconductor
set DEVLIST = ( $DEVLIST '"usbbda,5088"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbbda,8170"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbbda,8176"' )	# Realtek Semiconductor Corp. RTL8188CUS 802.11n WLAN Adapter
#set DEVLIST = ( $DEVLIST '"usbbda,8177"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbbda,8178"' )	# Realtek Semiconductor Corp. RTL8192CU 802.11n WLAN Adapter
#set DEVLIST = ( $DEVLIST '"usbbda,817a"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbbda,817b"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbbda,817c"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbbda,817d"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbbda,817e"' )	# Realtek Semiconductor
set DEVLIST = ( $DEVLIST '"usbbda,817f"' )	# Realtek Semiconductor Corp. RTL8188RU 802.11n WLAN Adapter
set DEVLIST = ( $DEVLIST '"usbbda,8186"' )	# Realtek Semiconductor
set DEVLIST = ( $DEVLIST '"usbbda,818a"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbbda,8191"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbbda,8754"' )	# Realtek Semiconductor
#set DEVLIST = ( $DEVLIST '"usbdf6,52"' )	# Sitecom Europe B.V.
set DEVLIST = ( $DEVLIST '"usbdf6,5c"' )	# Sitecom Europe B.V.
set DEVLIST = ( $DEVLIST '"usbdf6,61"' )	# Sitecom Europe B.V.
set DEVLIST = ( $DEVLIST '"usbe66,19"' )	# Hawking Technologies
#set DEVLIST = ( $DEVLIST '"usbeb0,9071"' )	# NovaTech
#set DEVLIST = ( $DEVLIST '"usb103c,1629"' )	#
set DEVLIST = ( $DEVLIST '"usb13d3,3357"' )	# IMC Networks
set DEVLIST = ( $DEVLIST '"usb13d3,3358"' )	# IMC Networks
set DEVLIST = ( $DEVLIST '"usb13d3,3359"' )	# IMC Networks
#set DEVLIST = ( $DEVLIST '"usb2001,3307"' )	# D-Link Corp.
#set DEVLIST = ( $DEVLIST '"usb2001,3308"' )	# D-Link Corp. DWA-121 802.11n Wireless N 150 Pico Adapter [Realtek RTL8188CUS]
#set DEVLIST = ( $DEVLIST '"usb2001,3309"' )	# D-Link Corp. DWA-135 802.11n Wireless N Adapter(rev.A1) [Realtek RTL8192CU]
#set DEVLIST = ( $DEVLIST '"usb2001,330a"' )	# D-Link Corp. DWA-133 802.11n Wireless N Adapter [Realtek RTL8192CU]
set DEVLIST = ( $DEVLIST '"usb2019,1201"' )	# PLANEX
set DEVLIST = ( $DEVLIST '"usb2019,4902"' )	# PLANEX
#set DEVLIST = ( $DEVLIST '"usb2019,ab2a"' )	# PLANEX GW-USNano2 802.11n Wireless Adapter [Realtek RTL8188CUS]
#set DEVLIST = ( $DEVLIST '"usb2019,ab2b"' )	# PLANEX
#set DEVLIST = ( $DEVLIST '"usb2019,ab2e"' )	# PLANEX
#set DEVLIST = ( $DEVLIST '"usb2019,ed17"' )	# PLANEX GW-USValue-EZ 802.11n Wireless Adapter [Realtek RTL8188CUS]
set DEVLIST = ( $DEVLIST '"usb20f4,624d"' )	# TRENDnet
#set DEVLIST = ( $DEVLIST '"usb20f4,648b"' )	# TRENDnet TEW-648UBM 802.11n 150Mbps Micro Wireless N Adapter [Realtek RTL8188CUS]
#set DEVLIST = ( $DEVLIST '"usb4855,90"' )	# Memorex
#set DEVLIST = ( $DEVLIST '"usb4855,91"' )	# Memorex
set DEVLIST = ( $DEVLIST '"usb4856,91"' )	#
#set DEVLIST = ( $DEVLIST '"usb7392,7811"' )	# Edimax Technology Co., Ltd EW-7811Un 802.11n Wireless Adapter [Realtek RTL8188CUS]
#set DEVLIST = ( $DEVLIST '"usb7392,7822"' )	# Edimax Technology Co., Ltd
set DEVLIST = ( $DEVLIST '"usb9846,9041"' )	#

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

set existing = `grep "urtwn " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" urtwn
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" urtwn
endif
sync
