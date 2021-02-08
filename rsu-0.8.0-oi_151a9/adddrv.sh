#! /bin/csh -f
#
set DEVLIST = ( )

#
# Tested USB devices
#
set DEVLIST = ( $DEVLIST '"usb7aa,47"' )	# Corega WLUSBNM
set DEVLIST = ( $DEVLIST '"usb2019,ed16"' )	# Planex GW-USmicroN2W

#
# Untested USB devices
#

set DEVLIST = ( $DEVLIST '"usbbda,8171"' )
set DEVLIST = ( $DEVLIST '"usbbda,8173"' )
set DEVLIST = ( $DEVLIST '"usbbda,8712"' )
set DEVLIST = ( $DEVLIST '"usbbda,8713"' )
set DEVLIST = ( $DEVLIST '"usbbda,c512"' )
set DEVLIST = ( $DEVLIST '"usb7b8,8188"' )
set DEVLIST = ( $DEVLIST '"usbb05,1786"' )
set DEVLIST = ( $DEVLIST '"usbb05,1791"' )
set DEVLIST = ( $DEVLIST '"usb50d,945a"' )
# set DEVLIST = ( $DEVLIST '"usb7aa,47"' )
set DEVLIST = ( $DEVLIST '"usb2001,3306"' )
set DEVLIST = ( $DEVLIST '"usb7d1,3306"' )
set DEVLIST = ( $DEVLIST '"usb7392,7611"' )
set DEVLIST = ( $DEVLIST '"usb1740,9603"' )
set DEVLIST = ( $DEVLIST '"usbe66,16"' )
set DEVLIST = ( $DEVLIST '"usb6f8,e034"' )
set DEVLIST = ( $DEVLIST '"usb6f8,e032"' )
set DEVLIST = ( $DEVLIST '"usb789,167"' )
set DEVLIST = ( $DEVLIST '"usb2019,ab28"' )
# set DEVLIST = ( $DEVLIST '"usb2019,ed16"' )
set DEVLIST = ( $DEVLIST '"usbdf6,57"' )
set DEVLIST = ( $DEVLIST '"usbdf6,45"' )
set DEVLIST = ( $DEVLIST '"usbdf6,59"' )
set DEVLIST = ( $DEVLIST '"usbdf6,4b"' )
set DEVLIST = ( $DEVLIST '"usbdf6,63"' )
set DEVLIST = ( $DEVLIST '"usb177f,154"' )
set DEVLIST = ( $DEVLIST '"usbbda,5077"' )
set DEVLIST = ( $DEVLIST '"usb1690,752"' )
set DEVLIST = ( $DEVLIST '"usb20f4,646b"' )
set DEVLIST = ( $DEVLIST '"usb83a,c512"' )
set DEVLIST = ( $DEVLIST '"usbbda,8172"' )
set DEVLIST = ( $DEVLIST '"usbeb0,9061"' )
set DEVLIST = ( $DEVLIST '"usbbda,8172"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3323"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3311"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3342"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3333"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3334"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3335"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3336"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3309"' )
set DEVLIST = ( $DEVLIST '"usb50d,815f"' )
set DEVLIST = ( $DEVLIST '"usb7d1,3302"' )
set DEVLIST = ( $DEVLIST '"usb7d1,3300"' )
set DEVLIST = ( $DEVLIST '"usb7d1,3303"' )
set DEVLIST = ( $DEVLIST '"usb7392,7612"' )
set DEVLIST = ( $DEVLIST '"usb1740,9605"' )
set DEVLIST = ( $DEVLIST '"usb6f8,e031"' )
set DEVLIST = ( $DEVLIST '"usbe66,15"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3306"' )
set DEVLIST = ( $DEVLIST '"usb2019,ed18"' )
set DEVLIST = ( $DEVLIST '"usb2019,4901"' )
set DEVLIST = ( $DEVLIST '"usbdf6,58"' )
set DEVLIST = ( $DEVLIST '"usbdf6,49"' )
set DEVLIST = ( $DEVLIST '"usbdf6,4c"' )
set DEVLIST = ( $DEVLIST '"usbdf6,64"' )
set DEVLIST = ( $DEVLIST '"usb14b2,3300"' )
set DEVLIST = ( $DEVLIST '"usb14b2,3301"' )
set DEVLIST = ( $DEVLIST '"usb14b2,3302"' )
set DEVLIST = ( $DEVLIST '"usb4f2,aff2"' )
set DEVLIST = ( $DEVLIST '"usb4f2,aff5"' )
set DEVLIST = ( $DEVLIST '"usb4f2,aff6"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3339"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3340"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3341"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3310"' )
set DEVLIST = ( $DEVLIST '"usb13d3,3325"' )
set DEVLIST = ( $DEVLIST '"usbbda,8174"' )
set DEVLIST = ( $DEVLIST '"usbbda,8174"' )
set DEVLIST = ( $DEVLIST '"usb50d,845a"' )
set DEVLIST = ( $DEVLIST '"usb7aa,51"' )
set DEVLIST = ( $DEVLIST '"usb7392,7622"' )
set DEVLIST = ( $DEVLIST '"usb409,2b6"' )
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

set existing = `grep "rsu " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" rsu
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" rsu
endif
sync
