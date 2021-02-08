#! /bin/csh -f
#
# urf: rtl8150 driver
set DEVLIST = ( )

#
# Tested USB devices
#
set DEVLIST = ( $DEVLIST '"usbbda,8150"' )	# realtek 8150 generic
set DEVLIST = ( $DEVLIST '"usb411,12"' )	# melco buffalo LUAKTX
#
# UnTested USB devices
#
set DEVLIST = ( $DEVLIST '"usb3980,3"' )	# micronet SP128AR
set DEVLIST = ( $DEVLIST '"usb7b8,401a"' )	# longshine LCS8138TX
set DEVLIST = ( $DEVLIST '"usb1557,8150"' )	# oqo rtl8150
set DEVLIST = ( $DEVLIST '"usb586,401a"' )	# zyxel prestige

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

set existing = `grep "urf " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" urf
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" urf
endif
sync
