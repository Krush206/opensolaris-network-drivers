#! /bin/csh -f
#
# axg ax88179/178a driver
set DEVLIST = ( )

#
# Tested USB devices
#
set DEVLIST = ( $DEVLIST '"usbb95,1790"' )	# AXIS ax88179 generic
#
# Untested USB devices
#
set DEVLIST = ( $DEVLIST '"usbb95,178a"' )	# AXIS ax88178a generic
set DEVLIST = ( $DEVLIST '"usbdf6,72"' )	# Sitecom USB3.0 to Gigabit
set DEVLIST = ( $DEVLIST '"usb17ef,304b"' )	# ThinkPad OneLinkDoc
set DEVLIST = ( $DEVLIST '"usb930,a13"' )	# Toshiba USB3.0 to Gigabit LAN
set DEVLIST = ( $DEVLIST '"usb4e8,a100"' )	# Samsung USB ethernet


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

set existing = `grep "axg " /etc/driver_aliases`
#echo $existing

if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" axg
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" axg
endif
sync
