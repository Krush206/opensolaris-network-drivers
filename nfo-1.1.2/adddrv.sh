#! /bin/csh -fv
#
# nfo: nforce built-in GbE  driver
set DEVLIST = ( )

set DEVLIST = ( $DEVLIST '"pci10de,1c3"' )
set DEVLIST = ( $DEVLIST '"pci10de,66"' )
set DEVLIST = ( $DEVLIST '"pci10de,d6"' )
set DEVLIST = ( $DEVLIST '"pci10de,86"' )
set DEVLIST = ( $DEVLIST '"pci10de,8c"' )
set DEVLIST = ( $DEVLIST '"pci10de,e6"' )
set DEVLIST = ( $DEVLIST '"pci10de,df"' )
set DEVLIST = ( $DEVLIST '"pci10de,56"' )
set DEVLIST = ( $DEVLIST '"pci10de,57"' )
set DEVLIST = ( $DEVLIST '"pci10de,37"' )
set DEVLIST = ( $DEVLIST '"pci10de,268"' )	# MCP51
set DEVLIST = ( $DEVLIST '"pci10de,269"' )	# MCP51
set DEVLIST = ( $DEVLIST '"pci10de,372"' )	# MCP55
set DEVLIST = ( $DEVLIST '"pci10de,373"' )	# MCP55

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

set existing = `grep "nfo " /etc/driver_aliases`
echo $existing
if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" nfo
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" nfo
endif
sync
