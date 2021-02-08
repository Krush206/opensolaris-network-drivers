#! /bin/csh -f
#
# nfo: nforce built-in fast/gigabit ethernet driver
#
set DRV = nfo
set DEVLIST = ( )

set DEVLIST = ( $DEVLIST '"pci10de,1c3"' )	# MCP
set DEVLIST = ( $DEVLIST '"pci10de,66"' )	# MCP-R
set DEVLIST = ( $DEVLIST '"pci10de,d6"' )	# nForce3 150
set DEVLIST = ( $DEVLIST '"pci10de,86"' )	# nForce2 Ultra
set DEVLIST = ( $DEVLIST '"pci10de,8c"' )	# nForce2 Ultra
set DEVLIST = ( $DEVLIST '"pci10de,e6"' )	# nForce3 250Gb
set DEVLIST = ( $DEVLIST '"pci10de,df"' )	# nForce3 250Gb

set DEVLIST = ( $DEVLIST '"pci10de,56"' )	# CK804
set DEVLIST = ( $DEVLIST '"pci10de,57"' )	# CK804

set DEVLIST = ( $DEVLIST '"pci10de,37"' )	# MCP04
set DEVLIST = ( $DEVLIST '"pci10de,38"' )	# MCP04

set DEVLIST = ( $DEVLIST '"pci10de,268"' )	# MCP51
set DEVLIST = ( $DEVLIST '"pci10de,269"' )	# MCP51

set DEVLIST = ( $DEVLIST '"pci10de,372"' )	# MCP55
set DEVLIST = ( $DEVLIST '"pci10de,373"' )	# MCP55

set DEVLIST = ( $DEVLIST '"pci10de,3e5"' )	# MCP61
set DEVLIST = ( $DEVLIST '"pci10de,3e6"' )	# MCP61
set DEVLIST = ( $DEVLIST '"pci10de,3ee"' )	# MCP61
set DEVLIST = ( $DEVLIST '"pci10de,3ef"' )	# MCP61

set DEVLIST = ( $DEVLIST '"pci10de,450"' )	# MCP65
set DEVLIST = ( $DEVLIST '"pci10de,451"' )	# MCP65
set DEVLIST = ( $DEVLIST '"pci10de,452"' )	# MCP65
set DEVLIST = ( $DEVLIST '"pci10de,453"' )	# MCP65

set DEVLIST = ( $DEVLIST '"pci10de,54c"' )	# MCP67
set DEVLIST = ( $DEVLIST '"pci10de,54d"' )	# MCP67
set DEVLIST = ( $DEVLIST '"pci10de,54e"' )	# MCP67
set DEVLIST = ( $DEVLIST '"pci10de,54f"' )	# MCP67

set DEVLIST = ( $DEVLIST '"pci10de,7dc"' )	# MCP73
set DEVLIST = ( $DEVLIST '"pci10de,7dd"' )	# MCP73
set DEVLIST = ( $DEVLIST '"pci10de,7de"' )	# MCP73
set DEVLIST = ( $DEVLIST '"pci10de,7df"' )	# MCP73

set DEVLIST = ( $DEVLIST '"pci10de,760"' )	# MCP77 GeForce 8200
set DEVLIST = ( $DEVLIST '"pci10de,761"' )	# MCP77
set DEVLIST = ( $DEVLIST '"pci10de,762"' )	# MCP77
set DEVLIST = ( $DEVLIST '"pci10de,763"' )	# MCP77

set DEVLIST = ( $DEVLIST '"pci10de,ab0"' )	# MCP79
set DEVLIST = ( $DEVLIST '"pci10de,ab1"' )	# MCP79
set DEVLIST = ( $DEVLIST '"pci10de,ab2"' )	# MCP79
set DEVLIST = ( $DEVLIST '"pci10de,ab3"' )	# MCP79

set DEVLIST = ( $DEVLIST '"pci10de,d7d"' )	# MCP89


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
