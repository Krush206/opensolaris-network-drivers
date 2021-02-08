#! /bin/csh -f
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pci8086,1229"')	#
set DEVLIST = ($DEVLIST '"pci8086,1209"')	#
set DEVLIST = ($DEVLIST '"pci8086,1029"')	#
set DEVLIST = ($DEVLIST '"pci8086,1030"')	#
set DEVLIST = ($DEVLIST '"pci8086,1031"')	#
set DEVLIST = ($DEVLIST '"pci8086,1032"')	#
set DEVLIST = ($DEVLIST '"pci8086,1033"')	#
set DEVLIST = ($DEVLIST '"pci8086,1034"')	#
set DEVLIST = ($DEVLIST '"pci8086,1035"')	#
set DEVLIST = ($DEVLIST '"pci8086,1036"')	#
set DEVLIST = ($DEVLIST '"pci8086,1037"')	#
set DEVLIST = ($DEVLIST '"pci8086,1038"')	#
set DEVLIST = ($DEVLIST '"pci8086,1039"')	#
set DEVLIST = ($DEVLIST '"pci8086,103a"')	#
set DEVLIST = ($DEVLIST '"pci8086,103b"')	#
set DEVLIST = ($DEVLIST '"pci8086,103c"')	#
set DEVLIST = ($DEVLIST '"pci8086,103d"')	#
set DEVLIST = ($DEVLIST '"pci8086,103e"')	#
set DEVLIST = ($DEVLIST '"pci8086,1050"')	#
set DEVLIST = ($DEVLIST '"pci8086,1051"')	#
set DEVLIST = ($DEVLIST '"pci8086,1052"')	#
set DEVLIST = ($DEVLIST '"pci8086,1053"')	#
set DEVLIST = ($DEVLIST '"pci8086,1054"')	#
set DEVLIST = ($DEVLIST '"pci8086,1055"')	#
set DEVLIST = ($DEVLIST '"pci8086,1056"')	#
set DEVLIST = ($DEVLIST '"pci8086,1057"')	#
set DEVLIST = ($DEVLIST '"pci8086,1059"')	#
set DEVLIST = ($DEVLIST '"pci8086,1064"')	#
set DEVLIST = ($DEVLIST '"pci8086,1065"')	#
set DEVLIST = ($DEVLIST '"pci8086,1066"')	#
set DEVLIST = ($DEVLIST '"pci8086,1067"')	#
set DEVLIST = ($DEVLIST '"pci8086,1068"')	#
set DEVLIST = ($DEVLIST '"pci8086,1069"')	#
set DEVLIST = ($DEVLIST '"pci8086,106a"')	#
set DEVLIST = ($DEVLIST '"pci8086,106b"')	#
set DEVLIST = ($DEVLIST '"pci8086,1091"')	#
set DEVLIST = ($DEVLIST '"pci8086,1092"')	#
set DEVLIST = ($DEVLIST '"pci8086,1093"')	#
set DEVLIST = ($DEVLIST '"pci8086,1094"')	#
set DEVLIST = ($DEVLIST '"pci8086,1095"')	#
set DEVLIST = ($DEVLIST '"pci8086,1227"')	#
set DEVLIST = ($DEVLIST '"pci8086,1228"')	#
set DEVLIST = ($DEVLIST '"pci8086,2449"')	#
set DEVLIST = ($DEVLIST '"pci8086,2459"')	#
set DEVLIST = ($DEVLIST '"pci8086,245d"')	#
set DEVLIST = ($DEVLIST '"pci8086,27dc"')	#
set DEVLIST = ($DEVLIST '"pci8086,5200"')	#
set DEVLIST = ($DEVLIST '"pci8086,5201"')	#


# echo $DEVLIST

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

set existing = `grep "ife " /etc/driver_aliases`
echo $existing
if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" ife
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" ife
endif
sync
