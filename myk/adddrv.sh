#! /bin/csh -f
#
#
# Marvell yukon2 driver
#
set DEVLIST = ( )

#
#set DEVLIST = ( $DEVLIST '"pci10b7,1700"' )	# 3com 3c940
#set DEVLIST = ( $DEVLIST '"pci10b7,80eb"' )	# 3com 3c940b
#set DEVLIST = ( $DEVLIST '"pci1148,4320"' )	# SK-V2
#set DEVLIST = ( $DEVLIST '"pci1148,9000"' )	# SK-9SXX
#set DEVLIST = ( $DEVLIST '"pci1148,9e00"' )	# SK-9EXX

#set DEVLIST = ( $DEVLIST '"pci1148,9e01"' )	# SK-9EXX
#set DEVLIST = ( $DEVLIST '"pci1186,4001"' )	# DGE-550SX (pci)
#set DEVLIST = ( $DEVLIST '"pciex1186,4b00"' )	# DGE-560T (pci-e)
#set DEVLIST = ( $DEVLIST '"pciex1186,4b01"' )	# DGE-530T (pci)
#set DEVLIST = ( $DEVLIST '"pciex1186,4b02"' )	# DGE-560SX (pci-e)

#set DEVLIST = ( $DEVLIST '"pci1186,4b03"' )	# DGE-550T (pci)
#set DEVLIST = ( $DEVLIST '"pci1186,4c00"' )	# DGE-530T (pci)
#set DEVLIST = ( $DEVLIST '"pci11ab,4320"' )	# 88E8001/8003/8010
#set DEVLIST = ( $DEVLIST '"pci11ab,4340"' )	# 88E8021CU (pci-x)
#set DEVLIST = ( $DEVLIST '"pci11ab,4341"' )	# 88E8022CU (pci-x)

set DEVLIST = ( $DEVLIST '"pciex11ab,4342"' )	# 88E8061CU
set DEVLIST = ( $DEVLIST '"pciex11ab,4343"' )	# 88E8062CU
set DEVLIST = ( $DEVLIST '"pci11ab,4344"' )	# 88E8021SX (pci-x)
set DEVLIST = ( $DEVLIST '"pci11ab,4345"' )	# 88E8022SX (pci-x)
set DEVLIST = ( $DEVLIST '"pciex11ab,4346"' )	# 88E8061SX

set DEVLIST = ( $DEVLIST '"pciex11ab,4347"' )	# 88E8062SX
set DEVLIST = ( $DEVLIST '"pciex11ab,4350"' )	# 88E8035
set DEVLIST = ( $DEVLIST '"pciex11ab,4351"' )	# 88E8036
set DEVLIST = ( $DEVLIST '"pciex11ab,4352"' )	# 88E8038 (FE)
set DEVLIST = ( $DEVLIST '"pciex11ab,4353"' )	# 88E8039 (FE?) not worked

set DEVLIST = ( $DEVLIST '"pciex11ab,4354"' )	# 88E8040 (FE+)
set DEVLIST = ( $DEVLIST '"pciex11ab,4355"' )	# 88E8040T (FE?)
set DEVLIST = ( $DEVLIST '"pciex11ab,4356"' )	# 88EC033
set DEVLIST = ( $DEVLIST '"pciex11ab,4357"' )	# 88E8042 (FE?)
set DEVLIST = ( $DEVLIST '"pci11ab,435a"' )	# 88E8048 (FE?)

set DEVLIST = ( $DEVLIST '"pciex11ab,4360"' )	# 88E8052 (EC)
set DEVLIST = ( $DEVLIST '"pciex11ab,4361"' )	# 88E8050 (EC)
set DEVLIST = ( $DEVLIST '"pciex11ab,4362"' )	# 88E8053 (EC) worked
set DEVLIST = ( $DEVLIST '"pciex11ab,4363"' )	# 88E8055 worked
set DEVLIST = ( $DEVLIST '"pciex11ab,4364"' )	# 88E8056 worked

set DEVLIST = ( $DEVLIST '"pciex11ab,4365"' )	# 88E8070
set DEVLIST = ( $DEVLIST '"pciex11ab,4366"' )	# 88EC036
set DEVLIST = ( $DEVLIST '"pci11ab,4367"' )	# 88EC032
set DEVLIST = ( $DEVLIST '"pci11ab,4368"' )	# 88EC034
set DEVLIST = ( $DEVLIST '"pciex11ab,4369"' )	# 88EC042

set DEVLIST = ( $DEVLIST '"pciex11ab,436a"' )	# 88E8058 (EC Ultra)
set DEVLIST = ( $DEVLIST '"pciex11ab,436b"' )	# 88E8071
set DEVLIST = ( $DEVLIST '"pciex11ab,436c"' )	# 88E8072
set DEVLIST = ( $DEVLIST '"pciex11ab,436d"' )	# 88E8055
set DEVLIST = ( $DEVLIST '"pciex11ab,4370"' )	# 88E8075

set DEVLIST = ( $DEVLIST '"pciex11ab,4380"' )	# 88E8057 (Ultra2)
set DEVLIST = ( $DEVLIST '"pciex11ab,4381"' )	# 88E8059 (Optima)
#set DEVLIST = ( $DEVLIST '"pci11ab,5005"' )	# Belkin gigabit desktop
#set DEVLIST = ( $DEVLIST '"pci1371,434e"' )	# CNet (pci)
#set DEVLIST = ( $DEVLIST '"pci1737,1032"' )	# Lynksys EG1032 (pci-x)
#set DEVLIST = ( $DEVLIST '"pci1737,1064"' )	# Lynksys EG1064 (pci-x)

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

set existing = `grep "myk " /etc/driver_aliases`
echo $existing
if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" myk
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" myk
endif
sync
