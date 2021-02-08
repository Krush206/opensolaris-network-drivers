#! /bin/csh -fv
#
# NIC driver for
#   digital 21140, 21140A, 21142, 2143
#   ADMtek 983/983B, Davicom DM9102/9102A, Macronix MX98715
#
# Tested PCI cards
#   Corega ETHER-PCI TM (ADMtek 983B)
#   Kuroto-shikou 9102A-PCI (Davicom DM9102A)
#   Melco BUFFALO (Macronix MX98715)
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pci1317,9511"')	# ADMtek ADM9511
set DEVLIST = ($DEVLIST '"pci1317,9513"')	# ADMtek ADM9513
set DEVLIST = ($DEVLIST '"pci1317,981"')	# ADMtek AL981 (comet)
set DEVLIST = ($DEVLIST '"pci1317,985"')	# ADMtek AN983/AN983B
set DEVLIST = ($DEVLIST '"pci1317,1985"')	# ADMtek AN985
set DEVLIST = ($DEVLIST '"pci13d1,ab02,"')	# ADMtek AN985 (AboCom)
set DEVLIST = ($DEVLIST '"pci1259,a120"')	# special for Corega EtherPCI TM
#set DEVLIST = ($DEVLIST '"pci1282,9100"')	# Davicom 9100
set DEVLIST = ($DEVLIST '"pci1282,9102"')	# Davicom 9102/9102A
set DEVLIST = ($DEVLIST '"pci10d9,512"')	# Macronix MX98713
set DEVLIST = ($DEVLIST '"pci10d9,531"')	# Macronix MX98715/25
set DEVLIST = ($DEVLIST '"pci1011,9"')		# digital 21140/40A
set DEVLIST = ($DEVLIST '"pci1011,19"')		# digital 21142/43
set DEVLIST = ($DEVLIST '"pci14f1,1803"')	# Conexant RS7112
set DEVLIST = ($DEVLIST '"pci11ad,2"')		# LC82C168 PNIC
set DEVLIST = ($DEVLIST '"pci11ad,c115"')	# LC82C115 PNIC II
set DEVLIST = ($DEVLIST '"pci115d,3"')		# Xircom CBE-100
set DEVLIST = ($DEVLIST '"pci1109,1400"')	# Cogent/Adaptec
set DEVLIST = ($DEVLIST '"pci1109,2400"')	# Cogent/Adaptec
set DEVLIST = ($DEVLIST '"pci10b8,2001"')	# SMSC
set DEVLIST = ($DEVLIST '"pci2646,1"')		# Kingstone
set DEVLIST = ($DEVLIST '"pci2646,2"')		# Kingstone pccard
set DEVLIST = ($DEVLIST '"pci10b9,5261"')	# ULi integrated nic
set DEVLIST = ($DEVLIST '"pci10b9,5263"')	# ULi integrated nic

#echo $DEVLIST

set DEVLIST2 = ( )
foreach i ($DEVLIST)
	set pcidev = `grep $i /etc/driver_aliases`
	echo $pcidev
	if ("$pcidev" == "") then
		set DEVLIST2 = ( $DEVLIST2 "$i" )
	endif
end

if ("$DEVLIST2" == "") then
        echo nothing to do.
        exit 1
endif

set existing = `grep "^tu " /etc/driver_aliases`
echo $existing
if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" tu
else
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" tu
endif
sync
