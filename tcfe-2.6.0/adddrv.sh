#! /bin/csh -f
#
# tcfe: 3com 3c90x driver
set DEVLIST = ( )

# 3C59X
set DEVLIST = ( $DEVLIST '"pci10B7,5900"' )	#  3C590
set DEVLIST = ( $DEVLIST '"pci10B7,5920"' )	#  3C592
set DEVLIST = ( $DEVLIST '"pci10B7,5970"' )	#  3C597
set DEVLIST = ( $DEVLIST '"pci10B7,5950"' )	#  3C595
set DEVLIST = ( $DEVLIST '"pci10B7,5951"' )	#  3C595
set DEVLIST = ( $DEVLIST '"pci10B7,5952"' )	#  3C595

# 3C900 10M
set DEVLIST = ( $DEVLIST '"pci10b7,9000"' )	#  3C900 10baseT
set DEVLIST = ( $DEVLIST '"pci10b7,9001"' )	#  3C900 10Mbps combo

# cyclone 10M
set DEVLIST = ( $DEVLIST '"pci10b7,9004"' )	#  3C900 10Mbps TPO
set DEVLIST = ( $DEVLIST '"pci10b7,9005"' )	#  3C900 10Mbps combo

# tornade 10M
set DEVLIST = ( $DEVLIST '"pci10b7,9006"' )	#  3C900 10Mbps TPC
set DEVLIST = ( $DEVLIST '"pci10b7,900a"' )	#  3C900B-FL 10base-FL

# boomerang 100M
set DEVLIST = ( $DEVLIST '"pci10b7,9050"' )	#  3C905 100baseTx
set DEVLIST = ( $DEVLIST '"pci10b7,9051"' )	#  3C905 100baseT4

# cyclone 100M
set DEVLIST = ( $DEVLIST '"pci10b7,9055"' )	#  3C905B 100baseTx
set DEVLIST = ( $DEVLIST '"pci10b7,9058"' )	#  3C905B 10/100/BNC
set DEVLIST = ( $DEVLIST '"pci10b7,905a"' )	#  3C905B-FX 100baseFx

# tornade 100M
set DEVLIST = ( $DEVLIST '"pci10b7,9200"' )	#  3C905C
set DEVLIST = ( $DEVLIST '"pci10b7,9202"' )	#  3C920B-EMB-WNM (ATI)
set DEVLIST = ( $DEVLIST '"pci10b7,9201"' )	#  3C920

# 980 series cyclone
set DEVLIST = ( $DEVLIST '"pci10b7,9800"' )	#  3C980
set DEVLIST = ( $DEVLIST '"pci10b7,9805"' )	#  3C980C Python

# SOHO series tornade
set DEVLIST = ( $DEVLIST '"pci10b7,7646"' )	#  3CSOHO100-TX

# mini-pci 555/556w/modem series
set DEVLIST = ( $DEVLIST '"pci10b7,5055"' )	#  3C555
set DEVLIST = ( $DEVLIST '"pci10b7,6055"' )	#  3C556 
set DEVLIST = ( $DEVLIST '"pci10b7,6056"' )	#  3C556B

# cardbus 575 series
set DEVLIST = ( $DEVLIST '"pci10b7,5b57"' )	#  3C575
set DEVLIST = ( $DEVLIST '"pci10b7,5057"' )	#  3C575
set DEVLIST = ( $DEVLIST '"pci10b7,5157"' )	#  3C575BT
set DEVLIST = ( $DEVLIST '"pci10b7,5257"' )	#  3C575CT

# cardbus 656 series
set DEVLIST = ( $DEVLIST '"pci10b7,6560"' )	#  3C656
set DEVLIST = ( $DEVLIST '"pci10b7,6562"' )	#  3C656B
set DEVLIST = ( $DEVLIST '"pci10b7,6564"' )	#  3C656C
set DEVLIST = ( $DEVLIST '"pci10b7,4500"' )	#  3C450

# tornade chipset
set DEVLIST = ( $DEVLIST '"pci10b7,1201"' )	#  3C982
set DEVLIST = ( $DEVLIST '"pci10b7,1202"' )	#  3C982
set DEVLIST = ( $DEVLIST '"pci10b7,9056"' )	#  3C905BT4
set DEVLIST = ( $DEVLIST '"pci10b7,9210"' )	#  3C920B-EMB-WNM

#echo $DEVLIST

set DEVLIST2 = ( )
foreach i ($DEVLIST)
	set pcidev = `grep $i /etc/driver_aliases`
#	echo $pcidev
	if ("$pcidev" == "") then
		set DEVLIST2 = ( $DEVLIST2 "$i" )
	endif
end

#echo $DEVLIST2
if ("$DEVLIST2" == "") then
	echo nothing to do.
	exit 1
endif

set existing = `grep "tcfe " /etc/driver_aliases`
echo $existing
if ("$existing" == "") then
	/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST2" tcfe
else 
	/usr/sbin/update_drv -a -v -m '* 0600 root sys' -i "$DEVLIST2" tcfe
endif
sync
