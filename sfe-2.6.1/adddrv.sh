#! /bin/csh -f
#
# DP83815/SiS900/SiS7016 fast ethernet controler driver
#
# PCI vendoro-id/device-id
#  NS DP83815 100b/20   tested (Netgear F311  DP83815 revid 0)
#  SiS900     1039/900  tested (Melco inc. BUFFLO LGY-PCI-TXC)
#  SiS7016    1039/7016 untested
#
set DEVLIST = '"pci1039,900" "pci100b,20" "pci1039,7016"'
#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" sfe
sync
