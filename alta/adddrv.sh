#! /bin/csh -f
#
# Sundance ST201 fast ethernet mac driver
#
# Tested PCI cards
#  D-Link DFE-550TX
#
set DEVLIST = ""
set DEVLIST = ($DEVLIST '"pci1186,1002"')	# D-Link DFE-550TX
set DEVLIST = ($DEVLIST '"pci13f0,201"')	# Generic ST201 (IP100)
set DEVLIST = ($DEVLIST '"pci13f0,200"')	# Generic IP100A

#echo $DEVLIST

/usr/sbin/add_drv -n -v -m '* 0600 root sys' -i "$DEVLIST" alta
sync
