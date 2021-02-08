***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the VIA Rhine family NIC driver for Solaris.  Rh will work with
VT86C100 (Rhine I), DL10030 (Rhine I), VT6102 (Rhine II), VT6105 (Rhine
III), built-in Rhine II ethernet mac in VT8235 and VT8237 south bridges.
It is distributed under the BSD license.

2. Specification of the driver

File name of the driver:
	/kernel/drv/vfe
	/kernel/drv/sparcv9/vfe (for sparc 64bit kernel)

Special file name:
	/dev/vfeN (where N is a unit number, typcally 0 for first card)
        For example
                % ifconfig vfe0

Tested chips and cards
  VT6102(Rhine II rev 43) -- Planex communications Inc.(Japan) FNW-9702-T3
  VT6102(Rhine II rev 47)
  DL10030(Rhine I rev 6) --  Corega Inc.(Japan) FEtherII PCI TX
  VT6105(Rhine III rev 85) -- Corega Inc.(Japan) FEther PCI-TXA

This driver is ready for VT6105M (Rhine-III management adaptor) but not
tested.

3. Prepare for installation 

(1) Install your PCI card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd vfe-x.x.x.tar.gz | tar xf -

(3) Add hostname for the NIC card into the /etc/hosts file

(4) If you have installed rh driver, remove it and reboot the system.
	# rem_drv rh
	# sync
	# init 0

(5) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../vfe-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(6) Making binaries (only for sparc platform)
Gani driver is ready for 64bit and 32bit solaris8 10/00 sparc or
later. (But currently it is unstable on sparc platforms.)
This distribution does not include sparc binaries.  To make the binaries,
you need Sun C compiler or gcc version 3, and do the following operations.

        % /usr/ccs/bin/make


4. Testing

        # cd /.../vfe-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# /usr/ccs/bin/make uninstall (don`t do this for Solaris7)
        # modload obj/vfe
        # devfsadm -i vfe ( for solaris7, use drvconfig and reboot with -r option )
        # ifconfig vfeN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for vfe0, ensure ethernet address is correct)
        # ifconfig vfeN HOSTNAME
        # ifconfig vfeN      ( ensure IP address is correct)
        # ifconfig vfeN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the vfe driver is fully functional, install it.

(1) install the vfe driver into kernel directory
        # cd /.../vfe-x.x.x
        # /usr/ccs/bin/make install

       If you did not test vfe driver yet, execute the following commands:
            # ./adddrv.sh
            # devfsadm -i vfe ( for Solaris7 use drvconfig and reboot with -r option )

(2) Configure the network interface. Create and/or modify the following files:
        /etc/hostname.vfeN

    If you want to use vfe with dhcp, you also need to create a empty file
    below to get an assigned IP address automatically from a dhcp server
    at boot time.
        /etc/dhcp.vfeN

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    NOTE: vfeN: link up but auto-negotiation failed.

A. This massage appears for Rhine I chips when the link partner does not
   have auto negotiation capability.
   Please disable auto negotiation capability for your NIC card by specifying
   the correct media mode in /kernel/drv/vfe.conf with the following syntax:

   vfeN-duplex=["full"|"half"] vfeN-speed=[100|10]; # where N is a unit number

  For example
	vfe0-duplex="full" vfe0-speed=100;   # full-duplex 100Mbps for vfe0
	vfe0-duplex="half" vfe0-speed=10;    # half-duplex 10Mbps for vfe0

Q. The driver cannot be unloaded because the device is busy, and the following
   message appears in /var/adm/messages file:
    NOTICE: vfeN: vfe_detach: buffer is busy

A. Wait for a little while until some modules in the kernel release receive
   buffers which were allocated by the vfe driver.

Q. The system does not boot after the driver is installed.
A. Uninstall the NIC card and reboot solaris. Then, unregister the driver:
     # rem_drv vfe

   Or boot solaris with -a option and use /etc/system.novfe instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.novfe is automatically generated when executing 'make install'
   or 'make test'

Q. Solaris boots, but the network interface does not work.
A. Is the network interface running?

   Execute 'ifconfig -a'.  If you see an entry for vfeN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d vfeN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv

  (3) Output of prtconf -vD

  (4) Output of adb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'adb -k unix.N vmcore.N' and type the following subcommands.
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)
