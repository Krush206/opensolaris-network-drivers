***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the 3com fast ethernet pccard driver for Solaris, and is an open
source alternative for Solaris elxl driver.  It is distributed under
the BSD license. You also need to install the cardbus driver v0.3 or later,
which is distributed from opensolaris web site below.

http://www.opensolaris.org/os/community/laptop/wireless/cardbus/

This driver also requires Solaris 10 or later.

2. Specification of the driver
File name of the driver: /kernel/drv/tcfe
Special file name: /dev/tcfeN (Where N is a unit number, typically 0 for
first card)
        For example
                % ifconfig tcfe0

Currently tcfe driver isn't tested on sparc platforms.

Tested OS version
 Solaris 11 b124 x86

Known prpblems
  3C575BT -- 3Com	(full duplex mode didn't work.)

3. Preparing for installation

(1) Install or update cardbus driver, then reboot system.

(2) Copy source and binary files of tcfe driver.
        # gunzip -cd tcfe-x.x.x.tar.gz | tar xf -

(3) install your pccard into a pcmcia slot. You can check newly created
    device node for the pccard by using "prtconf -v".

(4) Add hostname for the NIC card into /etc/hosts file

(5) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../tcfe-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(5) Making binaries (only for sparc platform)
Ae driver is ready for 64bit and 32bit solaris8 10/00 sparc or
later. (But currently it isn't tested on sparc platforms.)
This distribution does not include sparc binaries.  To make the binaries,
you need Sun C compiler or gcc version 3, and do the following operations.

        % /usr/ccs/bin/make

(6) Making binaries only for OpenSolaris users.
The driver likely work with GLD v3 in OpenSolaris, known as Nemo.
You can enjoy the new functions by recompiling the source code.

        % rm Makefile.config
        % ln -s Makefile.config_gld3 Makefile.config
        % /usr/ccs/bin/make


4. Testing
  Testing before installation is strongly recommended.

        # cd /.../tcfe-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
        # /usr/ccs/bin/make uninstall
	# init 6	(reboot)
        # modload obj/tcfe
        # devfsadm -i tcfe
        # ifconfig tcfeN plumb ( where N is an instance number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for tcfeN)
        # ifconfig tcfeN YOUR-HOST-NAME
        # ifconfig tcfeN      ( ensure IP address is correct)
        # ifconfig tcfeN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the tcfe driver is fully functional, install it.

(1) install the tcfe driver into the kernel directory
        # cd /.../tcfe-x.x.x
        # /usr/ccs/bin/make install

    If you do not test the tcfe driver yet, execute the following commands:
            # ./adddrv.sh
	    # init 6	(reboot)
            # devfsadm -i tcfe (for solaris7, use drvconfig and reboot with -r)

(2) Configure the network interface. Create and/or modify the following file:
        /etc/hostname.tcfeN

    If you want to use tcfe with dhcp, you also need to create a empty file
    below to get an assigned IP address automatically from a dhcp server
    at boot time.
        /etc/dhcp.tcfeN

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    WARNING: tcfeN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC card by specifying
  the correct media mode in /kernel/drv/tcfe.conf according to the following
  syntax:

   tcfeN-duplex=["full"|"half"] tcfeN-speed=[100|10]; # where N is a unit number

  For example:
        tcfe0-duplex="full" tcfe0-speed=100;   # full-duplex 100Mbps for tcfe0
        tcfe0-duplex="half" tcfe0-speed=10;    # half-duplex 10Mbps for tcfe0

Q. The system does not boot after the driver is installed.
A. Uninstall the NIC card and reboot solaris. Then, unregister the driver.
     # rem_drv tcfe

   Or boot solaris with -a option and use /etc/system.notcfe instead of
   default [etc/system] to inhibit loading the driver.

   /etc/system.notcfe is automatically generated when executing 'make install'

Q. Solaris boots, but the network interface does not work.
A.
   Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for tcfeN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d tcfeN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv

  (3) Output of prtconf -v

  (4) Output of adb (only for solaris9 or previous)
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'adb -k unix.N vmcore.N' and type the following subcommands:
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)


