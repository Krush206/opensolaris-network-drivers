***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the AMD PCnet series ethernet NIC driver for Solaris, and is an
open source alternative for Solaris pcn driver supported under only 32bit
environment.  It is distributed under the BSD license,

2. Specification of the driver
File name of the driver: /kernel/drv/ae
Special file name: /dev/aeN (Where N is a unit number, typically 0 for first
card)
        For example
                % ifconfig ae0

Currently ae driver isn't tested on sparc platforms.

Tested OS version
 Solaris 11 x86 nv_17 (Athlon 2800+ w/ SiS760GX chipset)

Tested cards
  AM79C970A -- unknown vendor
  AM79C971 -- unknown vendor
  AM79C972 -- unknown vendor

3. Preparing for installation

(1) Unregister solaris pcn driver
	# rem_drv pcn
	# init 0	(then turn off the system)

(2) Install your PCI card and boot Solaris.

(3) Copy source and binary files.
        # gunzip -cd ae-x.x.x.tar.gz | tar xf -

(4) Add hostname for the NIC card into /etc/hosts file

(5) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../ae-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(6) Making binaries (only for sparc platform)
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

        # cd /.../ae-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
        # /usr/ccs/bin/make uninstall  (for solaris7, don't remove the file )
	# init 6	(reboot the system)
        # modload obj/ae
        # devfsadm -i ae  (for solaris7, use drvconfig and reboot with -r )
        # ifconfig aeN plumb ( where N is an instance number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for aeN)
        # ifconfig aeN YOUR-HOST-NAME
        # ifconfig aeN      ( ensure IP address is correct)
        # ifconfig aeN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the ae driver is fully functional, install it.

(1) install the ae driver into the kernel directory
        # cd /.../ae-x.x.x
        # /usr/ccs/bin/make install

    If you do not test the ae driver yet, execute the following commands:
            # ./adddrv.sh
	    # init 6		(reboot the system)
            # devfsadm -i ae (for solaris7, use drvconfig and reboot with -r)

(2) Configure the network interface. Create and/or modify the following file:
        /etc/hostname.aeN

    If you want to use ae with dhcp, you also need to create a empty file
    below to get an assigned IP address automatically from a dhcp server
    at boot time.
        /etc/dhcp.aeN

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    WARNING: aeN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC card by specifying
  the correct media mode in /kernel/drv/ae.conf according to the following
  syntax:

   aeN-duplex=["full"|"half"] aeN-speed=[100|10]; # where N is a unit number

  For example:
        ae0-duplex="full" ae0-speed=100;   # full-duplex 100Mbps for ae0
        ae0-duplex="half" ae0-speed=10;    # half-duplex 10Mbps for ae0

Q. The system does not boot after the driver is installed.
A. Uninstall the NIC card and reboot solaris. Then, unregister the driver.
     # rem_drv ae

   Or boot solaris with -a option and use /etc/system.noae instead of
   default [etc/system] to inhibit loading the driver.

   /etc/system.noae is automatically generated when executing 'make install'

Q. Solaris boots, but the network interface does not work.
A.
   Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for aeN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d aeN

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


