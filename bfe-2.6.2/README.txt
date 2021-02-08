***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the broadcom bcm4401 NIC driver for Solaris.  It is distributed
under the BSD license.

2. Specification of the driver

File name of the driver: /kernel/drv/bfe

Special file name:
	/dev/bfeN (where N is a unit number, typcally 0 for first card)
        For example
                % ifconfig bfe0

Tested OS version
 Solaris8 00/10 x86, Solaris9 12/02 x86
 bmc440x nics don't work with sparc.

Tested chips and cards
  bcm4401 on GREEN HOUSE GH-WL100BB

3. Preparing for installation

(1) Install your PCI card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd bfe-x.x.x.tar.gz | tar xf -

(3) Add hostname for the NIC card into the /etc/hosts file

(4) If you have installed bcf driver, remove it and reboot the system.
        # rem_drv bcf
	# sync
	# init 0 

(5) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../bfe-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the binary of the driver.

(6) Making binaries (only for sparc platform)
Bcf driver is ready for 64bit and 32bit solaris8 10/00 sparc or
later. (But currently it isn't tested on sparc platforms.)
This distribution does not include sparc binaries.  To make the binaries,
you need Sun C compiler or gcc version 3, and do the following operations.

        % /usr/ccs/bin/make

(7) Making binaries only for OpenSolaris users.
Bfe driver likely work with GLD v3, known as Nemo, under OpenSolaris.
You can enjoy the new functions by recompiling bfe source code with
OpenSolaris source code.

        % rm Makefile.config
        % ln -s Makefile.config_gld3 Makefile.config
	% vi Makefile.config  (fix ONUTSDIR macro to point uts directory in opensolaris kernel source)
        % /usr/ccs/bin/make


4. Testing
  Testing before installation is strongly recommended.

        # cd /.../bfe-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# /usr/ccs/bin/make uninstall (for Solaris7, do not execute this )
	# init 6	(reboot)
        # modload obj/bfe
        # devfsadm -i bfe ( for solaris7, use drvconfig and reboot with -r option )
        # ifconfig bfeN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for bfe0, ensure ethernet address is correct)
        # ifconfig bfeN HOSTNAME
        # ifconfig bfeN      ( ensure IP address is correct)
        # ifconfig bfeN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the bfe driver is fully functional, install it.

(1) install the bfe driver into kernel directory
        # cd /.../bfe-x.x.x
        # /usr/ccs/bin/make install

       If you did not test bfe driver yet, execute the following commands:
            # ./adddrv.sh
	    # init 6	(reboot)
            # devfsadm -i bfe ( for Solaris7 use drvconfig and reboot with -r option )

(2) Configure the network interface. Create and/or modify the following files:
        /etc/hostname.bfeN

    If you want to use bfe with dhcp, you also need to create a empty file
    below to get an assigned IP address automatically from a dhcp server
    at boot time.
        /etc/dhcp.bfeN
    And /etc/hostname.bfe0 should be an empty file.

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    NOTE: bfeN: link up but auto-negotiation failed.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC card by specifying
   the correct media mode in /kernel/drv/bfe.conf with the following syntax:

  full-duplex=[1|0] speed=[100|10];

  For example
	full-duplex=1 speed=100;   # full-duplex 100Mbps
	full-duplex=0 speed=10;    # half-duplex 10Mbps

Q. The driver cannot be unloaded because the device is busy, and the following
   message appears in /var/adm/messages file:
    NOTICE: bfeN: bfe_detach: buffer is busy

A. Wait for a little while until some modules in the kernel release receive
   buffers which were allocated by the bfe driver.

Q. The system does not boot after the driver is installed.
A. Uninstall the NIC card and reboot solaris. Then, unregister the driver:
     # rem_drv bfe

   Or boot solaris with -a option and use /etc/system.nobfe instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.nobfe is automatically generated when executing 'make install'
   or 'make test'

Q. Solaris boots, but the network interface does not work.
A. Is the network interface running?

   Execute 'ifconfig -a'.  If you see an entry for bfeN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d bfeN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv

  (3) Output of prtconf -vD

  (4) Output of adb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'mdb -k unix.N vmcore.N' and type the following subcommands.
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)

  If you use Solaris10 or later, use mdb instead of adb:
   $c   (stack trace back will be printed)
   ::msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)
