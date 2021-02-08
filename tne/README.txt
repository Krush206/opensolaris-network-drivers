***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the Texas Instruments ThuderLAN Fast Ethernet NIC driver for Solaris.
 It is distributed under the BSD license.

2. Specification of the driver

File name of the driver: /kernel/drv/tne

Special file name:
	/dev/tneN (where N is a unit number, typcally 0 for first card)
        For example
                % ifconfig tne0

Tested OS version
 Solaris envada x86 b90

Tested chip and card
  Texas Instruments ThunderLAN TNETE100A -- Compaq Netelligent 10/100 TX UTP

3. Preparing for installation

(1) Install your PCI card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd tne-x.x.x.tar.gz | tar xf -

(3) Add hostname for the nic card into /etc/hosts file

(4) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../tne-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  Here ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(5) Making binaries (only for sparc platform)
Tne driver is ready for 64bit and 32bit solaris8 10/00 sparc or
later.
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

        # cd /.../tne-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# /usr/ccs/bin/make uninstall( for solaris7, don't execute this)
	# init 6	(reboot)
        # modload obj/tne
        # devfsadm -i tne    ( for solaris7, use drvconfig and reboot with -r)
        # ifconfig tneN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for tne0 )
        # ifconfig tneN HOSTNAME
        # ifconfig tneN      ( ensure IP address is correct)
        # ifconfig tneN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the tne driver is fully functional, install it.

(1) install the tne driver into the kernel directory
        # cd /.../tne-x.x.x
        # /usr/ccs/bin/make install

       If you do not test the tne driver yet, execute the following commands:
            # ./adddrv.sh
	    # init 6	(reboot)
            # devfsadm -i tne ( for Solaris7, do drvconfig and reboot with -r )

(2) Configure the network interface. Create and/or modify the following files:
        /etc/hostname.tneN

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in /var/adm/messages file:
    WARNING: tneN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your nic card by specifying
   tne correct media mode in /kernel/drv/tne.conf with the following syntax:

   full-duplex=[0|1] speed=[100|10]; # where N is a unit number

  For example:
	full-duplex=1 speed=100;   # full-duplex 100Mbps
	full-duplex=0 speed=10;    # half-duplex 10Mbps

Q. The driver cannot be unloaded because the device is busy, and the following
  message appears in /var/adm/messages file.
    NOTICE: tneN: tne_detach: buffer is busy

A. Wait for a little while until some modules in the kernel release receive
  buffers which was allocated by the tne driver.

Q. The system does not boot after the driver is installed.
A. Uninstall the nic card and reboot solaris. Then, unregister the driver.
     # rem_drv tne

   Or boot solaris with -a option and use /etc/system.notne instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.notne is automatically generated when executing 'make install'

Q. Solaris boots, but the network interface does not work.
A. Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for tneN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d tneN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv

  (3) Output of prtconf -vD

  (4) Output of adb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'mdb -k unix.N vmcore.N' and type following subcommands.
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)
