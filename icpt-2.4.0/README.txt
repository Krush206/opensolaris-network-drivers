***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the IC Plus IP1000A Gigabit Ethernet NIC driver for Solaris.
 It is distributed under the BSD license.

2. Specification of the driver

File name of the driver: /kernel/drv/icpt

Special file name:
	/dev/icptN (where N is a unit number, typcally 0 for first card)
        For example
                % ifconfig icpt0

Tested OS version
 Solaris Express x86 NV14

Tested chip and card
  IC Plus IP1000A (ASUS tech NX1101)

3. Preparing for Installation

(1) Install your PCI card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd icpt-x.x.x.tar.gz | tar xf -

(3) Add hostname for the nic card into /etc/hosts file

(4) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../icpt-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(5) Making binaries (only for sparc platform)
Alta driver is ready for 64bit and 32bit solaris8 10/00 sparc or later.
This distribution does not include sparc binaries.  To make the binaries,
you need Sun C compiler or gcc version 3, and do following command.

        % /usr/ccs/bin/make

(6) Making binaries only for OpenSolaris users.
The driver likely work with GLD v3 in OpenSolaris, known as Nemo.
You can enjoy the new functions by recompiling the source code.

        % rm Makefile.config
        % ln -s Makefile.config_gld3 Makefile.config
        % /usr/ccs/bin/make


4. Testing
  Testing before installation is strongly recommended.

        # cd /.../icpt-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# /usr/ccs/bin/make uninstall ( for solaris7, don't do this )
        # modload obj/icpt
        # devfsadm -i icpt    ( for solaris7, use drvconfig and reboot with -r)
        # ifconfig icptN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for icpt0 )
        # ifconfig icptN HOSTNAME
        # ifconfig icptN      ( ensure IP address is correct)
        # ifconfig icptN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the icpt driver is fully functional, install it.

(1) install the icpt driver into the kernel directory
        # cd /.../icpt-x.x.x
        # /usr/ccs/bin/make install

       If you do not test the icpt driver yet, execute the following commands:
            # ./adddrv.sh
            # devfsadm -i icpt ( for Solaris7, do drvconfig and reboot with -r )

(2) Configure the network interface. Create and/or modify the following files:
        /etc/hostname.icptN
    If your box is a dhcp client, you also need to create an empty file named
    /etc/dhcp.icptN.

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in /var/adm/messages file:
    WARNING: icptN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your nic card by specifying
   tne correct media mode in /kernel/drv/icpt.conf with the following syntax:

   icptN-duplex=["full"|"half"] icptN-speed=[1000|100|10]; # where N is a unit number

  For example:
	icpt0-duplex="full" icpt0-speed=100;   # full-duplex 100Mbps for icpt0
	icpt0-duplex="half" icpt0-speed=10;    # half-duplex 10Mbps for icpt0

Q. The driver cannot be unloaded because the device is busy, and the following
  message appears in /var/adm/messages file.
    NOTICE: icptN: icpt_detach: buffer is busy

A. Wait for a little while until some modules in the kernel release receive
  buffers which was allocated by the icpt driver.

Q. The system does not boot after the driver is installed.
A. Uninstall the nic card and reboot solaris. Then, unregister the driver.
     # rem_drv icpt

   Or boot solaris with -a option and use /etc/system.noicpt instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.noicpt is automatically generated when executing 'make install'

Q. Solaris boots, but the network interface does not work.
A. Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for icptN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d icptN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv

  (3) Output of prtconf -vD

  (4) Output of mdb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'mdb -k unix.N vmcore.N' and type the following subcommands:
   $c   (stack trace back will be printed)
   ::msgbuf   (last messages are printed)
   ^D         (To quit mdb, type control-D)
