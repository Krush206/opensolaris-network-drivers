***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the Sundance Technology ST201 Fast Ethernet NIC driver for Solaris.
 It is distributed under the BSD license.

2. Specification of the driver

File name of the driver: /kernel/drv/alta

Special file name:
	/dev/altaN (where N is a unit number, typcally 0 for first card)
        For example
                % ifconfig alta0

Tested OS version
 Solaris8 x86 00/10, Solaris9 x86 12/02

Tested chip and card
  DL10050A -- D-Link DFE-550TX

DL10050 and DL10050B are not tested, but are likely to work.


3. Preparing for Installation

(1) Install your PCI card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd alta-x.x.x.tar.gz | tar xf -

(3) Add hostname for the nic card into /etc/hosts file

(4) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../alta-x.x.x
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

        # cd /.../alta-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# /usr/ccs/bin/make uninstall ( for solaris7, don't do this )
	# init 6	(reboot)
        # modload obj/alta
        # devfsadm -i alta    ( for solaris7, use drvconfig and reboot with -r)
        # ifconfig altaN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for alta0 )
        # ifconfig altaN HOSTNAME
        # ifconfig altaN      ( ensure IP address is correct)
        # ifconfig altaN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the alta driver is fully functional, install it.

(1) install the alta driver into the kernel directory
        # cd /.../alta-x.x.x
        # /usr/ccs/bin/make install

       If you do not test the alta driver yet, execute the following commands:
            # ./adddrv.sh
	    # init 6	(reboot)
            # devfsadm -i alta ( for Solaris7, do drvconfig and reboot with -r )

(2) Configure the network interface. Create and/or modify the following files:
        /etc/hostname.altaN
    If your box is a dhcp client, you also need an empty file named
    /etc/dhcp.altaN.

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in /var/adm/messages file:
    WARNING: altaN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your nic card by specifying
   tne correct media mode in /kernel/drv/alta.conf with the following syntax:

   altaN-duplex=["full"|"half"] altaN-speed=[100|10]; # where N is a unit number

  For example:
	alta0-duplex="full" alta0-speed=100;   # full-duplex 100Mbps for alta0
	alta0-duplex="half" alta0-speed=10;    # half-duplex 10Mbps for alta0

Q. The driver cannot be unloaded because the device is busy, and the following
  message appears in /var/adm/messages file.
    NOTICE: altaN: alta_detach: buffer is busy

A. Wait for a little while until some modules in the kernel release receive
  buffers which was allocated by the alta driver.

Q. The system does not boot after the driver is installed.
A. Uninstall the nic card and reboot solaris. Then, unregister the driver.
     # rem_drv alta

   Or boot solaris with -a option and use /etc/system.noalta instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.noalta is automatically generated when executing 'make install'

Q. Solaris boots, but the network interface does not work.
A. Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for altaN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d altaN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv

  (3) Output of prtconf -vD

  (4) Output of adb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'adb -k unix.N vmcore.N' and type following subcommands.
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)
