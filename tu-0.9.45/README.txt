***************************************************************************
CAUTION: This software is NO WARRANTY.
As this software is loaded into kernel, it might cause panic or hung.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is a nic driver for tulip like nics under Solaris.  It is distributed
under the BSD license.

2. Specification of the driver

File name of the driver: /kernel/drv/tu

Special file name:
	/dev/tuN (where N is an unit number, typcally 0 for first card)
        For example
                % ifconfig tu0

Tested OS version
 Solaris8 x86 00/10, Solaris9 x86 12/02
 Solaris9 sparc

Tested chips (and cards).
	ADMtek AL981 (Planex communications, FNW-9800-T)
	ADMtek AN985 (Netgear FA511)
	ADMtek AN983B (Corega Ether PCI-TM, Planex Communications FNW-9803-T)
	DAVICOM DM9102A (HITPOINT HT-9102)
	Macronix MX98713
	Macronix MX98715AEC (Melco BUFFALO LCI2-TXI)
	Conexant RS7112 (Mini PCI built-in Compaq Presario 17XL369)
		(This was tested under Solaris9 by Antonio Giordano.)
	LITE-ON LC82C115
	LITE-ON LC82C168
	LITE-ON LC82C169 (Netgear FA310)
		(This was tested under Solaris9 by Klaus Ziegler.)
	DEC 21140 with NS DP83840 MII PHY (IntraServer)
		(This was tested under SPARC Solaris9 by Klaus Ziegler.)
	DEC 21140AF with DAVICOM DM9101F MII PHY
	DEC 21143PC with AB 10100 PHY (Melco BUFFALO LCI-TXJ)
	DEC 21143PC with KENDIN KS8761 PHY
	XIRCOM CBE-100 (IBM 10/100 EtherJet)

3. Prepare for installation

(1) Install your PCI card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd tu-x.x.x.tar.gz | tar xf -

(3) Add hostname for the NIC card into /etc/hosts file

(4) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../tu-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(5) Making binaries (only for sparc platforms)
Gani driver is ready for 64bit and 32bit solaris8 10/00 sparc or
later. (But currently it is unstable on sparc platforms.)
This distribution does not include sparc binaries.  To make the binaries,
you need Sun C compiler or gcc version 3, and do the following operations.

        % /usr/ccs/bin/make

(6) Remove dnet driver (only for x86 platforms)
	# rem_drv dnet
	# init 6

4. Testing
        # cd /.../tu-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# /usr/ccs/bin/mamke uninstall ( for solaris7, do not remove the driver)
        # modload obj/tu
        # devfsadm -i tu (for solaris7, use drvconfig and reboot with -r )
        # ifconfig tuN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for tu0, ensure ethernet address is correct)
        # ifconfig tuN HOSTNAME
        # ifconfig tuN      ( ensure IP address is correct)
        # ifconfig tuN up   ( and then you can test ping, telnet, ftp ...)

5. Installation
    After you ensure that tu driver is fully functional, install it.

(1) install tu driver into kernel directory
        # cd /.../tu-x.x.x
        # /usr/ccs/bin/make install

       If you do not test tu driver yet, here execute following commands.
            # ./adddrv.sh
            # devfsadm -i tu (for solaris7, use drvconfig then reboot with -r)

(2) Configure the network interface. Create and/or modify following files.
        /etc/hostname.tuN

    If you want to use tu with dhcp, you also need to create a empty file
    below to get an assigned IP address automatically from a dhcp server
    at boot time.
        /etc/dhcp.tuN

(3) Reboot the system.
        # init 6


6. Trouble shooting

Q. Following message is printed in /var/adm/messages file
    WARNING: tuN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC card by specifying
  correct media mode in /kernel/drv/tu.conf according to following syntax.

   tuN-duplex=["full"|"half"] tuN-speed=[100|10]; # where N is an unit number

  For example:
	tu0-duplex="full" tu0-speed=100;   # full-duplex 100Mbps for tu0
	tu0-duplex="half" tu0-speed=10;    # half-duplex 10Mbps for tu0

Q. The driver cannot be unloaded because of device busy, and following
  message appears in /var/adm/messages file.
    NOTICE: tuN: tu_detach: buffer is busy

A. Wait for a little time until some modules in kernel release receive
  buffers which was allocated by tu driver.

Q. The system does not boot after the driver was installed.
A. Uninstall the NIC card and reboot solaris. Then, unregister the driver.
     # rem_drv tu

   Or boot solaris with -a option and use /etc/system.notu instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.notu is automatically generated while executing 'make install'
  or 'make test'

Q. Solaris boots, but the network interface does not work.
A.
   Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for tuN, the driver is loaded
   and working correctly.
   Please check configuration for the network interface.


   Is ethernet cable connected correctly?

   Please check if the network interface receive packets.
	% snoop -P -d tuN


   Change BIOS setting to disable ACPI or PnP.
   Operation is depend on your BIOS. Refer your BIOS manual.

Q. How to install dnet driver again?
A. After removing tu driver and reboot the system, do following.
        # cd /.../tu-x.x.x
	# ./adddrv_dnet.sh

If Solaris panics while testing, please send me following information.
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

