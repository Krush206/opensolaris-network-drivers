***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is Attansic L1/L2 and Atheross AR81 family driver for Solaris.
It is distributed under the BSD license.

2. Specification of the driver

File name of the driver:
	/kernel/drv/atge
	/kernel/drv/amd64/atge
	/kernel/drv/sparcv9/atge (for sparc 64bit kernel)

Special file name:
	/dev/atgeN (where N is a unit number, typcally 0 for first card)
        For example
                % ifconfig atge0

Tested chips and cards
  Attansic L1 (ASUS P5KPL-VM)
  Attansic L1E (ASUS P5KPL-CM)
  Atheros L2
  Atheros L2C
  Atheros L1C (Gigabyte GA-G31M ES2L)

3. Prepare for installation 

(1) Copy source and binary files.
        # gunzip -cd atge-x.x.x.tar.gz | tar xf -

(2) Add hostname for the NIC card into the /etc/hosts file

(3) Make links to correct binary directory and Makefile according to your
configuration.  amd64 and gcc are default.

        % cd /.../atge-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(4) Making binaries with GLD v3 only for OpenSolaris users.
Atge driver likely work with GLD v3 in OpenSolaris, known as Nemo.
You can enjoy the new functions by recompiling atge source code.
To do this, you need to download the source code of OpenSolaris and
modify ONUTSDIR macro in Makefile.config_gld3 to refer the uts directory
After that, compile atge according with followings.

	% rm Makefile.config
	% ln -s Makefile.config_gld3 Makefile.config
        % /usr/ccs/bin/make


4. Testing

        # cd /.../atge-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# /usr/ccs/bin/make uninstall (don`t do this for Solaris7)
        # modload obj/atge
        # devfsadm -i atge ( for solaris7, use drvconfig and reboot with -r option )
        # ifconfig atgeN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for atge0, ensure ethernet address is correct)
        # ifconfig atgeN HOSTNAME
        # ifconfig atgeN      ( ensure IP address is correct)
        # ifconfig atgeN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the atge driver is fully functional, install it.

(1) install the atge driver into kernel directory
        # cd /.../atge-x.x.x
        # /usr/ccs/bin/make install

       If you did not test atge driver yet, execute the following commands:
            # ./adddrv.sh
            # devfsadm -i atge ( for Solaris7 use drvconfig and reboot with -r option )

(2) Configure the network interface. Create and/or modify the following files:
        /etc/hostname.atgeN

    If you want to use atge with dhcp, you also need to create a empty file
    below to get an assigned IP address automatically from a dhcp server
    at boot time.
	/etc/dhcp.atge0

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    NOTE: atgeN: link up but auto-negotiation failed.

A. This massage appears when the link partner does not have auto negotiation
   capability.
   Please disable auto negotiation capability for your NIC card by specifying
   the correct media mode in /kernel/drv/atge.conf with the following syntax:

   full-duplex=[1|0] speed=[1000|100|10];

  For example
	full-duplex=1 speed=100;   # full-duplex 100Mbps
	full-duplex=0 speed=10;    # half-duplex 10Mbps

Q. The driver cannot be unloaded because the device is busy, and the following
   message appears in /var/adm/messages file:
    NOTICE: atgeN: atge_detach: buffer is busy

A. Wait for a little while until some modules in the kernel release receive
   buffers which were allocated by the atge driver.

Q. The system does not boot after the driver is installed.
A. Uninstall the NIC card and reboot solaris. Then, unregister the driver:
     # rem_drv atge

   Or boot solaris with -a option and use /etc/system.noatge instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.noatge is automatically generated when executing 'make install'

Q. Solaris boots, but the network interface does not work.
A. Is the network interface running?

   Execute 'ifconfig -a'.  If you see an entry for atgeN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d atgeN

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
