***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the SiS190/191 integrated fast/gigabit ethernt NIC driver for Solaris.
It is distributed under the BSD license.

2. Specification of the driver
File name of the driver: /kernel/drv/sige
Special file name: /dev/sigeN (Where N is a unit number, typcally 0 for first
card)
        For example
                % ifconfig sige0
Currently sige driver isn't tested for SiS191.

Tested OS version
 Solaris11 x86 nv_17 (Athlon 2800+ w/ SiS760GX+SiS965L chipset)

Tested product
  SiS965L --- ASUS K8X-MX mainboard

3. Preparing for installation

(1) Copy source and binary files.
        # gunzip -cd sige-x.x.x.tar.gz | tar xf -

(2) Add hostname for the NIC card into /etc/hosts file

(3) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../sige-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(4) Making binaries
To make the binaries, you need Sun C compiler or gcc version 3, and do
the following operations.

        % /usr/ccs/bin/make

(5) Making binaries only for OpenSolaris users.
The driver likely work with GLD v3 in OpenSolaris, known as Nemo.
You can enjoy the new functions by recompiling the source code.

        % rm Makefile.config
        % ln -s Makefile.config_gld3 Makefile.config
        % /usr/ccs/bin/make


4. Testing
  Testing before installation is strongly recommended.

        # cd /.../sige-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
        # /usr/ccs/bin/make uninstall  (for solaris7, don't remove the file )
        # modload obj/sige
        # devfsadm -i sige  (for solaris7, use drvconfig and reboot with -r )
        # ifconfig sigeN plumb ( where N is an instance number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for sigeN)
        # ifconfig sigeN YOUR-HOST-NAME
        # ifconfig sigeN      ( ensure IP address is correct)
        # ifconfig sigeN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the sige driver is fully functional, install it.

(1) install the sige driver into the kernel directory
        # cd /.../sige-x.x.x
        # /usr/ccs/bin/make install

    If you do not test the sige driver yet, execute the following commands:
            # ./adddrv.sh
            # devfsadm -i sige (for solaris7, use drvconfig and reboot with -r)

(2) Configure the network interface. Create and/or modify the following file:
        /etc/hostname.sigeN

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    WARNING: sigeN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC card by specifying
  the correct media mode in /kernel/drv/sige.conf according to the following
  syntax:

   sigeN-duplex=["full"|"half"] sigeN-speed=[1000|100|10]; # where N is a unit number

  For example:
        sige0-duplex="full" sige0-speed=100;   # full-duplex 100Mbps for sige0
        sige0-duplex="half" sige0-speed=10;    # half-duplex 10Mbps for sige0

Q. The system does not boot after the driver is installed.
A.
   Disable on-board ethernet controller by bios setting

   Or boot solaris with -a option and use /etc/system.nosige instead of
   default [etc/system] to inhibit loading the driver.

   /etc/system.nosige is automatically generated when executing 'make install'

Q. Solaris boots, but the network interface does not work.
A.
   Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for sigeN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d sigeN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv

  (3) Output of prtconf -v

  (4) Output of adb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'adb -k unix.N vmcore.N' and type the following subcommands:
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)


