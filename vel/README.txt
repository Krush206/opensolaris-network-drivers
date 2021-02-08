***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the VIA VT6122 giga bit ethernt NIC driver for Solaris.
It is distributed under the BSD license.

2. Specification of the driver

File name of the driver: /kernel/drv/vel

Special file name: /dev/velN (Where N is a unit number, typcally 0 for first
card)

        For example
                % ifconfig vel0

Tested OS version
 Solaris8 x86 10/00, Solaris10 x86 b72

Tested card
  VT6122 -- kurotoshiko GbE

3. Preparing for installation

(1) Install your PCI card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd vel-x.x.x.tar.gz | tar xf -

(3) Add hostname for the NIC card into /etc/hosts file

(4) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../vel-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER{ Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you use to make the driver.

(5) Making binaries (only for sparc platform)
The vel driver will work under 64bit and 32bit sparc solaris8 10/00 or
later, but this distribution does not include sparc binaries.
To make binaries, you need Sun C compiler or gcc, and do the following
operations before testing and installation:

        % /usr/ccs/bin/make

(6) Making binaries only for OpenSolaris users.
The driver likely work with GLD v3 in OpenSolaris, known as Nemo.
You can enjoy the new functions by recompiling the source code.

        % rm Makefile.config
        % ln -s Makefile.config_gld3 Makefile.config
        % /usr/ccs/bin/make


4. Testing
   Testing before installation is strongly recommended.
        # cd /.../vel-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
        # /usr/ccs/bin/make uninstall  (for solaris7, don't remove the file )
        # modload obj/vel
        # devfsadm -i vel  (for solaris7, use drvconfig and reboot with -r )
        # ifconfig velN plumb ( where N is an instance number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for velN)
        # ifconfig velN YOUR-HOST-NAME
        # ifconfig velN      ( ensure IP address is correct)
        # ifconfig velN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the vel driver is fully functional, install it.

(1) install the vel driver into the kernel directory
        # cd /.../vel-x.x.x
        # /usr/ccs/bin/make install

    If you do not test the vel driver yet, execute the following commands:
            # ./adddrv.sh
            # devfsadm -i vel (for solaris7, use drvconfig and reboot with r )

(2) Configure the network interface. Create and/or modify the following file:
        /etc/hostname.velN

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    WARNING: velN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC card by specifying
  the correct media mode in /kernel/drv/vel.conf according to the following
  syntax:

   velN-duplex=["full"|"half"] velN-speed=[1000|100|10]; # where N is a unit number

  For example:
        vel0-duplex="full" vel0-speed=100;   # full-duplex 100Mbps for vel0
        vel0-duplex="half" vel0-speed=10;    # half-duplex 10Mbps for vel0

Q. The system does not boot after the driver is installed.
A. Uninstall the NIC card and reboot solaris. Then, unregister the driver.
     # rem_drv vel

   Or boot solaris with -a option and use /etc/system.novel instead of
   default [etc/system] to inhibit loading the driver.

   /etc/system.novel is automatically generated when executing 'make install'

Q. Solaris boots, but the network interface does not work.
A.
   Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for velN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d velN

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


