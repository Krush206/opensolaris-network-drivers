***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the JMicron JMC250/260 nic driver for Solaris. It is distributed
under the BSD license.

2. Specification of the driver

File name of the driver:
	/kernel/drv/jmge
	/kernel/drv/sparcv9/jmge (for sparc 64bit kernel)

Special file name:
	/dev/jmgeN (where N is a unit number, typcally 0 for first card)
        For example
                % ifconfig jmge0

Tested chips and cards
  JMC250 - evaluation board from JMicron
  JMC261 - evaluation board from JMicron

3. Prepare for installation 

(1) Install your PCI card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd jmge-x.x.x.tar.gz | tar xf -

(3) Add hostname for the NIC card into the /etc/hosts file

(4) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../jmge-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(6) Making binaries (only for sparc platform)
Vfe driver is ready for 64bit and 32bit solaris8 10/00 sparc or
later. (But currently it is unstable on sparc platforms.)
This distribution does not include sparc binaries.  To make the binaries,
you need Sun C compiler or gcc version 3, and do the following operations.

        % /usr/ccs/bin/make

(7) Making binaries only for OpenSolaris users.
Vfe driver likely work with GLD v3 in OpenSolaris, known as Nemo.
You can enjoy the new functions by recompiling jmge source code.

	% rm Makefile.config
	% ln -s Makefile.config_gld3 Makefile.config
        % /usr/ccs/bin/make


4. Testing

        # cd /.../jmge-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# /usr/ccs/bin/make uninstall (don`t do this for Solaris7)
        # modload obj/jmge
        # devfsadm -i jmge ( for solaris7, use drvconfig and reboot with -r option )
        # ifconfig jmgeN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for jmge0, ensure ethernet address is correct)
        # ifconfig jmgeN HOSTNAME
        # ifconfig jmgeN      ( ensure IP address is correct)
        # ifconfig jmgeN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the jmge driver is fully functional, install it.

(1) install the jmge driver into kernel directory
        # cd /.../jmge-x.x.x
        # /usr/ccs/bin/make install

       If you did not test jmge driver yet, execute the following commands:
            # ./adddrv.sh
            # devfsadm -i jmge ( for Solaris7 use drvconfig and reboot with -r option )

(2) Configure the network interface. Create and/or modify the following files:
        /etc/hostname.jmgeN

    If you want to use jmge with dhcp, you also need to create a empty file
    below to get an assigned IP address automatically from a dhcp server
    at boot time.
	/etc/dhcp.jmge0

(3) Reboot the system.
        # init 6


6. Troubleshooting

Q. The system does not boot after the driver is installed.
A. Uninstall the NIC card and reboot solaris. Then, unregister the driver:
     # rem_drv jmge

   Or boot solaris with -a option and use /etc/system.nojmge instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.nojmge is automatically generated when executing 'make install'
   or 'make test'

Q. Solaris boots, but the network interface does not work.
A. Is the network interface running?

   Execute 'ifconfig -a'.  If you see an entry for jmgeN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d jmgeN

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
