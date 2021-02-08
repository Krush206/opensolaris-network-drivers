***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the DP838915/SiS900 NIC driver of Solaris.
It is distributed under the BSD license.


2. Specification of the driver

File name of the driver: /kernel/drv/sfe

Special file name:
	/dev/sfeN (where N is a unit number, typcally 0 for first card)

        For example
                % ifconfig sfe0

Tested OS version
  Solaris8 x86 00/10, Solaris9 x86 00/10

Tested chips and cards
  DP83815CVNG -- NETGEAR F311
  SiS900  -- MELCO LGY-PCI-TXC
  SiS900 built in SiS630ET chip set (thank to Rahul)

3. Preparing for installation
(1) Install your PCI card and boot Solaris.

(2) If previous version of sfe driver is installed, uninstall it first
    and reboot the system.
	# rem_drv sfe	(ignore error messages)
	# init 6

(3) Copy source and binary files.
        # gunzip -cd sfe-x.x.x.tar.gz | tar xf -

(4) Add a hostname entry for the nic card into /etc/hosts file

(5) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../sfe-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(6) Making binaries (only for sparc platform)
Sfe driver is ready for 64bit and 32bit solaris8 10/00 sparc or later.
This distribution does not include sparc binaries.  To make the binaries,
you need Sun C compiler or gcc version 3, and do following command.

        % /usr/ccs/bin/make

(7) Making binaries only for OpenSolaris users.
The driver likely work with GLD v3 in OpenSolaris, known as Nemo.
You can enjoy the new functions by recompiling the source code.

        % rm Makefile.config
        % ln -s Makefile.config_gld3 Makefile.config
        % /usr/ccs/bin/make


4. Testing

	# cd sfe-x.x.x
	# /usr/ccs/bin/make install
	# ./adddrv.sh
	# /usr/ccs/bin/make uninstall (for solaris7, do not remove the file)
	# init 6	(reboot the system)
	# modload obj/sfe
	# devfsadm -i sfe (for solaris7, use drvconfig and then reboot with -r )
	# ifconfig sfeN plumb
	# ifconfig -a	    (you will see an entry for sfeN, ensure MAC address)
	# ifconfig sfeN YOUR-HOSTNAME
	# ifconfig sfeN	    (ensure IP address is correct)
	# ifconfig sfeN up  (and then you can test with ping, telnet, ftp ...)

  CAUTION for on-board chips
	If any error occurs, REMOVE THE DRIVER INSTANTLY BEFORE SHUTDOWN THE
        SYSTEM.
	To remove, execute the following command:
	# rem_drv sfe
	And ensure that /etc/driver_aliases file does NOT include any
	definitions for sfe.

5. Installation
    It is recommended to ensure that sfe driver works for your box before
    installing it.

(1) install sfe driver into the kernel directory
	# cd /.../sfe-x.x.x
        # /usr/ccs/bin/make install

    If you do not test sfe driver yet, execute following commands.
	    # ./adddrv.sh
	    # init 6	(reboot the system)
	    # devfsadm -i sfe (for solaris7, use drvconfig and reboot with -r )

(2) Configure the network interface. Create and/or modify the following file.
	/etc/hostname.sfeN	(N is an unit number)
    If your box is a dhcp client, you also need make an empty file named
    /etc/dhcp.sfeN.

(3) Reboot the system.
	# init 6

6. Upgrade
   If you have installed the driver, do following to upgrade.

(1) install new sfe driver into the kernel directory
	# cd /.../sfe-x.x.x
        # /usr/ccs/bin/make install

(2) Reboot the system.
	# init 6

7. Troubleshooting

Q. The system didn't boot after the driver was installed.
A. Uninstall the nic card and reboot solaris. Then, unregister the driver.
     # rem_drv sfe

   Or boot solaris with -a option and use /etc/system.nosfe instead of
   default [etc/system] to inhibit loading the driver.

   /etc/system.nosfe will be automatically generated while executing 'make
   install'

Q. Solaris booted, but the network interface didn't works.
A. Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for sfeN, the driver is loaded
   and working correctly.
   Please check the configuration on network interface showed by ifconfig.


   Is an ethernet cable connected properly?

   Please check if packets are receoved via the network interface.
	% snoop -P -d sfeN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv

  (3) Output of prtconf -vD

  (4) Output of adb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'adb -k unix.N vmcore.N' and type the following subcommands:
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)
