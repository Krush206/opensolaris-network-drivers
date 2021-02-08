***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is the Intel i8255x Ethernet NIC driver for Solaris.  It is distributed
under the BSD license.

2. Specification of the driver

File name of the driver: /kernel/drv/ife

Special file name:
	/dev/ifeN (where N is a unit number, typcally 0 for first card)
        For example
                % ifconfig ife0

Tested OS version
  Solaris9 x86 12/02

Tested chip and card
  Intel i82557 with DP83840 PHY, IBM Etherjet
  Intel i82557 with i82555 PHY,  Fujitsu FMW-188
  Intel i82559, Intel PRO100+ Management adaptor

3. Seting up Operating Environment

(1) Install your PCI card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd ife-x.x.x.tar.gz | tar xf -

(3) Add hostname for the nic card into /etc/hosts file

(4) Makeing binaries (for sparc platforms only)
Ife driver will work under 64bit and 32bit sparc solaris8 10/00 or
later, but this distribution does not include sparc binaries.
To make binaries, you need Sun C compiler or gcc, and do the following
operations before testing and installation:

(for sparc 64bit kernel with Sun C compilers)
        % cd /.../ife-x.x.x
        % rm Makefile
        % ln -s Makefile.sparcv9_suncc Makefile
        % rm obj
        % ln -s sparcv9 obj
        % /usr/ccs/bin/make

(for sparc 32bit kernel with Sun C compilers)
        % cd /.../ife-x.x.x
        % rm Makefile
        % ln -s Makefile.sparc_suncc Makefile
        % rm obj
        % ln -s sparc obj
        % /usr/ccs/bin/make

(for sparc 64bit kernel with gcc 3.x.x)
        % cd /.../ife-x.x.x
        % rm Makefile
        % ln -s Makefile.sparcv9_gcc Makefile
        % rm obj
        % ln -s sparcv9 obj
        % /usr/ccs/bin/make

(for sparc 32bit kernel with gcc)
        % cd /.../ife-x.x.x
        % rm Makefile
        % ln -s Makefile.sparc_gcc Makefile
        % rm obj
        % /usr/ccs/bin/make


4. Testing
  You need tp remove irpb driver for x86 solaris before install ife driver.
	# rem_drv iprb       ( for x86 solaris )
	# init 6
	( The system reboots here )
        # cd /.../ife-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# rm /kernel/drv/ife ( for solaris7, don't remove the file )
        # modload obj/ife
        # devfsadm -i ife    ( for solaris7, use drvconfig and reboot with -r)
        # ifconfig ifeN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for ife0 )
        # ifconfig ifeN HOSTNAME
        # ifconfig ifeN      ( ensure IP address is correct)
        # ifconfig ifeN up   ( and then you can test with ping, telnet, ftp ...)

5. Installation
    After you ensure that the ife driver is fully functional, install it.

(1) install the ife driver into the kernel directory
        # cd /.../ife-x.x.x
        # /usr/ccs/bin/make install

       If you do not test the ife driver yet, execute the following commands:
            # ./adddrv.sh
            # devfsadm -i ife ( for Solaris7, do drvconfig and reboot with -r )

(2) Configure the network interface. Create and/or modify the following files:
        /etc/hostname.ifeN

(3) Reboot the system.
        # init 6

6. How to install iprb driver again

	# rem_drv ife
	# init 6
	( The system reboots here )
        # cd /.../ife-x.x.x
	# ./adddrv_iprb.sh
	# devfsadm -i iprb

7. Troubleshooting

Q. Following message is printed in /var/adm/messages file:
    WARNING: ifeN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link parifer does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your nic card by specifying
   ife correct media mode in /kernel/drv/ife.conf with the following syntax:

   ifeN-duplex=["full"|"half"] ifeN-speed=[100|10]; # where N is a unit number

  For example:
	ife0-duplex="full" ife0-speed=100;   # full-duplex 100Mbps for ife0
	ife0-duplex="half" ife0-speed=10;    # half-duplex 10Mbps for ife0

Q. The driver cannot be unloaded because the device is busy, and the following
  message appears in /var/adm/messages file.
    NOTICE: ifeN: ife_detach: buffer is busy

A. Wait for a little while until some modules in the kernel release receive
  buffers which was allocated by the ife driver.

Q. The system does not boot after the driver is installed.
A. Uninstall the nic card and reboot solaris. Then, unregister the driver.
     # rem_drv ife

   Or boot solaris with -a option and use /etc/system.noife instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.noife is automatically generated when executing 'make install'

Q. Solaris boots, but the network interface does not work.
A. Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for ifeN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d ifeN

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
