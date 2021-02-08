***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

  This is the Realtek RTL8150 usb to ethernet driver for Solaris.  It is
distributed under the BSD license.

2. Specification of the driver

File name of the driver:
	/kernel/drv/urf
	/kernel/drv/sparcv9/urf (for sparc 64bit kernel)

Special file name:
	/dev/urfN (where N is a unit number, typcally 0 for first device)
        For example
                % ifconfig urf0

Tested OS version
 Solaris Express nv114  x86 with usbddk 0.9
 As urf driver requires new usb framework in solaris, it doesn't work under
 solaris9 or the previous version.

Tested chips and usb devices
 RTL8150, Melco BUFFALO LUA-KTX

3. Preparing for installation

(1) extract distributed source and binary files.
        # gunzip -cd urf-x.x.x.tar.gz | tar xf -

(2) Add hostname for the NIC device into the /etc/hosts file

(3) Making binaries (required for sparc platforms only)
  Urf driver is ready for 64bit and 32bit solaris10 sparc but not tested yet.
If you want to test or install it, you need to make
sparc binaries using Forte C compiler or gcc 3.x.x. Do the following
operations before testing or installation:

(for sparc 64bit kernel with Sun C compilers)
        % cd /.../urf-x.x.x
	% rm Makefile
	% ln -s Makefile.sparcv9_suncc Makefile
	% rm obj
	% ln -s sparcv9 obj
	% /usr/ccs/bin/make

(for sparc 32bit kernel with Sun C compilers)
        % cd /.../urf-x.x.x
	% rm Makefile
	% ln -s Makefile.sparc_suncc Makefile
	% rm obj
	% ln -s sparc obj
	% /usr/ccs/bin/make

(for sparc 64bit kernel with gcc3)
        % cd /.../urf-x.x.x
	% rm Makefile
	% ln -s Makefile.sparcv9_gcc Makefile
	% rm obj
	% ln -s sparcv9 obj
	% /usr/ccs/bin/make

(for sparc 32bit kernel with gcc)
        % cd /.../urf-x.x.x
	% rm Makefile
	% ln -s Makefile.sparc_gcc Makefile
	% rm obj
	% ln -s sparc obj
	% /usr/ccs/bin/make

4. Installation

 (1) goto the directory of distributed files.
      # cd ..../urf-x.x.x

 (2) register the urf driver with solaris.
	# /usr/ccs/bin/make install
        # ./adddrv.sh

 (3) install your usb device to the PC.

 (4) set up the network port 
        # ifconfig urfN plumb ( where N is unit number, typcally 0 for first device)
        # ifconfig -a        ( you will see an entry for urf0, ensure ethernet address is correct)
        # ifconfig urfN HOSTNAME
        # ifconfig urfN      ( ensure IP address is correct)
        # ifconfig urfN up   ( and then you can test with ping, telnet, ftp ...)

  To configure the network interface on boot-time automatically. Create and/or
  modify the following file.
        /etc/hostname.urfN

5. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    NOTE: urfN: link up but auto-negotiation failed.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC device by specifying
   the correct media mode in /kernel/drv/urf.conf with the following syntax:

   full-duplex=[1|0] speed=[100|10];

  For example
	full-duplex=1 speed=100;   # full-duplex 100Mbps
	full-duplex=0 speed=10;    # half-duplex 10Mbps

Q. The system does not boot after the driver is installed.
A. Uninstall the usb device and reboot solaris. Then, unregister the driver:
     # rem_drv urf 

Q. Solaris boots, but the network interface does not work.
A. Is the network interface running?

   Execute 'ifconfig -a'.  If you see an entry for urfN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d urfN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv when the usb device is inserted in a usb slot.

  (3) Output of prtconf -vD when the usb device is inserted in a usb slot.

  (4) Output of adb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'mdb -k unix.N vmcore.N' and type the following subcommands.
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)
