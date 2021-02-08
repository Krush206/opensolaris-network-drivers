***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

  This is the admtek pegasus usb to fast ethernet driver for Solaris.
It is distributed under the BSD license.

2. Specification of the driver

File name of the driver:
	/kernel/drv/upf
	/kernel/drv/sparcv9/upf (for sparc 64bit kernel)

Special file name:
	/dev/upfN (where N is a unit number, typcally 0 for first device)
        For example
                % ifconfig upf0

Tested OS version
 Solaris11 x86
 As upf driver requires new usb framework in solaris, it doesn't work under
 solaris9 or the previous version.

Tested chips and usb devices
  ADM8515 -- IO DATA USB ETX2-US2
  ADM8511 -- Planex communications UE-200TX usb to ethernet adapter
  AN986   -- Melco BUFFALO LUA-TX usb to ethernet adapter


3. Preparing for installation

(1) extract distributed source and binary files.
        # gunzip -cd upf-x.x.x.tar.gz | tar xf -

(2) Add hostname for the NIC device into the /etc/hosts file

(3) Making binaries (required for sparc platforms only)
  Urf driver is ready for 64bit and 32bit solaris10 sparc but not tested yet.
If you want to test or install it, you need to make
sparc binaries using Forte C compiler or gcc 3.x.x. Do the following
operations before testing or installation:

(for sparc 64bit kernel with Sun C compilers)
        % cd /.../upf-x.x.x
	% rm Makefile
	% ln -s Makefile.sparcv9_suncc Makefile
	% rm obj
	% ln -s sparcv9 obj
	% /usr/ccs/bin/make

(for sparc 32bit kernel with Sun C compilers)
        % cd /.../upf-x.x.x
	% rm Makefile
	% ln -s Makefile.sparc_suncc Makefile
	% rm obj
	% ln -s sparc obj
	% /usr/ccs/bin/make

(for sparc 64bit kernel with gcc3)
        % cd /.../upf-x.x.x
	% rm Makefile
	% ln -s Makefile.sparcv9_gcc Makefile
	% rm obj
	% ln -s sparcv9 obj
	% /usr/ccs/bin/make

(for sparc 32bit kernel with gcc)
        % cd /.../upf-x.x.x
	% rm Makefile
	% ln -s Makefile.sparc_gcc Makefile
	% rm obj
	% ln -s sparc obj
	% /usr/ccs/bin/make

4. Installation

 (1) goto the directory of distributed files.
	# cd ...../upf-x.x.x

 (2) register the upf driver with solaris.
	# /usr/ccs/bin/make install
        # ./adddrv.sh

 (3) install your usb device to the PC.

 (4) set up the network port 
        # ifconfig upfN plumb ( where N is unit number, typcally 0 for first device)
        # ifconfig -a        ( you will see an entry for upf0, ensure ethernet address is correct)
        # ifconfig upfN HOSTNAME
        # ifconfig upfN      ( ensure IP address is correct)
        # ifconfig upfN up   ( and then you can test with ping, telnet, ftp ...)

  To configure the network interface on boot-time automatically. Create and/or
  modify the following file.
        /etc/hostname.upfN

5. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    NOTE: upfN: link up but auto-negotiation failed.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC device by specifying
   the correct media mode in /kernel/drv/upf.conf with the following syntax:

   full-duplex=[1|0] speed=[100|10];

  For example
	full-duplex=1 speed=100;   # full-duplex 100Mbps
	full-duplex=0 speed=10;    # half-duplex 10Mbps

Q. The system does not boot after the driver is installed.
A. Uninstall the usb device and reboot solaris. Then, unregister the driver:
     # rem_drv upf 

Q. Solaris boots, but the network interface does not work.
A. Is the network interface running?

   Execute 'ifconfig -a'.  If you see an entry for upfN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d upfN

If Solaris panics while testing, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -pv when the usb device is inserted in a usb slot.

  (3) Output of prtconf -vD when the usb device is inserted in a usb slot.

  (4) Output of mdb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'mdb -k unix.N vmcore.N' and type the following subcommands.
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit mdb, type control-D)
