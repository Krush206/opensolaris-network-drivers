***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

  This is ASIX AX88178a/179 usb to giga ethernet driver for OpenSolaris.
It is distributed under the BSD license.

2. Specification of the driver

File name of the driver:
	/kernel/drv/axg
	/kernel/drv/amd64/axg

Special file name:
	/dev/axgN (where N is a unit number, typcally 0 for first device)
        For example
                % ifconfig axg0

Tested OS version
 OpenIndiana 151a9

Tested chip set and usb devices
 AX88179, Buffalo LUA4-U3-AGT

3. Preparing for installation

(1) extract distributed source and binary files.
        # gunzip -cd axg-x.x.x.tar.gz | tar xf -

(2) Add hostname for the NIC device into the /etc/hosts file

(3) Making binaries (not required if you use binaries included in the tar ball )

        % cd /.../axg-x.x.x
	% /usr/ccs/bin/make

4. Installation

 (1) goto the directory of distributed files.
	# cd .../axg-x.x.x

 (2) register the axg driver with OpenSolaris.
	# /usr/ccs/bin/make install
        # ./adddrv.sh

 (3) reboot your system
	# init 6

 (4) install your usb device to the PC.

 (5) set up the network port 
        # ifconfig axgN plumb ( where N is unit number, typcally 0 for first device)
        # ifconfig -a        ( you will see an entry for axg0, ensure ethernet address is correct)
        # ifconfig axgN HOSTNAME
        # ifconfig axgN      ( ensure IP address is correct)
        # ifconfig axgN up   ( and then you can test with ping, telnet, ftp ...)

  To configure the network interface on boot-time automatically. Create and/or
  modify the following file.
        /etc/hostname.axgN

5. Troubleshooting

Q. Following message is printed in the /var/adm/messages file
    NOTE: axgN: link up but auto-negotiation failed.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC device by specifying
   the correct media mode in /kernel/drv/axg.conf with the following syntax:

   full-duplex=[0 | 1] speed=[100|10];

  For example
	full-duplex=1 speed=100;   # full-duplex 100Mbps
	full-duplex=0 speed=10;    # half-duplex 10Mbps

Q. The system does not boot after the driver is installed.
A. Uninstall the usb device and reboot solaris. Then, unregister the driver:
     # rem_drv axg 

Q. Solaris boots, but the network interface does not work.
A. Is the network interface running?

   Execute 'ifconfig -a'.  If you see an entry for axgN, the driver is loaded
   and working correctly.
   Please check the configuration of the network interface.


   Is an ethernet cable connected properly?

   Please check if the network interface is receiving packets.
	% snoop -P -d axgN

If Solaris panics during you test, please send me the following information:
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
