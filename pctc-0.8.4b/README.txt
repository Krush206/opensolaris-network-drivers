***************************************************************************
CAUTION: This software is NO WARRANTY.
As this software is loaded into kernel, it might cause panic or hung.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is a 3com 3c589 PCMCIA NIC driver of Solaris. It is distributed under
LGPL license.

2. Specifications of the driver

File name of the driver: /kernel/drv/pctc

Special file name: /dev/pctcN    (where N is slot number)
	For example
		% ifconfig pctc0

3COM 3C562 is not soppored.

Hot plugging is not supported. Your pccard should be inserted into a
PCMCIA slot before the system is turned on.

Default media is 10BASE-T. If you want to use other media i.e. AUI
or 10BASE2, create /kernel/drv/pctc.conf file that include a line
below.

pctcN-media-type=M;   ( where N is slot number and
			M is 0 for 10BASE-T, 1 for AUI, 3 for 10BASE2)

Tested 3c589 cards are
 3Com EtherLink III 3C589C	(3c589 C version)
 3Com EtherLink III 3C589D	(3c589 D version)
 3Com Megahertz Model 3CCE589ET	(3c589 E version)

Tested OS version
 x86 Solaris8 10/00

Tested hardware
 Toshiba SS 3380 (with TOPIC97 i82365 compatible mode)


3. Installation

(1) Unregister pcelx and shutdown the system
	# rem_drv pcelx
	# halt

(2) Power-off, insert the pccard, and boot solaris again.

(3) Add hostname for the NIC card into /etc/hosts file

(4) Copy driver files.
	# gunzip -cd pctc-x.x.x.tgz | tar xf -

(5) Copy driver into /kernel/drv directory
	# cd pctc-x.x.x
	# /usr/ccs/bin/make install

(6) Register this driver
	# ./addpctc.sh

(7) Force the driver loaded
	# devfsadm -i pctc
	# ifconfig pctcN plumb
	# ifconfig -a   ( you will see an entry for pctcN )
	# ifconfig pctcN HOSTNAME 
	# ifconfig pctcN  ( ensure IP address is correct )
	# ifconfig pctcN up ( and then you can test ping, telnet, ftp ... )

(8) Configure the network interface. Create following file.
	/etc/hostname.pctcN
	
(9) Reboot the system
	# init 6


4. Trouble shooting

Q. The system panics while booting after the driver was installed.
A.
   Unregister the driver after the system booted without the pccard.
     # rem_drv pctc

   If you want to regisger pcecx driver again, execute addpcelx.sh.

Q. The system boots, but the network interface does not work.
A.
   Is the pccard detected?

      Execute 'ifconfig -a'. If you see an entry for pctcN, the
   driver works correctly. Please check the configuration for the network
   interface.


   Is the ethernet cable connected correctly?

   Please check if the network interface receive packets.
	% snoop -P -d pctcN
