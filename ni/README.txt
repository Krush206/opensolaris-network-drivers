***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

						Masayuki Murayama
						KHF04453@nifty.ne.jp
***************************************************************************


Chapter 1. For PCI and PnP ISA cards

1. Introduction

This is the NE2000 compatible PCI NIC driver of Solaris. It is distributed
under the BSD license.


2. Tested NE2000 compatible PCI NIC chips

There are several NE2000 compatible PCI NIC chips. You should know the part
number of your NIC chip. You can do it by several ways. You may find
it by Windows Device Manager or seeing the surface of the chip by your eyes.

The chips that I tested are:
  (1) VIA 82C926 Amazon
  (2) Realtek 8029

It will likely work with chips/cards below, but untested.
  (1) Winbond 89C940, 89C940F
  (2) Compex ReadyLink 2000
  (3) KTI ET32P2
  (4) NetVin NV5000
  (5) SureCom NE34

If your card isn't one of PCI cards above, add its node name like
pciXXXX,YYYY  into /etc/driver_aliases. For ISA PnP cards, the name
is pnpXXX,YYYY. You can know its node name by "prtconf -pv".
In Solaris, the node name is a pair of PCI subsystem-vendor-id and
subsystem-id of the PCI card.


3. Specifications of this driver

 File name of driver:
	/kernel/drv/ni
	/kernel/drv/dp8390 (internally used by ni)

 Special file name:
	/dev/niN (where N is a unit number, typcally 0 for first card)

	For example:
		% ifconfig ni0

 Tested OS versions:
	Solaris10 x86 3/10


4. Set up Operating Environment

(1) Install your PCI card and boot Solaris.

(2) Extract source and binary files. (You have done this.)
        # gunzip -cd ni-x.x.x.tar.gz | tar xf -

(3) Add hostname for the NIC card into /etc/hosts file

(4) Make links to correct binary directory and Makefile according to your
configuration.  i386 and gcc are default.

        % cd /.../ni-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(5) Making binaries (only if you want to compile source code)

        % /usr/ccs/bin/make

5. Testing

        # cd /.../ni-x.x.x
        # /usr/ccs/bin/make install
        # ./addni.sh
	# /usr/ccs/bin/make uninstall (for solaris7, don't execute this)

   If your NIC card is not listed in section 2, add it into
   /etc/driver_aliases file with following form,
       ni "niXXXX,YYYYY"

        # modload obj/ni
        # devfsadm -i ni    (for solaris7, use drvconfig and reboot with -r)
        # ifconfig niN plumb
        # ifconfig -a       (you will see an entry for niN, ensure ethernet address is correct)
        # ifconfig niN HOSTNAME
        # ifconfig niN      (ensure IP address is correct)
        # ifconfig niN up   (and then you can test with ping, telnet, ftp ...)

6. Installation
    After you ensure that the ni driver is fully functional, install it.

(1) install ni driver into the kernel directory
        # cd /.../ni-x.x.x
        # /usr/ccs/bin/make install

    If you do not test ni driver yet, execute the following commands:
            # ./addni.sh
            # devfsadm -i ni (for solaris7, use drvconfig and reboot with -r)

(2) Configure the network interface. Create and/or modify the following file:
        /etc/hostname.niN  (Where N is a unit number)

(3) Reboot the system.
        # init 6

6. Troubleshooting

Q. The system doen not boot after the driver installed.
A. Boot the system with /etc/system.noni file.
   'Make intall' generates alternative /etc/system file that named
   /etc/system.noni to inhibit loading this driver.

   Boot the system with -a option and then specify system.noni instead of
  /etc/system file.
   After the system booted, uninstall the driver.

  Another way to boot the system, uninstall the nic card.

Q. How to uninstall the driver.
A.   # rem_drv ni
     # rem_drv pcni

Q. The system rebooted, but the network device does not work.
A. Is the device identified?
    Do 'ifconfig -a'.  Is niN displayed?

    If niN entry is displayed, the driver detected correctly. You should
    ensure the network configuration of the device.

 o  Is an ethernet cable connected properly?
    You can ensure if the device is receiving packets by using
    'snoop -d /dev/niN'

=================================================================

Chapter 2 :  For PCMCIA cards


1. Introduction

  To install the driver, You should know ID string of your PCMCIA
NIC pard. It is a pair of the Manifacture ID and Product ID of
the card and represented as pccardXXXX,YYYY.

 PCMCIA Ether Cards that I tested are :
   Planex Communications FNW-3600-T
      ID string is pccard149,c1ab

   Planex Communications FNW-3503-T
      ID string is pccard149,c1ab

   IBM Credit Card Adapter Ethernet II
      ID string is pccarda4,2

2. How to find ID of your PCIMCIA NIC card.

Do 'prtconf -v' and find a device node named 'network' under 'pcic' node.
And you will see 'pccardXXXX,YYYY' string  in the property list of the node.
This is ID string of your PCMCIA card.

3. Specification

 File names of drivers:
	/kernel/drv/pcni	(for NE2000 compatible pccards)
	/kernel/drv/dp8390	(8390 core, internally used)

 Special file name:
	/dev/pcniN (where N is slot number, typcally 0 or 1 )

	 For example
		% ifconfig pcniN	where N is a PCMCIA slot number

 CAUTION: Hot-plugging is not supported. Please insert the card into a
       PCMCIA slot before the system is turned on.

 Tested OS version:
	x86 Solaris 8

4. Preparing for installation

(1) Install your PCMCIA card and boot Solaris.

(2) Copy source and binary files.
        # gunzip -cd ni-x.x.x.tar.gz | tar xf -

(3) Add hostname for the PCMCIA NIC card into /etc/hosts file

(4) Create pcni.conf file to define MAC address for the card.

    Make /kernel/drv/pcni.conf file which includes following line, that
    defines a MAC address for your NIC card. It will be used when the
    driver cannot get a factory MAC address from the card.

    pcniN-mac-addr="XX:XX:XX:XX:XX:XX";

    Where N is a socket number in which your NIC card is inserted,
    typically 0 or 1, and XX:XX:XX:XX:XX is a 6-byte MAC address in
    hexdecimal format, like 00:01:12:23:34:ab. If you don't know it for
    the card, use "00:00:00:00:00:00".  It makes pcni driver to generate
    a local MAC address automatically.  Do not use it for long time
    because it isn't authorized.

5. Testing

        # cd /.../ni-x.x.x
        # /usr/ccs/bin/make install
        # ./addpcni.sh
	# /usr/ccs/bin/make uninstall

    Here, add the ID string of your pccard (pccardXXXX,YYYY) into
    /etc/driver_aliases as below.

        pcni "pccardXXXX,YYYY"

        # modload obj/pcni
        # devfsadm -i pcni
        # ifconfig pcniN plumb
        # ifconfig -a        ( you will see an entry for pcniN,
			       ensure ethernet address is correct)
        # ifconfig pcniN HOSTNAME
        # ifconfig pcniN      ( ensure IP address is correct)
        # ifconfig pcniN up   ( and then you can test ping, telnet, ftp ...)


6. Installation
    After you ensure that the ni driver is fully functional, install it.

(1) install ni driver into kernel directory
        # cd /.../ni-x.x.x
        # /usr/ccs/bin/make install

    If you do not test pcni driver yet, execute the following commands:
            # ./addpcni.sh
            # devfsadm -i pcni

(2) Configure the network interface. Create and/or modify the following files.
        /etc/hostname.pcniN

(3) Reboot the system.
        # init 6
