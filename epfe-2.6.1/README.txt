***************************************************************************
CAUTION: This software is NO WARRANTY.
As this software is loaded into kernel, it might cause panic or hung.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

This is an alternative open source nic driver for SMC epic nics LAN83C17x
under Solaris. It is distributed under the BSD license.

2. Specification of the driver

File name of the driver: /kernel/drv/epfe

Special file name:
	/dev/epfeN (where N is an unit number, typcally 0 for first card)
        For example
                % ifconfig epfe0

Tested OS version
 Solaris nv x86 b90


3. Preparing for installation

(1) Unregister spwr driver with solaris
	# rem_drv spwr
	# init 0

(2) Install your PCI card and boot Solaris.

(3) Copy source and binary files.
        # gunzip -cd epfe-x.x.x.tar.gz | tar xf -

(4) Make links to the correct binary directory and Makefile according
to your configuration.  i386 and gcc are default.

        % cd /.../epfe-x.x.x
        % rm obj Makefile
        % ln -s Makefile.${KARCH}_${COMPILER} Makefile
        % ln -s ${KARCH} obj

  where ${KARCH} is the result of `isainfo -n`, and ${COMPILER} is
 "gcc" or "suncc" which you want to use to make the driver.

(5) Making binaries (only for re-compiling the drivers by your self)
To make the binaries, you need Sun C compiler or gcc version 3, and do
the following operations.

	% /usr/ccs/bin/make

(6) Making binaries only for OpenSolaris users.
The driver likely work with GLD v3 in OpenSolaris, known as Nemo.
You can enjoy the new functions by recompiling the source code.

        % rm Makefile.config
        % ln -s Makefile.config_gld3 Makefile.config
        % /usr/ccs/bin/make


4. Testing

        # cd /.../epfe-x.x.x
        # /usr/ccs/bin/make install
        # ./adddrv.sh
	# /usr/ccs/bin/make uninstall ( for solaris7, do not execute it )
        # modload obj/epfe
        # devfsadm -i epfe (for solaris7, use drvconfig and reboot with -r )
        # ifconfig epfeN plumb ( where N is unit number, typcally 0 for first card)
        # ifconfig -a        ( you will see an entry for epfe0, ensure ethernet address is correct)
        # ifconfig epfeN HOSTNAME
        # ifconfig epfeN      ( ensure IP address is correct)
        # ifconfig epfeN up   ( and then you can test ping, telnet, ftp ...)

5. Installation
    After you ensure that epfe driver is fully functional, install it.

(1) install epfe driver into kernel directory
        # cd /.../epfe-x.x.x
        # /usr/ccs/bin/make install

       If you do not test epfe driver yet, here execute following commands.
            # ./adddrv.sh
            # devfsadm -i epfe (for solaris7, use drvconfig then reboot with -r)

(2) Configure the network interface. Create and/or modify following files.
        /etc/hostname.epfeN

(3) Reboot the system.
        # init 6


6. Uninstallation

	# rem_drv epfe
	# ./addspwr.sh		# register swpr driver with solaris again.
	# init 0

7. Trouble shooting

Q. Following message is printed in /var/adm/messages file
    WARNING: epfeN: link up but auto-nego failed, it's funny.

A. This massage may appear when the link partner does not have auto
   negotiation capability.
   Please disable auto negotiation capability for your NIC card by specifying
  correct media mode in /kernel/drv/epfe.conf according to following syntax.

   full-duplex=[1|0] speed=[100|10]; # where N is an unit number

  For example:
	full-duplex=1 speed=100;   # full-duplex 100Mbps
	full-duplex=0 speed=10;    # half-duplex 10Mbps

Q. The driver cannot be unloaded because of device busy, and following
  message appears in /var/adm/messages file.
    NOTICE: epfeN: epfe_detach: buffer is busy

A. Wait for a little time until some modules in kernel release receive
  buffers which was allocated by epfe driver.

Q. The system does not boot after the driver was installed.
A. Uninstall the NIC card and reboot solaris. Then, unregister the driver.
     # rem_drv epfe 

   Or boot solaris with -a option and use /etc/system.noepfe instead of
   default [etc/system]. This inhibits loading the driver.

   /etc/system.noepfe is automatically generated while executing 'make install'
  or 'make test'

Q. Solaris boots, but the network interface does not work.
A.
   Is the network interface detected?

   Execute 'ifconfig -a'.  If you see an entry for epfeN, the driver is loaded
   and working correctly.
   Please check configuration for the network interface.


   Is ethernet cable connected correctly?

   Please check if the network interface receive packets.
	% snoop -P -d epfeN


   Change BIOS setting to disable ACPI or PnP.
   Operation is depend on your BIOS. Refer your BIOS manual.


If Solaris panics while testing, please send me following information.
  (1) /var/adm/messages

  (2) Output of prtconf -pv

  (3) Output of prtconf -vD

  (4) Output of adb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'mdb -k unix.N vmcore.N' and type following subcommands.
   $c   (stack trace back will be printed)
   $<msgbuf   (last messages are printed)
   ^D         (To quit adb, type control-D)

