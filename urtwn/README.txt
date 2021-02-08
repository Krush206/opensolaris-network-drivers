***************************************************************************
CAUTION: There is NO WARRANTY for this software.
As this software is loaded into the kernel, it might cause the kernel to
panic or hang.

                                                Masayuki Murayama
                                                KHF04453@nifty.ne.jp
***************************************************************************


1. Introduction

  This is the realtek RTL8188CU/RTL8192CU usb2.0 to 802.11 wifi controller
driver for OpenSolaris. It is distributed under the BSD license.

2. Specification of the driver

File name of the driver:
	/kernel/drv/urtwn

Special file name:
	/dev/urtwnN (N is a unit number, typcally 0 for the first device)
        For example
                % ifconfig urtwn0

Tested OS version
 openindiana 151a
 solaris11

Tested chips and usb devices
 RTL8188CU, Planex communications GW-USNano2
 RTL8188CU, Oar Inc SW-WF02-AD15
 RTL8192CU, Planex GW-USEco300

3. Installation

 (1) goto the directory where files in the tarball were extracted.
	# cd .../urtwn-x.x.x

 (2) register the urtwn driver with solaris.
	# /usr/ccs/bin/make install
        # ./adddrv.sh
	# devfsadm -i urtwn	( ingnore error messages )

 (3) shutdown your pc
	# init 0

 (4) install your usb device to your PC.

 (5) boot your pc

 (6) configure the urtwn driver according with tools in openindiana/solaris11
     ( I recommend nwam gui tool )

4. Making binaries (optional)
  If you want the binary of urtwn driver by yourself, do below.

  (1) download system headers
    # pkg install system/header
    # pkg install system/header/header-usb (only for openindiana)

  (2) download compiler
    # pkg install gcc-3

  (3) compile the source code of urtwn driver
    $ /usr/ccs/bin/make clean
    $ /usr/ccs/bin/make

5. Troubleshooting

If Solaris panics during you test, please send me the following information:
  (1) /var/adm/messages

  (2) Output of prtconf -v when the usb device is inserted in a usb slot.

  (3) Output of prtconf -vD when the usb device is inserted in a usb slot.

  (4) Output of mdb
  Solaris core dump consists of unix.N and vmcore.N which are created in
   /var/crash/YOUR-HOST-NAME/ .

  Execute 'mdb -k unix.N vmcore.N' and type the following subcommands.
   $c   (stack trace back will be printed)
   ::msgbuf   (last messages are printed)
   ::threadlist -v (stack traves for all thread in the kernel)
   ^D         (To quit mdb, type control-D)
