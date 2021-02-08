#!/bin/bash

# make sure we are in the directory containing this script
SCRIPTDIR=`dirname $0`
cd $SCRIPTDIR
PATH="${PATH}:/bin:/sbin"

CC="$1"
ISYSTEM=`$CC -print-file-name=include`
SOURCES=$2
HEADERS=$SOURCES/include
OUTPUT=$3

CFLAGS="-D__KERNEL__ \
-nostdinc -isystem $ISYSTEM \
-Werror -Wimplicit-function-declaration"

if [ "$OUTPUT" != "$SOURCES" ]; then
    CFLAGS="$CFLAGS -I$OUTPUT/include2 -I$OUTPUT/include \
-I$HEADERS -I$HEADERS/asm/mach-default"
else
    CFLAGS="$CFLAGS -I$HEADERS -I$HEADERS/asm/mach-default"
fi

case "$4" in
    remap_page_range)
        #
        # Determine the number of arguments expected by remap_page_range.
        #

        echo "#include <linux/mm.h>
        int do_test_remap_page_range(void) {
           pgprot_t pgprot;
           remap_page_range(NULL, 0L, 0L, 0L, pgprot);
        }" > conftest$$.c

        $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
        rm -f conftest$$.c

        if [ -f conftest$$.o ]; then
          echo "5"
          rm -f conftest$$.o
          exit 0
        fi

        echo "#include <linux/mm.h>
        int do_test_remap_page_range(void) {
           pgprot_t pgprot;
           remap_page_range(0L, 0L, 0L, pgprot);
        }" > conftest$$.c

        $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
        rm -f conftest$$.c

        if [ -f conftest$$.o ]; then
          echo "4"
          rm -f conftest$$.o
          exit 0
        else
          #
          # We couldn't determine the number of arguments expected by the
          # remap_page_range function.
          #
          exit 1
        fi
    ;;

    cc_sanity_check)
        #
        # Verify that the same compiler is used for the kernel and kernel
        # module.
        #
        VERBOSE=$5
        
        if test -n "$IGNORE_CC_MISMATCH" -o -n "$SYSSRC" -o -n "$SYSINCLUDE"; then
          #
          # The user chose to disable the CC sanity test (which may or
          # may not be wise) or is building the module for a kernel not
          # currently running, which renders our test meaningless.
          #
          exit 0
        fi

        # usage: conftest.sh [cc_sanity_check] [full_output|just_msg] [$CC]

        rm -f gcc-version-check
        $CC gcc-version-check.c -o gcc-version-check > /dev/null 2>&1
        if [ -f gcc-version-check ]; then
            PROC_VERSION=`cat /proc/version`
            MSG=`./gcc-version-check "$PROC_VERSION"`
            RET=$?
            rm -f gcc-version-check
        else
            MSG="Could not compile gcc-version-check.c"
            RET=1
        fi

        if [ "$RET" != "0" ]; then
            #
            # The gcc version check failed
            #
            
            if [ "$VERBOSE" = "full_output" ]; then
                echo "";
                echo "gcc-version-check failed:";
                echo "";
                echo "$MSG" | fmt -w 60
                echo "";
                echo "If you know what you are doing and want to override";
                echo "the gcc version check, you can do so by setting the";
                echo "IGNORE_CC_MISMATCH environment variable to \"1\".";
                echo "";
                echo "In any other case, set the CC environment variable";
                echo "to the name of the compiler that was used to compile";
                echo "the kernel.";
                echo ""
                echo -e  "*** Failed cc sanity check. Bailing out! ***";
                echo "";
            else
                echo "$MSG"
            fi
            exit 1;
        else
            exit 0
        fi
    ;;

    kernel_patch_level)
        #
        # Determine the kernel's major patch level; this is only done if we
        # aren't told by KBUILD.
        #

        echo $(grep "^PATCHLEVEL =" $SOURCES/Makefile | cut -d " " -f 3)
        exit 0
    ;;

    suser_sanity_check)
        #
        # Determine the caller's user id to determine if we have sufficient
        # privileges for the requested operation.
        #
        if test $(id -ur) != 0; then
            echo "";
            echo "Please run \"make install\" as root.";
            echo "";
            echo -e  "*** Failed super-user sanity check. Bailing out! ***";
            exit 1
        else
            exit 0
        fi
    ;;

    rmmod_sanity_check)
        #
        # Make sure that any currently loaded NVIDIA kernel module can be
        # unloaded.
        #
        MODULE="nvidia"

        if test -n "$SYSSRC" -o -n "$SYSINCLUDE"; then
          #
          # Don't attempt to remove the kernel module if we're not
          # building against the running kernel.
          #
          exit 0
        fi

        if lsmod | grep -wq $MODULE; then
          rmmod $MODULE >& /dev/null
        fi

        if lsmod | grep -wq $MODULE; then
            #
            # The NVIDIA kernel module is still loaded, most likely because
            # it is busy.
            #
            echo "";
            echo "Unable to remove existing NVIDIA kernel module.";
            echo "Please be sure you have exited X before attempting";
            echo "to install the NVIDIA kernel module.";
            echo "";
            echo -e  "*** Failed rmmod sanity check. Bailing out! ***";
            exit 1
        else
            exit 0
        fi
    ;;

    select_makefile)
        #
        # Select which Makefile to use based on the version of the
        # kernel we are building against: use the kbuild Makefile for
        # 2.6 and newer kernels, and the old Makefile for kernels older
        # than 2.6.
        #
        rm -f Makefile
        RET=1
        VERBOSE=$5
        FILE="linux/version.h"

        if [ -f $HEADERS/$FILE -o -f $OUTPUT/include/$FILE ]; then
            #
            # We are either looking at a configured kernel source
            # tree or at headers shipped for a specific kernel.
            # Determine the kernel version using a compile check.
            #
            echo "#include \"linux/version.h\"
            int main() {
              if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) {
                return 0;
              } else {
                return 1;
              }
            }" > conftest$$.c

            gcc conftest$$.c -o conftest$$ -nostdinc -I $HEADERS > /dev/null 2>&1
            rm -f conftest$$.c

            if [ -f conftest$$ ]; then
                ./conftest$$ > /dev/null 2>&1
                if [ $? = "0" ]; then
                    ln -s Makefile.kbuild Makefile
                else
                    ln -s Makefile.nvidia Makefile
                fi
                rm -f conftest$$
                RET=0
            fi
        else
            MAKEFILE=$HEADERS/../Makefile
            CONFIG=$HEADERS/../.config

            if [ -f $MAKEFILE -a -f $CONFIG ]; then
                #
                # This source tree is not configured, but includes
                # a Makefile and a .config file. If this is a 2.6
                # kernel older than 2.6.6, that's all we require to
                # build our module.
                #
                PATCHLEVEL=$(grep "^PATCHLEVEL =" $MAKEFILE | cut -d " " -f 3)
                SUBLEVEL=$(grep "^SUBLEVEL =" $MAKEFILE | cut -d " " -f 3)

                if [ $PATCHLEVEL -ge 6 -a $SUBLEVEL -le 5 ]; then
                    ln -s Makefile.kbuild Makefile
                    RET=0
                fi
            fi
        fi

        if [ "$RET" != "0" ]; then
            echo "";
            echo "If you are using a Linux 2.4 kernel, please make sure";
            echo "you either have configured kernel sources matching your";
            echo "kernel or the correct set of kernel headers installed";
            echo "on your system.";
            echo "";
            echo "If you are using a Linux 2.6 kernel, please make sure";
            echo "you have configured kernel sources matching your kernel";
            echo "installed on your system. If you specified a separate";
            echo "output directory using either the \"KBUILD_OUTPUT\" or";
            echo "the \"O\" KBUILD parameter, make sure to specify this";
            echo "directory with the SYSOUT environment variable or with";
            echo "the appropriate nvidia-installer command line option.";
            echo "";
            if [ "$VERBOSE" = "full_output" ]; then
                echo "*** Unable to determine the target kernel version. ***";
                echo "";
            fi
            exit 1
        else
            exit 0
        fi
    ;;

    get_uname)
        #
        # print UTS_RELEASE from the kernel sources that we are building
        # against; if an error occurs, give up and just return `uname -r`
        #

        echo "#include \"linux/version.h\"
        int main() {
            printf(\"%s\n\", UTS_RELEASE);
            return 0;
        }" > conftest$$.c

        CC $CFLAGS -o conftest$$ conftest$$.c > /dev/null 2>&1
        rm -f conftest$$.c

        if [ -f conftest$$ ]; then
            ./conftest$$
            rm -f conftest$$
            exit 0
        else
            uname -r
            exit 1
        fi
    ;;

    rivafb_sanity_check)
        #
        # Check if the kernel was compiled with rivafb support. If so, then
        # exit, since our driver no longer works with rivafb.
        #
        RET=1
        VERBOSE=$5
        FILE="linux/autoconf.h"

        if [ -f $HEADERS/$FILE -o -f $OUTPUT/include/$FILE ]; then
            #
            # We are looking at a configured source tree; verify
            # that its configuration doesn't include rivafb using
            # a compile check.
            #
            echo "#include \"linux/autoconf.h\"
            #ifdef CONFIG_FB_RIVA
            #error CONFIG_FB_RIVA defined!!
            #endif
            " > conftest$$.c

            $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
            rm -f conftest$$.c

            if [ -f conftest$$.o ]; then
                rm -f conftest$$.o
                RET=0
            fi
        else
            CONFIG=$HEADERS/../.config
            if [ -f $CONFIG ]; then
                if [ -z "$(grep "^CONFIG_FB_RIVA=y" $CONFIG)" ]; then
                    RET=0
                fi
            fi
        fi

        if [ "$RET" != "0" ]; then
            echo "Your kernel was configured to include rivafb support!";
            echo "";
            echo "The rivafb driver conflicts with the NVIDIA driver, please";
            echo "reconfigure your kernel and *disable* rivafb support, then";
            echo "try installing the NVIDIA kernel module again.";
            echo "";
            if [ "$VERBOSE" = "full_output" ]; then
                echo -e  "*** Failed rivafb sanity check. Bailing out! ***";
                echo "";
            fi
            exit 1
        else
            exit 0
        fi
    ;;

    rivafb_module_sanity_check)
        #
        # Check if the kernel was compiled with rivafb support as a module.
        # If so, notify the user that the two are not compatible. Don't
        # fail in this case, as many distros are likely to include this as
        # a module.
        #
        RET=1
        VERBOSE=$4
        FILE="linux/autoconf.h"

        if [ -f $HEADERS/$FILE -o -f $OUTPUT/include/$FILE ]; then
            #
            # We are looking at a configured source tree; verify
            # that its configuration doesn't include rivafb using
            # a compile check.
            #
            echo "#include \"linux/autoconf.h\"
            #ifdef CONFIG_FB_RIVA_MODULE
            #error CONFIG_FB_RIVA_MODULE defined!!
            #endif
            " > conftest$$.c

            $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
            rm -f conftest$$.c

            if [ -f conftest$$.o ]; then
                rm -f conftest$$.o
                RET=0
            fi
        else
            CONFIG=$HEADERS/../.config
            if [ -f $CONFIG ]; then
                if [ -z "$(grep "^CONFIG_FB_RIVA=m" $CONFIG)" ]; then
                    RET=0
                fi
            fi
        fi

        if [ "$RET" != "0" ]; then
            echo "";
            echo "Your kernel was configured to include rivafb support as";
            echo "a loadable kernel module.";
            echo "";
            echo "The rivafb driver conflicts with the NVIDIA driver; the";
            echo "NVIDIA kernel module will still be built and installed,";
            echo "but be aware that the NVIDIA driver will not be able to";
            echo "function properly if the rivafb module is loaded!";
            echo "";
            if [ "$VERBOSE" = "full_output" ]; then
                echo -en "*** Failed rivafb module sanity check, but ";
                echo -e  "continuing! ***";
                echo "";
            fi
        fi

        exit 0
    ;;

    change_page_attr)
        #
        # Determine if change_page_attr() is present
        #
        
        echo "#include <linux/version.h>
        #include <linux/mm.h>
        #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
          #include <asm/cacheflush.h>
        #endif
        int test_cpattr(struct page *pp, int i, pgprot_t prot) {
            return change_page_attr(pp, i, prot);
        }" > conftest$$.c

        $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
        rm -f conftest$$.c

        if [ -f conftest$$.o ]; then
            rm -f conftest$$.o
            echo 1
        else
            echo 0
        fi
    ;;

    class_simple_create)
        #
        # Determine if class_simple_create() is present.
        #

        echo "#include <linux/device.h>
        struct class_simple*
        test_class_create(struct module *owner, char *name) {
            return class_simple_create(owner, name);
        }" > conftest$$.c

        $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
        rm -f conftest$$.c

        if [ -f conftest$$.o ]; then
            rm -f conftest$$.o
            echo 1
        else
            echo 0
        fi
    ;;

esac
