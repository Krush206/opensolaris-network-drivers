#
# Top level makefile for the NVIDIA kernel module; this file selects
# which "real" Makefile should actually be used to build the kernel
# module (either the "old" nvidia Makefile, or the kbuild makefile),
# and then passes the target on to that Makefile.
#

ifndef SYSSRC
  ifdef SYSINCLUDE
    KERNEL_HEADERS := $(SYSINCLUDE)
    KERNEL_SOURCES := $(KERNEL_HEADERS)/..
  else
    KERNEL_SOURCES := /lib/modules/$(shell uname -r)/build
    KERNEL_HEADERS := $(KERNEL_SOURCES)/include
  endif
else
    KERNEL_SOURCES := $(SYSSRC)
    KERNEL_HEADERS := $(KERNEL_SOURCES)/include
endif

ifndef SYSOUT
  KERNEL_OUTPUT := $(KERNEL_SOURCES)
else
  #
  # Support separate KBUILD output directories; SYSOUT
  # needs to reflect the correct KBUILD_OUTPUT.
  #
  KERNEL_OUTPUT := $(SYSOUT)
endif

CC ?= cc
CONFTEST := sh ./conftest.sh "$(CC)" $(KERNEL_SOURCES) $(KERNEL_OUTPUT)

# These two rules are designed to catch most build targets, set up the
# appropriate makefile and then re-invoke make with that makefile. Two rules
# appear to be needed because a .DEFAULT rule does not override make's
# implicit rule for .o files.

module print-module-filename:
	@ $(MAKE) --no-print-directory select_makefile
	@ $(MAKE) --no-print-directory -f Makefile $(MAKECMDGOALS)

%.o %.ko:
	@ $(MAKE) --no-print-directory select_makefile
	@ $(MAKE) --no-print-directory -f Makefile $(MAKECMDGOALS)

select_makefile:
	@ $(CONFTEST) select_makefile full_output

install uninstall:
	@ if [ -f Makefile ]; then \
	  $(MAKE) --no-print-directory -f Makefile $@; \
	fi; \

clean:
	@ if [ -f Makefile ]; then \
	  $(MAKE) --no-print-directory -f Makefile $@; \
	fi; \
	rm -f Makefile

.PHONY: select_makefile module print-module-filename

