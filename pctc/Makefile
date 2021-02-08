VERSION	= 0.8.4
BINDIR	= ./i386
DISTDIR	= ../pctc-${VERSION}
CFLAGS	= -O -D__STDC__=0 -U_NO_LONGLONG -D_KERNEL -DDEBUG

TC589_FLAGS =	-DDEBUG_LEVEL=1 -DTX_TIMEOUT_CHECK \
		-DVERSION='"${VERSION}"' -DUNDERRUN_TEST -UTIMEOUT_TEST

CC	= gcc

all:	${BINDIR}/pctc

${BINDIR}/pctc:	${BINDIR}/pctc.o
	ld -r ${BINDIR}/pctc.o -o $@

${BINDIR}/pctc.o:	pctc.c
	$(CC) -c ${CFLAGS} ${TC589_FLAGS} pctc.c -o $@

install:
	/usr/sbin/install -f /kernel/drv -m 755 -u root -g sys ${BINDIR}/pctc
	cp /etc/system /etc/system.nopctc
	echo "exclude: pctc" >> /etc/system.nopctc
dist:
	-mkdir	${DISTDIR}
	-mkdir	${DISTDIR}/i386
	-mkdir	${DISTDIR}/sparc
	/usr/sbin/install -f ${DISTDIR} -m 444 COPYING
	/usr/sbin/install -f ${DISTDIR} -m 444 README.txt
	/usr/sbin/install -f ${DISTDIR} -m 444 README.japanese-euc
	/usr/sbin/install -f ${DISTDIR} -m 444 Makefile
	/usr/sbin/install -f ${DISTDIR} -m 444 Makefile.sparc
	/usr/sbin/install -f ${DISTDIR} -m 444 pctcreg.h
	/usr/sbin/install -f ${DISTDIR} -m 444 pctc.c
	/usr/sbin/install -f ${DISTDIR} -m 755 addpctc.sh
	/usr/sbin/install -f ${DISTDIR} -m 755 addpcelx.sh
	/usr/sbin/install -f ${DISTDIR}/i386 -m 644 i386/pctc.o
	/usr/sbin/install -f ${DISTDIR}/i386 -m 755 i386/pctc

clean:
	${RM} ${BINDIR}/*
