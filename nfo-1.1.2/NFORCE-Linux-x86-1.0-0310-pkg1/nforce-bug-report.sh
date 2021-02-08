#!/bin/sh

# NVIDIA NFORCE driver bug reporting shell script.  This shell
# script will generate a log file named "nforce-bug-report.log", which
# should be attached when emailing bug reports to NVIDIA.

LOG_FILENAME=nforce-bug-report.log

PATH="/sbin:/usr/sbin:$PATH"

#
# append() - append the contents of the specified file to the log
#

append() {
    echo "____________________________________________" >> $LOG_FILENAME
    echo ""                                             >> $LOG_FILENAME

    if [ ! -f "$1" ]; then
        echo "$1 does not exist"                        >> $LOG_FILENAME
    elif [ ! -r "$1" ]; then
        echo "$1 is not readable"                       >> $LOG_FILENAME
    else
        echo "$1"                                       >> $LOG_FILENAME
        cat  "$1"                                       >> $LOG_FILENAME
    fi
    echo ""                                             >> $LOG_FILENAME
}

#
# append_glob() - use the shell to expand a list of files, and invoke
# append() for each of them
#

append_glob() {
    for i in `ls $1 2> /dev/null;`; do
        append "$i"
    done
}

#
# Start of script
#


# check that we are root (needed for `lspci -vxxx` and potentially for
# accessing kernel log files)

if [ $UID -ne 0 ]; then
    echo "ERROR: Please run $(basename $0) as root."
    exit 1
fi


# move any old log file out of the way

if [ -f $LOG_FILENAME ]; then
    mv $LOG_FILENAME ${LOG_FILENAME}.old
fi


# make sure what we can write to the log file

touch $LOG_FILENAME 2> /dev/null

if [ "$?" -ne "0" ]; then
    echo
    echo "ERROR: Working directory is not writable; please cd to a directory"
    echo "       where you have write permission so that the $LOG_FILENAME"
    echo "       file can be written."
    echo
    exit 1
fi


# print a start message to stdout

echo ""
echo -n "Running $(basename $0)...";


# print prologue to the log file

echo "____________________________________________"          >> $LOG_FILENAME
echo ""                                                      >> $LOG_FILENAME
echo "Start of NVIDIA bug report log file.  Please send this report,"  >> $LOG_FILENAME
echo "along with a description of your bug, to linux-nforce-bugs@nvidia.com." >> $LOG_FILENAME  
echo ""                                                      >> $LOG_FILENAME
echo "Date: `date`"                                          >> $LOG_FILENAME
echo "uname: `uname -a`"                                     >> $LOG_FILENAME
echo ""                                                      >> $LOG_FILENAME


# append useful files

#append "/proc/driver/nvidia/version"
#append_glob "/proc/driver/nvidia/cards/*"
#append_glob "/proc/driver/nvidia/agp/*"

append "/proc/cmdline"
append "/proc/cpuinfo"
append "/proc/interrupts"
append "/proc/meminfo"
append "/proc/modules"
append "/proc/version"
append "/proc/pci"
append "/proc/iomem"
append "/proc/mtrr"

# Append module configuration files. Look for files of the form modules* and
# modprobe* in /etc ...

i=`ls -d /etc/modules* /etc/modprobe* 2>&1`

# ... and any files in /etc/modprobe.d, if it exists...

if [ -d /etc/modprobe.d ]; then
    i="$i `ls /etc/modprobe.d/*`"
fi

# Write the found files to the log

for j in $i; do
    if [ -f "$j" ]; then
        append $j
    fi
done

#if [ -f /etc/modules.conf ]; then
#append "/etc/modules.conf"
#fi

#if [ -f /etc/modprobe.conf ]; then
#append "/etc/modprobe.conf"
#fi


# installer log

append "/var/log/nvidia-nforce-installer.log"

# lspci information

lspci=`which lspci 2> /dev/null`

if [ "$?" -eq 0 -a "$lspci" ]; then
    
    # print out all the NVIDIA devices
    for i in `$lspci -n | grep 10de | awk '{print $1}'`; do
        dev=`echo "$i" | cut -d " " -f 1`
        echo "____________________________________________"  >> $LOG_FILENAME
        echo ""                                              >> $LOG_FILENAME
        $lspci -s "$dev" -vxxx                               >> $LOG_FILENAME
    done
else
    echo "Skipping lspci output (lspci not found)"           >> $LOG_FILENAME
fi


# ifconfig -a 

    echo "____________________________________________"      >> $LOG_FILENAME
    echo ""                                                  >> $LOG_FILENAME
    echo "ifconfig -a"                                       >> $LOG_FILENAME
    echo ""                                                  >> $LOG_FILENAME

    ifconfig -a                                              >> $LOG_FILENAME

# get any relevant kernel messages

if [ -f /var/log/messages ]; then
    echo "____________________________________________"      >> $LOG_FILENAME
    echo ""                                                  >> $LOG_FILENAME
    echo "Scanning kernel log file for nvsound messages:"    >> $LOG_FILENAME
    echo ""                                                  >> $LOG_FILENAME
    cat /var/log/messages | grep Nvsound                     >> $LOG_FILENAME
    echo ""                                                  >> $LOG_FILENAME
    echo "____________________________________________"      >> $LOG_FILENAME
    echo ""                                                  >> $LOG_FILENAME
    echo "Scanning kernel log file for nvnet messages:"      >> $LOG_FILENAME
    echo ""                                                  >> $LOG_FILENAME
    cat /var/log/messages | grep nvnet                       >> $LOG_FILENAME
else
    echo "Skipping kernel log file messages"                 >> $LOG_FILENAME
fi

echo "____________________________________________"      >> $LOG_FILENAME
echo ""                                                  >> $LOG_FILENAME
echo "Saving dmesg:"                                     >> $LOG_FILENAME
echo ""                                                  >> $LOG_FILENAME
dmesg                                                    >> $LOG_FILENAME

# print epilogue to log file

echo "____________________________________________"          >> $LOG_FILENAME
echo ""                                                      >> $LOG_FILENAME
echo "End of NFORCE bug report log file."                    >> $LOG_FILENAME


# Done

echo " complete."
echo ""
echo "The file $LOG_FILENAME has been created; please send this report,"
echo "along with a description of your bug, to linux-nforce-bugs@nvidia.com."
echo ""
