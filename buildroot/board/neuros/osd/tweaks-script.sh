#!/bin/bash

# This script is run just before the final rootfs is packaged. It is useful
# for applying last minute touches to the filesystem.

# Where is the target build directory that we want to work on?
TARGET=$(PWD)/output/target

# Internal storage on Neuros OSD, about 1MB in size. Do not write to this
# frequently, as it is running jffs2 on NOR flash and has limited write cycles
INT_STORAGE=$(TARGET)/var/mnt/internal


# Fix for dropbear wanting a writeable /etc/dropbear
if [ -f $(TARGET)/usr/sbin/dropbear ] then
	echo "Dropbear installed, customising for OSD:re"
	# Add lines to dropbear's init script to ensure dropbear directory 
	# created in $INT_STORAGE
#############Lines to Insert#######################
	INSERT='\
#Read-only filesystem workaround for OSD:re \
if [ ! -d /var/mnt/internal /dropbear ] then \
        mkdir -p /var/mnt/internal/dropbear \
fi \
ln -s /var/mnt/internal/dropbear /etc/dropbear'
####################################################
	sed "8i\
\
${INSERT}" < $(TARGET)/etc/init.d/S50dropbear \
		> $(TARGET)/etc/init.d/S50dropbear.tmp
	rm $(TARGET)/etc/init.d/S50dropbear
	mv $(TARGET)/etc/init.d/S50dropbear{.tmp,}
	chmod 755 $(TARGET)/etc/init.d/S50dropbear
fi
