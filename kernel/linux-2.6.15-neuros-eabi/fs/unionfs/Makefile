UNIONFS_VERSION  = 1.1.5
EXTRA_CFLAGS+=-DUNIONFS_VERSION=\"${UNIONFS_VERSION}\"
# This will disable debugging support
# EXTRA_CFLAGS+=-DUNIONFS_NDEBUG

obj-$(CONFIG_UNION_FS) += unionfs.o

unionfs-objs := subr.o dentry.o file.o inode.o main.o super.o \
	stale_inode.o branchman.o xattr.o rdstate.o copyup.o  \
	dirhelper.o rename.o unlink.o lookup.o persistent_inode.o \
	commonfops.o dirfops.o print.o
