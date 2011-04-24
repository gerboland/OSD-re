/*
 * Copyright (c) 2003-2006 Erez Zadok
 * Copyright (c) 2003-2006 Charles P. Wright
 * Copyright (c) 2005-2006 Josef Sipek
 * Copyright (c) 2005      Arun M. Krishnakumar
 * Copyright (c) 2005-2006 David P. Quigley
 * Copyright (c) 2003-2004 Mohammad Nayyer Zubair
 * Copyright (c) 2003      Puja Gupta
 * Copyright (c) 2003      Harikesavan Krishnan
 * Copyright (c) 2003-2006 Stony Brook University
 * Copyright (c) 2003-2006 The Research Foundation of State University of New York
 *
 * For specific licensing information, see the COPYING file distributed with
 * this package.
 *
 * This Copyright notice must be kept intact and distributed with all sources.
 */
/*
 *  $Id: unionimap.h,v 1.11 2006/01/03 01:15:56 jsipek Exp $
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/statfs.h>
#include <sys/stat.h>
#include <uuid/uuid.h>
#include <string.h>
/*
 * Defines,structs,and functions for persistent inodes used by kernel and user
 */
#define MAX_MAPS 256
#define UUID_LEN 16
#define FORWARDMAP_MAGIC 0x4b1cb38f
#define REVERSEMAP_MAGIC 0Xfcafad71
#define FORWARDMAP_VERSION 0x02
#define REVERSEMAP_VERSION 0x01

typedef u_int8_t uint8_t;
typedef u_int32_t uint32_t;
typedef u_int64_t uint64_t;

struct fmaphdr {
	uint32_t magic;
	uint32_t version;
	uint8_t usedbranches;
	uint8_t uuid[UUID_LEN];
};
struct bmapent {
	fsid_t fsid;
	uint8_t uuid[UUID_LEN];
};

struct rmaphdr {
	uint32_t magic;
	uint32_t version;
	uint8_t fwduuid[UUID_LEN];
	uint8_t revuuid[UUID_LEN];
	fsid_t fsid;
};

struct bpair {
	fsid_t fsid;
	uint8_t fsnum;
};

struct fmapent {
	uint8_t fsnum;
	uint64_t inode;
};

extern int mkfsid(char *path, fsid_t * fsid);
/*
 *
 * vim:shiftwidth=8
 * vim:tabstop=8
 *
 * For Emacs:
 * Local variables:
 * c-basic-offset: 8
 * c-comment-only-line-offset: 0
 * c-offsets-alist: ((statement-block-intro . +) (knr-argdecl-intro . 0)
 *              (substatement-open . 0) (label . 0) (statement-cont . +))
 * indent-tabs-mode: t
 * tab-width: 8
 * End:
 */
