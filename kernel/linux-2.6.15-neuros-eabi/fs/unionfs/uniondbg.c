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
 *  $Id: uniondbg.c,v 1.16 2006/01/13 03:00:24 jsipek Exp $
 */

#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>

#include "unionfs.h"

#define MAY_READ 4
#define MAY_WRITE 2

static int opt_d = 0;
static int opt_c = 0;
static int opt_g = 0;
static int optcount;
static const char *progname;

void usage(void)
{
	fprintf(stderr, "Usage:\n"
		"%s -d file [val]\n\tto set/get debugging values\n"
		"%s -c file\n\tto print out branch reference counts (in kernel debug output)\n"
		"%s -g file\n\tto increment the super block generation count\n"
		"%s -s file\n\tto duplicate the super block\n", progname,
		progname, progname, progname);
	exit(1);
}

int main(int argc, char *argv[])
{
	int fd, ret, val = 0;
	int i;

	progname = argv[0];

	/* check that minimum number of args were specified */
	if (argc < 3)
		usage();

	if (strcmp(argv[1], "-d") == 0) {
		if (argc > 4)
			usage();
		opt_d++;
		optcount++;
	}
	if (strcmp(argv[1], "-c") == 0) {
		if (argc > 3)
			usage();
		opt_c++;
		optcount++;
	}
	if (strcmp(argv[1], "-g") == 0) {
		if (argc > 3)
			usage();
		opt_g++;
		optcount++;
	}

	/* check that at least one option was used */
	if (!optcount)
		usage();

	/* open file on which ioctl will operate */
	fd = open(argv[2], O_RDONLY);
	if (fd < 0) {
		perror(argv[2]);
		exit(1);
	}

	/* if specified 3rd arg, want to set debug level */
	if (opt_d) {
		if (argc == 4) {
			val = atoi(argv[3]);
			ret = ioctl(fd, FIST_IOCTL_SET_DEBUG_VALUE, &val);
			if (ret < 0) {
				perror("ioctl set");
				exit(1);
			}
		} else {
			ret = ioctl(fd, FIST_IOCTL_GET_DEBUG_VALUE, &val);
			if (ret < 0) {
				perror("ioctl get");
				exit(1);
			}
			printf("debug ioctl returned value %d\n", val);
		}
		goto out;
	}

	/* branch refcounts */
	if (opt_c) {
		int *counts;
		ret = ioctl(fd, UNIONFS_IOCTL_BRANCH_COUNT, NULL);
		if (ret < 0) {
			perror("ioctl branchcount (a)");
			exit(1);
		}
		counts = malloc(ret * sizeof(int));
		if (!counts) {
			perror("malloc");
			exit(1);
		}
		ret = ioctl(fd, UNIONFS_IOCTL_BRANCH_COUNT, counts);
		if (ret < 0) {
			perror("ioctl branchcount (b)");
			exit(1);
		}
		printf("%d total branches.\n", ret);
		for (i = 0; i < ret; i++) {
			printf("%d: %d\n", i, counts[i]);
		}
		free(counts);
		goto out;
	}

	/* Update generation number. */
	if (opt_g) {
		ret = ioctl(fd, UNIONFS_IOCTL_INCGEN, NULL);
		if (ret < 0) {
			perror("ioctl incgen");
			exit(1);
		}
		printf("New generation %d\n", ret);
		goto out;
	}

      out:
	close(fd);
	exit(0);
}

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
