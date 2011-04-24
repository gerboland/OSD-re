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
 *  $Id: usercommon.c,v 1.8 2006/01/03 01:15:56 jsipek Exp $
 */
/*
 * Copyright (c) 2003-2005 Erez Zadok
 * Copyright (c) 2003-2005 Charles P. Wright
 * Copyright (c) 2003-2005 Mohammad Nayyer Zubair
 * Copyright (c) 2003-2005 Puja Gupta
 * Copyright (c) 2003-2005 Harikesavan Krishnan
 * Copyright (c) 2003-2005 Stony Brook University
 * Copyright (c) 2003-2005 The Research Foundation of State University of New York
 *
 * For specific licensing information, see the COPYING file distributed with
 * this package.
 *
 * This Copyright notice must be kept intact and distributed with all sources.
 */

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/sysmacros.h>
/*
 * This function will take a patch and check it against /proc/mounts to
 * find its mount point. If uniononly is set  then it will make sure its
 * a unionf  mount point. This function assumes the both options and actual_path
 * are valid and not null;
 */
int find_union(const char *path, char **options, char **actual_path,
	       int uniononly)
{
	FILE *f = NULL;
	char *s = NULL;
	char *s2 = NULL;
	char *p;
	char *q;
	int candidate = 0;
	int mallocsize = 1024;	/* Just a reasonable starting value. */

      retry:
	if (*options) {
		free(*options);
		*options = NULL;
	}

	if (*actual_path) {
		free(*actual_path);
		*actual_path = NULL;
	}
	if (f) {
		fclose(f);
		f = NULL;
	}
	s2 = realloc(s, mallocsize);
	if (!s2) {
		fprintf(stderr, "realloc(%d): %s\n", mallocsize,
			strerror(errno));
		goto out;
	}
	s = s2;

	f = fopen("/proc/mounts", "r");
	if (!f) {
		fprintf(stderr, "fopen(/proc/mounts): %s\n", strerror(errno));
		goto out;
	}
	while (fgets(s, mallocsize, f)) {
		int testcan;

		/* If we don't have enough information, we should remalloc it. */
		if (strlen(s) == (mallocsize - 1)) {
			mallocsize *= 2;
			goto retry;
		}

		p = strchr(s, ' ');
		if (!p)
			continue;
		p++;

		q = strchr(p, ' ');
		if (!q)
			continue;
		*q++ = '\0';

		testcan = strlen(p);
		if (testcan <= candidate) {
			continue;
		}

		if (!strncmp(path, p, testcan)) {
			if (*actual_path) {
				free(*actual_path);
			}
			*actual_path = strdup(p);
			if (!*actual_path) {
				fprintf(stderr, "strdup: %s\n",
					strerror(errno));
				goto out;
			}
			p = strchr(q, ' ');
			if (!p)
				continue;
			*p++ = '\0';
			if (uniononly) {
				if (strcmp(q, "unionfs")) {
					candidate = 0;
					continue;
				}
			}
			candidate = testcan;

			q = strrchr(p, ' ');
			if (!q)
				continue;
			*q = '\0';
			q = strrchr(p, ' ');
			if (!q)
				continue;
			*q = '\0';

			if (*options) {
				free(*options);
			}
			*options = strdup(p);
			if (!*options) {
				fprintf(stderr, "strdup: %s\n",
					strerror(errno));
				goto out;
			}
		}
	}

      out:
	if (s)
		free(s);
	if (f)
		fclose(f);

	if (*options) {
		return 0;
	}

	errno = -ENOENT;
	return -1;
}

/**
 * Takes the device and creates an fsid from it by placing major in the first
 * int and minor in the second.
 */
void fillfsid(dev_t dev, fsid_t * fsid)
{
	((unsigned int *)fsid)[0] = major(dev);
	((unsigned int *)fsid)[1] = minor(dev);
}

int mkfsid(char *path, fsid_t * fsid)
{
	int err = 0;
	char *actual_path = NULL;
	char *options = NULL;
	struct stat stat_struct;

	memset(&stat_struct, 0, sizeof(struct stat));

	err = find_union(path, &options, &actual_path, 0);
	if (err) {
		printf("find_union failed:\n");
		goto out;
	}
	err = stat(actual_path, &stat_struct);
	if (err) {
		perror("Couldent stat path: ");
		goto out;
	}

	fillfsid(stat_struct.st_dev, fsid);

      out:
	return err;
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
