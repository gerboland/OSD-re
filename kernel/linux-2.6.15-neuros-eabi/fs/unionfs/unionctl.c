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
 *  $Id: unionctl.c,v 1.40 2006/01/21 02:58:11 jsipek Exp $
 */

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>

#include "unionfs.h"

#define MAY_READ 4
#define MAY_WRITE 2

extern int find_union(const char *path, char **options, char **actual_path,
		      int uniononly);
static char **branches;
static int *branchperms;
static const char *progname;

void __attribute__ ((__noreturn__)) __usage(int line);

#define usage() __usage(__LINE__)

void __attribute__ ((__noreturn__)) __usage(int line)
{
#ifdef DEBUG
	fprintf(stderr, "Line: %d\n", line);
#endif
	fprintf(stderr,
		"unionctl version: $Id: unionctl.c,v 1.40 2006/01/21 02:58:11 jsipek Exp $\n");
	fprintf(stderr, "Distributed with Unionfs " UNIONFS_VERSION "\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "unionctl UNION ACTION [arguments]\n");
	fprintf(stderr,
		"ACTION can be --add, --remove, --mode, --list, or --query.\nFurther arguments depend on ACTION.\n");
	fprintf(stderr,
		"\tunionctl UNION --add [ --before BRANCH | --after BRANCH ] [ --mode (rw|ro|nfsro) ] DIRECTORY\n");
	fprintf(stderr, "\tunionctl UNION --remove BRANCH\n");
	fprintf(stderr, "\tunionctl UNION --mode BRANCH (rw|ro|nfsro)\n");
	fprintf(stderr, "\tunionctl UNION --list\n");
	fprintf(stderr, "\tunionctl FILENAME --query\n");
	fprintf(stderr,
		"The unionctl man page has a more detailed description of its usage.\n");
	exit(EXIT_FAILURE);
}

static inline int parse_rw(char *p)
{
	if (strcmp(p, "ro") == 0)
		return MAY_READ;
	else if (strcmp(p, "nfsro") == 0)
		return MAY_READ | MAY_NFSRO;
	else if (strcmp(p, "rw") == 0)
		return MAY_READ | MAY_WRITE;
	else
		return 0;
}

static char **parse_options(char *options)
{
	char **ret = NULL;
	int i = 0;

	char *p;
	char *q;
	char *r;
	char *s, *t, *u;

	p = options;
	do {
		q = strchr(p, ',');
		if (q) {
			*q++ = '\0';
		}
		if (!strncmp(p, "dirs=", strlen("dirs="))) {
			r = p + strlen("dirs=");
			do {
				s = strchr(r, ':');
				if (s) {
					*s++ = '\0';
				}

				i++;
				ret = realloc(ret, sizeof(char *) * (i + 1));
				if (!ret) {
					perror("realloc()");
					exit(EXIT_FAILURE);
				}
				branchperms =
				    realloc(branchperms, sizeof(int) * i);
				if (!branchperms) {
					perror("realloc()");
					exit(EXIT_FAILURE);
				}

				t = strchr(r, '=');
				u = t + 1;
				if (!t || !u || !*u)
					goto err;
				*t = 0;
				branchperms[i - 1] = parse_rw(u);
				if (!branchperms[i - 1]) {
				      err:
					fprintf(stderr, "cannot parse '%s'\n",
						r);
					exit(EXIT_FAILURE);
				}
				ret[i - 1] = strdup(r);
				ret[i] = NULL;

				r = s;
			}
			while (r);
		}
		p = q;
	}
	while (p);

	branches = ret;
	return ret;
}

static int get_branch(char *path)
{
	char *end;
	int ret;

	ret = strtol(path, &end, 0);
	if (!*end) {
		return ret;
	}

	ret = strlen(path);
	if ((ret > 1) && (path[ret - 1] == '/')) {
		path[ret - 1] = '\0';
	}

	ret = 0;
	while (branches[ret]) {
		if (!strcmp(branches[ret], path)) {
			return ret;
		}
		ret++;
	}

	return -1;
}

static void dump_branches(const char *prefix)
{
	int i = 0;

	while (branches[i]) {
		char r, w, n;
		r = (branchperms[i] & MAY_READ) ? 'r' : '-';
		w = (branchperms[i] & MAY_WRITE) ? 'w' : '-';
		n = (branchperms[i] & MAY_NFSRO) ? 'n' : '-';
		printf("%s%s (%c%c%c)\n", prefix, branches[i], r, w, n);
		i++;
	}
}

#define ADD 1
#define REMOVE 2
#define MODE 3
#define LIST 4
#define	QUERY 5

int main(int argc, char *argv[])
{
	struct unionfs_addbranch_args addargs;
	struct unionfs_rdwrbranch_args rdwrargs;
	unsigned long remountdata[3];
	fd_set branchlist;
	struct stat st;

	int fd = -1;
	int ret, i;

	char *path, resolv_path[PATH_MAX], resolv_bp[PATH_MAX];
	char *options = NULL, *actual_path = NULL;
	int action;

	char *branchpath;
	int branchnum;
	int unionpos = 1;
	int modepos = 2;

	progname = argv[0];

	/* check that minimum number of args were specified */
	if (argc < 3)
		usage();

	if (argv[1][0] == '-' && argv[1][1] == '-') {
		modepos = 1;
		unionpos = 2;
	} else {
		modepos = 2;
		unionpos = 1;
	}

	if (realpath(argv[unionpos], resolv_path) == NULL) {
		perror("realpath()");
		exit(EXIT_FAILURE);
	}
	path = resolv_path;
	if (strcmp(path, "/") && (path[strlen(path) - 1] == '/')) {
		path[strlen(path) - 1] = '\0';
	}

	if (!strcmp(argv[modepos], "--add")) {
		action = ADD;
	} else if (!strcmp(argv[modepos], "--remove")) {
		action = REMOVE;
	} else if (!strcmp(argv[modepos], "--mode")) {
		action = MODE;
	} else if (!strcmp(argv[modepos], "--list")) {
		action = LIST;
	} else if (!strcmp(argv[modepos], "--query")) {
		action = QUERY;
	} else {
		usage();
	}

	if (stat(path, &st) == -1) {
		fprintf(stderr, "stat(%s): %s\n", path, strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (find_union(path, &options, &actual_path, 1) < 0) {
		fprintf(stderr, "%s is not a valid union.\n", path);
		exit(EXIT_FAILURE);
	}
	branches = parse_options(options);
	if (!branches) {
		fprintf(stderr, "Could not parse options from /proc/mounts!\n");
		exit(EXIT_FAILURE);
	}

	/* open file on which ioctl will operate (that is actually any file in the union) */
	if (action != REMOVE) {
		fd = open(path, O_RDONLY);
		if (fd < 0) {
			fprintf(stderr, "open(%s): %s\n", path,
				strerror(errno));
			exit(EXIT_FAILURE);
		}
	}

	/* Parse the action's options into something usable, and do it. */
	switch (action) {
	case ADD:
		if (argc < 4)
			usage();

		/* Default values if the user leaves them unspecified. */
		branchnum = -1;
		addargs.ab_perms = MAY_READ | MAY_WRITE;
		branchpath = NULL;
		for (i = 3; i < argc; i++) {
			if (argv[i][0] == '-' && argv[i][1] == '-') {
				if (!strcmp(argv[i], "--before")) {
					i++;
					if (i == argc) {
						fprintf(stderr,
							"%s requires an argument!\n",
							argv[i - 1]);
						usage();
					}

					branchnum = get_branch(argv[i]);
					if (branchnum == -1) {
						fprintf(stderr,
							"%s is not a valid branch.\nThe current branch configuration is:\n",
							argv[i]);
						dump_branches("\t");
					}
				} else if (!strcmp(argv[i], "--after")) {
					i++;
					if (i == argc) {
						fprintf(stderr,
							"%s requires an argument!\n",
							argv[i - 1]);
						usage();
					}

					branchnum = get_branch(argv[i]);
					if (branchnum == -1) {
						fprintf(stderr,
							"%s is not a valid branch.\nThe current branch configuration is:\n",
							argv[i]);
						dump_branches("\t");
					}
					branchnum++;
				} else if (!strcmp(argv[i], "--mode")) {
					i++;
					if (i == argc) {
						fprintf(stderr,
							"%s requires an argument!\n",
							argv[i - 1]);
						usage();
					}

					addargs.ab_perms = parse_rw(argv[i]);
					if (!addargs.ab_perms) {
						fprintf(stderr,
							"Valid modes are ro, nfsro and rw you specified: \"%s\"\n",
							argv[i]);
						usage();
					}
				} else {
					fprintf(stderr, "Unknown option: %s\n",
						argv[i]);
					usage();
				}
			} else {
				int branchchk = -1;
				/* The options must go before the path */
				if ((i + 1) != argc) {
					fprintf(stderr,
						"The path of the branch to add must go after all options.\n");
					usage();
				}
				branchchk = get_branch(argv[i]);
				if (branchchk != -1) {
					fprintf(stderr,
						"%s already exists in the Union.\n",
						argv[i]);
					usage();
				}
				if (branchnum == -1)
					branchnum = 0;
				branchpath = argv[i];
			}
		}
		if (!branchpath) {
			fprintf(stderr,
				"You must specify the path to add into the union!\n");
			usage();
		}

		if (realpath(branchpath, resolv_bp) == NULL) {
			perror("realpath()");
			exit(EXIT_FAILURE);
		}

		addargs.ab_branch = branchnum;
		addargs.ab_path = resolv_bp;

		errno = 0;
		ret = ioctl(fd, UNIONFS_IOCTL_ADDBRANCH, &addargs);
		if (ret < 0) {
			switch (errno) {
			case E2BIG:
				fprintf(stderr,
					"Unionfs supports only %d branches.\n",
					FD_SETSIZE);
				break;
			}
			fprintf(stderr, "Failed to add %s into %s: %s",
				branchpath, path, strerror(errno));
			exit(EXIT_FAILURE);
		}
		break;
	case MODE:
		if (argc != 5) {
			usage();
		}

		branchnum = 3;
		rdwrargs.rwb_perms = parse_rw(argv[4]);
		if (!rdwrargs.rwb_perms) {
			branchnum = 4;
			rdwrargs.rwb_perms = parse_rw(argv[3]);
			if (!rdwrargs.rwb_perms) {
				usage();
				exit(EXIT_FAILURE);
			}
		}

		if (realpath(argv[branchnum], resolv_bp) == NULL) {
			perror("realpath()");
			exit(EXIT_FAILURE);
		}
		branchpath = resolv_bp;

		/* Set a branches writeable status. */
		rdwrargs.rwb_branch = get_branch(branchpath);
		if (rdwrargs.rwb_branch == -1) {
			fprintf(stderr,
				"%s is not a valid branch.\nThe current branch configuration is:\n",
				branchpath);
			dump_branches("\t");
			exit(EXIT_FAILURE);
		}

		ret = ioctl(fd, UNIONFS_IOCTL_RDWRBRANCH, &rdwrargs);
		if (ret < 0) {
			fprintf(stderr,
				"Failed to set permissions for %s in %s: %s",
				branchpath, path, strerror(errno));
			exit(EXIT_FAILURE);
		}

		goto out;
		break;
	case REMOVE:
		if (argc != 4)
			usage();

		if (realpath(argv[3], resolv_bp) == NULL) {
			perror("realpath()");
			exit(EXIT_FAILURE);
		}
		branchpath = resolv_bp;

		branchnum = get_branch(branchpath);
		if (branchnum == -1) {
			fprintf(stderr,
				"%s is not a valid branch.\nThe current branch configuration is:\n",
				branchpath);
			dump_branches("\t");
			exit(EXIT_FAILURE);
		}

		errno = 0;
		remountdata[0] = UNIONFS_REMOUNT_MAGIC;
		remountdata[1] = UNIONFS_IOCTL_DELBRANCH;
		remountdata[2] = branchnum;
		ret =
		    mount("unionfs", actual_path, "unionfs",
			  MS_REMOUNT | MS_MGC_VAL, remountdata);
		if (ret < 0) {
			fprintf(stderr, "Failed to remove %s from %s: %s",
				branchpath, path, strerror(errno));
			exit(EXIT_FAILURE);
		}
		break;
	case LIST:
		dump_branches("\t");
		break;

	case QUERY:
		if (argc != 3) {
			usage();
		}

		if ((fd = open(argv[unionpos], O_RDONLY)) < 0) {
			fprintf(stderr,
				"Unable to open file %s : %s",
				argv[unionpos], strerror(errno));
			exit(EXIT_FAILURE);
		}

		ret = ioctl(fd, UNIONFS_IOCTL_QUERYFILE, &branchlist);
		if (ret < 0) {
			fprintf(stderr,
				"Unable to retrieve list of branches for file %s : %s\n",
				argv[unionpos], strerror(errno));
			exit(EXIT_FAILURE);
		}

		for (i = 0; i <= ret; i++) {
			char r, w, n;
			r = (branchperms[i] & MAY_READ) ? 'r' : '-';
			w = (branchperms[i] & MAY_WRITE) ? 'w' : '-';
			n = (branchperms[i] & MAY_NFSRO) ? 'n' : '-';

			if (FD_ISSET(i, &branchlist))
				printf("%s\t%s (%c%c%c)\n", argv[unionpos],
				       branches[i], r, w, n);
		}
		break;
	}

      out:
	if (fd != -1)
		close(fd);
	exit(EXIT_SUCCESS);
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
