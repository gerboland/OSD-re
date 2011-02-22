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
 *  $Id: unionimap.c,v 1.25 2006/02/09 04:41:00 jsipek Exp $
 */

#include "unionimap.h"

/**
 *	print_usage()
 *	Function used to print out usage information
 *	when an error is encountered
 */
void print_usage(void)
{
	fprintf(stderr,
		"unionimap version: $Id: unionimap.c,v 1.25 2006/02/09 04:41:00 jsipek Exp $\n");
	fprintf(stderr, "Distributed with Unionfs " UNIONFS_VERSION "\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: unionimap [-c] [-a ARG:] filename path\n");
	fprintf(stderr, " flag -c: create forward map with name filename\n");
	fprintf(stderr,
		" flag -a ARG: create reverse map with name filename and path and add it to forwardmap ARG\n");
	fprintf(stderr,
		" flag -d: if set once will dump the header of the map if twice it will also dump the contents\n");
	fprintf(stderr, " flag -h: prints this message then quits\n");
	fprintf(stderr,
		" please note that -c -a and -d are mutually exclusive\n");
}

/**
 * create_forwardmap(char *filename)
 * verifies that the forward map is valid.
 *
 */
int create_forwardmap(char *filename)
{
	int byteswritten = 0, err = 0;
	int fwrdmap = 0;
	struct fmaphdr header;
	void *table = NULL;
	uuid_t uuid;

	fwrdmap = creat(filename, S_IRUSR | S_IWUSR);
	if (fwrdmap < 0) {
		err = -EINVAL;
		goto out;
	}
	memset(&header, 0, sizeof(struct fmaphdr));
	header.version = FORWARDMAP_VERSION;
	header.magic = FORWARDMAP_MAGIC;
	header.usedbranches = 0;
	uuid_generate(uuid);
	memcpy(&header.uuid, &uuid, UUID_LEN);
	byteswritten = write(fwrdmap, (void *)&header, sizeof(struct fmaphdr));
	if (byteswritten < sizeof(struct fmaphdr)) {
		perror("Failed Writting header: ");
		err = -EIO;
		goto out;
	}
	table = malloc(sizeof(struct bmapent[256]));
	if (!table) {
		err = -ENOMEM;
		goto out;
	}
	memset(table, 0, sizeof(struct bmapent[256]));
	byteswritten = write(fwrdmap, table, sizeof(struct bmapent[256]));
	if (byteswritten < sizeof(struct bmapent[256])) {
		perror("Failed writting table: ");
		err = -EIO;
		goto out;
	}
      out:
	if (fwrdmap)
		close(fwrdmap);
	if (table)
		free(table);

	return err;
}
int open_forwardmap(struct fmaphdr *header, char *forwardmap)
{
	int fwrdmap = 0, bytesread = 0, err = 0;
	fwrdmap = open(forwardmap, O_RDWR);
	if (fwrdmap < 0) {
		perror("Open on Forwardmap  Failed: ");
		err = errno;
		goto out_error;
	}
	bytesread = read(fwrdmap, (void *)header, sizeof(struct fmaphdr));
	if (bytesread < 0 || bytesread < sizeof(struct fmaphdr)) {
		err = -EINVAL;
		goto out_error;
	}
	if (header->magic != FORWARDMAP_MAGIC
	    || header->version != FORWARDMAP_VERSION) {
		err = -EINVAL;
		goto out_error;
	}
	if (header->usedbranches == 255) {
		fprintf(stderr,
			"Forwardmap already contains maximum number of reverse maps");
		err = -EINVAL;
		goto out_error;
	}
	err = fwrdmap;
	goto out;
      out_error:
	if (fwrdmap)
		close(fwrdmap);
      out:
	return err;
}
int check_if_entry_exists(int fwrdmap, struct statfs stat, char *path)
{
	int err = 0, bytesread = 0, i;
	struct fmaphdr header;
	struct bmapent btable[256];
	fsid_t fsid;

	if (fwrdmap < 0) {
		err = -EINVAL;
		goto out;
	}
	err = lseek(fwrdmap, 0L, SEEK_SET);
	if (err) {
		goto out;
	}
	bytesread = read(fwrdmap, (void *)&header, sizeof(struct fmaphdr));
	if (bytesread != sizeof(struct fmaphdr)) {
		err = -EIO;
		goto out;
	}
	bytesread = read(fwrdmap, (void *)&btable, sizeof(struct bmapent[256]));
	if (bytesread != sizeof(struct bmapent[256])) {
		err = -EIO;
		goto out;
	}
	if (((unsigned int *)&stat.f_fsid)[0]
	    || ((unsigned int *)&stat.f_fsid)[1]) {
		fsid = stat.f_fsid;
	} else {
		err = mkfsid(path, &fsid);
		if (err) {
			goto out;
		}
	}
	for (i = 0; i < header.usedbranches; i++) {
		if (!memcmp(&fsid, &btable[i].fsid, sizeof(fsid_t))) {
			err = 1;
			break;
		}
	}
      out:
	return err;
}
int create_reversemap(char *filename, char *path, char *forwardmap)
{
	int byteswritten = 0, err = 0, seekres = 0;
	off_t offset = 0;
	int fwrdmap = 0, revmap = 0;
	struct fmaphdr fwdheader;
	struct rmaphdr revheader;
	struct statfs stat;
	struct bmapent ent;
	uuid_t uuid;

	fwrdmap = open_forwardmap(&fwdheader, forwardmap);
	if (fwrdmap < 0) {
		err = fwrdmap;
		goto out;
	}
	memset(&stat, 0, sizeof(struct statfs));
	err = statfs(path, &stat);
	if (err) {
		perror("statfs failed on path: ");
		goto out;
	}
	err = check_if_entry_exists(fwrdmap, stat, path);
	if (err) {
		if (err > 0) {
			err = -EINVAL;
			fprintf(stderr,
				"Specified fs already exists in the forward map\n");
		}
		goto out;
	}
	revmap = open(filename, O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR);
	if (!revmap) {
		err = -EINVAL;
		perror("Open on Reversemap failed: ");
		goto out;
	}
	revheader.version = REVERSEMAP_VERSION;
	revheader.magic = REVERSEMAP_MAGIC;
	uuid_generate(uuid);
	memcpy(&revheader.revuuid, &uuid, UUID_LEN);
	memcpy(&revheader.fwduuid, &fwdheader.uuid, UUID_LEN);

	if (((unsigned int *)&stat.f_fsid)[0]
	    || ((unsigned int *)&stat.f_fsid)[1]) {
		revheader.fsid = stat.f_fsid;
	} else {
		err = mkfsid(path, &revheader.fsid);
		if (err) {
			goto out;
		}

	}
	byteswritten = write(revmap, &revheader, sizeof(struct rmaphdr));
	if (byteswritten < sizeof(struct rmaphdr)) {
		err = -EIO;
		perror("Reversemap Write failed: ");
		goto out;
	}

	offset =
	    sizeof(struct fmaphdr) +
	    (fwdheader.usedbranches * sizeof(struct bmapent));
	seekres = lseek(fwrdmap, offset, SEEK_SET);
	if (!(seekres == offset)) {
		err = -EIO;
		perror("Couldent seek to offset in uuid table: ");
	}
	errno = 0;
	memcpy(&ent.uuid, &uuid, UUID_LEN);
	memcpy(&ent.fsid, &revheader.fsid, sizeof(fsid_t));
	byteswritten = write(fwrdmap, &ent, sizeof(struct bmapent));
	if (byteswritten < sizeof(struct bmapent)) {
		perror("Forwardmap Write failed to write uuid to table: ");
		err = -EIO;
		goto out;
	}
	fwdheader.usedbranches++;
	offset = (int)&(((struct fmaphdr *)(0))->usedbranches);
	seekres = lseek(fwrdmap, offset, SEEK_SET);
	if (!(seekres == offset)) {
		perror("Couldent seek to usedbranch offset: ");
		err = -EIO;
		goto out;
	}
	byteswritten = write(fwrdmap, &fwdheader.usedbranches, sizeof(uint8_t));
	if (byteswritten < sizeof(uint8_t)) {
		perror("Forwardmap Write failed to update usedbranches: ");
		err = -EIO;
		goto out;
	}

      out:
	if (fwrdmap) {
		close(fwrdmap);
	}
	if (revmap) {
		close(revmap);
	}
	return err;
}
int print_forwardmap(int file, int debug_level)
{
	int err = 0, bytesread = 0, i;
	struct fmaphdr header;
	struct bmapent btable[256];
	char *uuid_unparsed;
	struct fmapent entry;

	if (file < 0 || debug_level <= 0) {
		err = -EINVAL;
		goto out;
	}
	err = lseek(file, 0L, SEEK_SET);
	if (err) {
		goto out;
	}
	bytesread = read(file, (void *)&header, sizeof(struct fmaphdr));
	if (bytesread != sizeof(struct fmaphdr)) {
		err = -EIO;
		goto out;
	}
	uuid_unparsed = (char *)malloc(37);
	if (!uuid_unparsed) {
		err = -EIO;
		goto out;
	}
	memset(uuid_unparsed, 0, 37);
	fprintf(stdout, "Unionfs Forwardmap:\n");
	fprintf(stdout, "Magic: %x\n", header.magic);
	fprintf(stdout, "Version: %d\n", header.version);
	fprintf(stdout, "Used Branches: %d\n", header.usedbranches);
	uuid_unparse(header.uuid, uuid_unparsed);
	fprintf(stdout, "UUID: %s\n", uuid_unparsed);
	bytesread = read(file, (void *)&btable, sizeof(struct bmapent[256]));
	if (bytesread != sizeof(struct bmapent[256])) {
		err = -EIO;
		goto out;
	}
	for (i = 0; i < header.usedbranches; i++) {
		fprintf(stdout, "Branch at index : %d\n", i);
		fprintf(stdout, "fsid: %04x%04x\n",
			((unsigned int *)&btable[i].fsid)[0],
			((unsigned int *)&btable[i].fsid)[1]);
		uuid_unparse(btable[i].uuid, uuid_unparsed);
		fprintf(stdout, "uuid: %s\n", uuid_unparsed);
	}
	if (debug_level > 1) {
		unsigned long inode = 1;
		fprintf(stdout, "%-11s %-8s %-22s\n", "Unionfs", "FS Num",
			"Lower-Level");
		while (read(file, (void *)&entry, sizeof(struct fmapent))) {
			fprintf(stdout, "%-11lu %-8d %-22llu\n", inode++,
				entry.fsnum,
				(long long unsigned int)entry.inode);
		}
	}
      out:
	return err;

}
int print_reversemap(int file, int debug_level)
{

	int err = 0, bytesread = 0;
	struct rmaphdr header;
	char *uuid_unparsed = NULL;
	uint64_t inode;

	if (file < 0 || debug_level <= 0) {
		err = -EINVAL;
		goto out;
	}
	err = lseek(file, 0L, SEEK_SET);
	if (err) {
		goto out;
	}
	bytesread = read(file, (void *)&header, sizeof(struct rmaphdr));
	if (bytesread != sizeof(struct rmaphdr)) {
		err = -EIO;
		goto out;
	}
	uuid_unparsed = (char *)malloc(37);
	if (!uuid_unparsed) {
		err = -ENOMEM;
		goto out;
	}
	memset(uuid_unparsed, 0, 37);
	fprintf(stdout, "Unionfs Reversemap:\n");
	fprintf(stdout, "Magic: %x\n", header.magic);
	fprintf(stdout, "Version: %d\n", header.version);
	uuid_unparse(header.fwduuid, uuid_unparsed);
	fprintf(stdout, "Forward UUID: %s\n", uuid_unparsed);
	uuid_unparse(header.revuuid, uuid_unparsed);
	fprintf(stdout, "Reverse UUID: %s\n", uuid_unparsed);
	fprintf(stdout, "fsid: %04x%04x\n", ((unsigned int *)&header.fsid)[0],
		((unsigned int *)&header.fsid)[1]);
	if (debug_level > 1) {
		fprintf(stdout, "%-11s %-22s\n", "Lower", "Unionfs");
		unsigned long lowerlevel = 0;
		while (read(file, (void *)&inode, sizeof(uint64_t))) {
			if (inode) {
				fprintf(stdout, "%-11lu %-22llu\n",
					lowerlevel++,
					(long long unsigned int)inode);
			}
		}
	}
      out:
	if (uuid_unparsed) {
		free(uuid_unparsed);
	}
	return err;
}
int dump_map(char *filename, int debug_level)
{
	int err = 0, bytesread = 0, file = 0;
	uint32_t magic;

	file = open(filename, O_RDONLY);
	if (file < 0) {
		err = -EINVAL;
		goto out;
	}
	bytesread = read(file, (void *)&magic, sizeof(uint32_t));
	if (bytesread < sizeof(uint32_t)) {
		err = -EINVAL;
		goto out;
	}
	if (magic == FORWARDMAP_MAGIC) {
		err = print_forwardmap(file, debug_level);
	} else if (magic == REVERSEMAP_MAGIC) {
		err = print_reversemap(file, debug_level);
	} else {
		err = -EINVAL;
	}
      out:
	if (file) {
		close(file);
	}

	return err;
}

int main(int argc, char **argv)
{
	int c = 0;		//Temp varliable to hold our character for the switch
	int cflag = 0, aflag = 0, dflag = 0;
	int err = 0;
	char *avalue = NULL;
	if (argc < 2) {
		print_usage();
		err = -EINVAL;
		goto out;
	}
	while ((c = getopt(argc, argv, "a:cd")) != -1) {
		switch (c) {
		case 'c':
			cflag = c;
			break;
		case 'a':
			aflag = 1;
			avalue = strdup(optarg);
			if (!avalue) {
				perror("strdup");
				exit(1);
			}
			break;
		case 'd':
			dflag++;
			break;
		case '?':
			if (isprint(optopt))
				fprintf(stderr, "Unknown option `-%c'.\n",
					optopt);
			else
				fprintf(stderr,
					"Unknown option character `\\x%x'.\n",
					optopt);
			print_usage();
			err = -EINVAL;
			goto out;
			break;
		default:
			print_usage();
			err = -EINVAL;
			goto out;
			break;
		}
	}
	if (optind == argc) {
		print_usage();
		err = -EINVAL;
		goto out;
	}
	if (!(cflag ^ aflag ^ dflag)) {
		print_usage();
		err = -EINVAL;
		goto out;
	}
	if (cflag) {
		err = create_forwardmap(argv[optind]);
		if (err) {
			unlink(argv[optind]);
		}
		goto out;
	} else if (aflag) {
		if (optind + 1 > argc) {
			print_usage();
			err = -EINVAL;
			goto out;
		}

		err = create_reversemap(argv[optind], argv[optind + 1], avalue);
		if (err) {
			unlink(argv[optind]);
			goto out;
		}

	} else if (dflag) {
		err = dump_map(argv[optind], dflag);
		if (err) {
			goto out;
		}
	}
      out:
	free(avalue);
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
