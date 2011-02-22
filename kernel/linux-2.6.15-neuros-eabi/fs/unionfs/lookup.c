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
 *  $Id: lookup.c,v 1.43 2006/02/13 00:18:40 jsipek Exp $
 */

#include "unionfs.h"

static int is_opaque_dir(struct dentry *dentry, int bindex);
static int is_validname(const char *name);

struct dentry *unionfs_lookup_backend(struct dentry *dentry, int lookupmode)
{
	int err = 0;
	struct dentry *hidden_dentry = NULL;
	struct dentry *wh_hidden_dentry = NULL;
	struct dentry *hidden_dir_dentry = NULL;
	struct dentry *parent_dentry = NULL;
	int bindex, bstart, bend, bopaque;
	int dentry_count = 0;	/* Number of positive dentries. */
	int first_dentry_offset = -1;
	struct dentry *first_hidden_dentry = NULL;
	int locked_parent = 0;
	int locked_child = 0;

	int opaque;
	char *whname = NULL;
	const char *name;
	int namelen;

	print_entry("mode = %d", lookupmode);

	/* We should already have a lock on this dentry in the case of a
	 * partial lookup, or a revalidation. Otherwise it is returned from
	 * new_dentry_private_data already locked.  */
	if (lookupmode == INTERPOSE_PARTIAL || lookupmode == INTERPOSE_REVAL
	    || lookupmode == INTERPOSE_REVAL_NEG) {
		verify_locked(dentry);
	} else {
		BUG_ON(dtopd_nocheck(dentry) != NULL);
		locked_child = 1;
	}
	if (lookupmode != INTERPOSE_PARTIAL)
		if ((err = new_dentry_private_data(dentry)))
			goto out;
	/* must initialize dentry operations */
	dentry->d_op = &unionfs_dops;

	parent_dentry = GET_PARENT(dentry);
	/* We never partial lookup the root directory. */
	if (parent_dentry != dentry) {
		lock_dentry(parent_dentry);
		locked_parent = 1;
	} else {
		DPUT(parent_dentry);
		parent_dentry = NULL;
		goto out;
	}

	fist_print_dentry("IN unionfs_lookup (parent)", parent_dentry);
	fist_print_dentry("IN unionfs_lookup (child)", dentry);

	name = dentry->d_name.name;
	namelen = dentry->d_name.len;

	/* No dentries should get created for possible whiteout names. */
	if (!is_validname(name)) {
		err = -EPERM;
		goto out_free;
	}

	/* Now start the actual lookup procedure. */
	bstart = dbstart(parent_dentry);
	bend = dbend(parent_dentry);
	bopaque = dbopaque(parent_dentry);
	BUG_ON(bstart < 0);

	/* It would be ideal if we could convert partial lookups to only have
	 * to do this work when they really need to.  It could probably improve
	 * performance quite a bit, and maybe simplify the rest of the code. */
	if (lookupmode == INTERPOSE_PARTIAL) {
		bstart++;
		if ((bopaque != -1) && (bopaque < bend))
			bend = bopaque;
	}

	fist_dprint(8, "bstart = %d, bend = %d\n", bstart, bend);
	for (bindex = bstart; bindex <= bend; bindex++) {
		hidden_dentry = dtohd_index(dentry, bindex);
		if (lookupmode == INTERPOSE_PARTIAL && hidden_dentry)
			continue;
		BUG_ON(hidden_dentry != NULL);

		hidden_dir_dentry = dtohd_index(parent_dentry, bindex);

		/* if the parent hidden dentry does not exist skip this */
		if (!(hidden_dir_dentry && hidden_dir_dentry->d_inode))
			continue;

		/* also skip it if the parent isn't a directory. */
		if (!S_ISDIR(hidden_dir_dentry->d_inode->i_mode))
			continue;

		/* Reuse the whiteout name because its value doesn't change. */
		if (!whname) {
			whname = alloc_whname(name, namelen);
			if (IS_ERR(whname)) {
				err = PTR_ERR(whname);
				goto out_free;
			}
		}

		/* check if whiteout exists in this branch: lookup .wh.foo */
		wh_hidden_dentry = LOOKUP_ONE_LEN(whname, hidden_dir_dentry,
						  namelen + WHLEN);
		if (IS_ERR(wh_hidden_dentry)) {
			DPUT(first_hidden_dentry);
			err = PTR_ERR(wh_hidden_dentry);
			goto out_free;
		}

		if (wh_hidden_dentry->d_inode) {
			/* We found a whiteout so lets give up. */
			fist_dprint(8, "whiteout found in %d\n", bindex);
			if (S_ISREG(wh_hidden_dentry->d_inode->i_mode)) {
				set_dbend(dentry, bindex);
				set_dbopaque(dentry, bindex);
				DPUT(wh_hidden_dentry);
				break;
			}
			err = -EIO;
			printk(KERN_NOTICE "EIO: Invalid whiteout entry type"
			       " %d.\n", wh_hidden_dentry->d_inode->i_mode);
			DPUT(wh_hidden_dentry);
			DPUT(first_hidden_dentry);
			goto out_free;
		}

		DPUT(wh_hidden_dentry);
		wh_hidden_dentry = NULL;

		/* Now do regular lookup; lookup foo */
		hidden_dentry = LOOKUP_ONE_LEN(name, hidden_dir_dentry,
					       namelen);
		fist_print_generic_dentry("hidden result", hidden_dentry);
		if (IS_ERR(hidden_dentry)) {
			DPUT(first_hidden_dentry);
			err = PTR_ERR(hidden_dentry);
			goto out_free;
		}

		/* Store the first negative dentry specially, because if they
		 * are all negative we need this for future creates. */
		if (!hidden_dentry->d_inode) {
			if (!first_hidden_dentry && (dbstart(dentry) == -1)) {
				first_hidden_dentry = hidden_dentry;
				first_dentry_offset = bindex;
			} else {
				DPUT(hidden_dentry);
			}
			continue;
		}

		/* number of positive dentries */
		dentry_count++;

		/* store underlying dentry */
		if (dbstart(dentry) == -1)
			set_dbstart(dentry, bindex);
		set_dtohd_index(dentry, bindex, hidden_dentry);
		set_dbend(dentry, bindex);

		/* update parent directory's atime with the bindex */
		fist_copy_attr_atime(parent_dentry->d_inode,
				     hidden_dir_dentry->d_inode);

		/* We terminate file lookups here. */
		if (!S_ISDIR(hidden_dentry->d_inode->i_mode)) {
			if (lookupmode == INTERPOSE_PARTIAL)
				continue;
			if (dentry_count == 1)
				goto out_positive;
			/* This can only happen with mixed D-*-F-* */
			BUG_ON(!S_ISDIR(dtohd(dentry)->d_inode->i_mode));
			continue;
		}

		opaque = is_opaque_dir(dentry, bindex);
		if (opaque < 0) {
			DPUT(first_hidden_dentry);
			err = opaque;
			goto out_free;
		}
		if (opaque) {
			set_dbend(dentry, bindex);
			set_dbopaque(dentry, bindex);
			break;
		}
	}

	if (dentry_count)
		goto out_positive;
	else
		goto out_negative;

      out_negative:
	if (lookupmode == INTERPOSE_PARTIAL)
		goto out;

	/* If we've only got negative dentries, then use the leftmost one. */
	if (lookupmode == INTERPOSE_REVAL) {
		if (dentry->d_inode) {
			itopd(dentry->d_inode)->uii_stale = 1;
		}
		goto out;
	}
	/* This should only happen if we found a whiteout. */
	if (first_dentry_offset == -1) {
		first_hidden_dentry = LOOKUP_ONE_LEN(name, hidden_dir_dentry,
						     namelen);
		first_dentry_offset = bindex;
		if (IS_ERR(first_hidden_dentry)) {
			err = PTR_ERR(first_hidden_dentry);
			goto out;
		}
	}
	set_dtohd_index(dentry, first_dentry_offset, first_hidden_dentry);
	set_dbstart(dentry, first_dentry_offset);
	set_dbend(dentry, first_dentry_offset);

	if (lookupmode == INTERPOSE_REVAL_NEG)
		BUG_ON(dentry->d_inode != NULL);
	else
		d_add(dentry, NULL);
	goto out;

/* This part of the code is for positive dentries. */
      out_positive:
	BUG_ON(dentry_count <= 0);

	/* If we're holding onto the first negative dentry throw it out. */
	DPUT(first_hidden_dentry);

	/* Partial lookups need to reinterpose, or throw away older negs. */
	if (lookupmode == INTERPOSE_PARTIAL) {
		if (dentry->d_inode) {
			unionfs_reinterpose(dentry);
			goto out;
		}

		/* This somehow turned positive, so it is as if we had a
		 * negative revalidation.  */
		lookupmode = INTERPOSE_REVAL_NEG;

		update_bstart(dentry);
		bstart = dbstart(dentry);
		bend = dbend(dentry);
	}

	err = unionfs_interpose(dentry, dentry->d_sb, lookupmode);
	if (err)
		goto out_drop;

	fist_checkinode(dentry->d_inode, "unionfs_lookup OUT: child");
	fist_checkinode(parent_dentry->d_inode, "unionfs_lookup OUT: dir");
	goto out;

      out_drop:
	d_drop(dentry);

      out_free:
	/* should dput all the underlying dentries on error condition */
	bstart = dbstart(dentry);
	if (bstart >= 0) {
		bend = dbend(dentry);
		for (bindex = bstart; bindex <= bend; bindex++)
			DPUT(dtohd_index(dentry, bindex));
	}
	KFREE(dtohd_ptr(dentry));
	dtohd_ptr(dentry) = NULL;
	set_dbstart(dentry, -1);
	set_dbend(dentry, -1);

      out:
	if (!err && dtopd(dentry)) {
		BUG_ON(dbend(dentry) > dtopd(dentry)->udi_bcount);
		BUG_ON(dbend(dentry) > sbmax(dentry->d_sb));
		BUG_ON(dbstart(dentry) < 0);
	}
	KFREE(whname);
	fist_print_dentry("OUT unionfs_lookup (parent)", parent_dentry);
	fist_print_dentry("OUT unionfs_lookup (child)", dentry);
	if (locked_parent)
		unlock_dentry(parent_dentry);
	DPUT(parent_dentry);
	if (locked_child)
		unlock_dentry(dentry);
	print_exit_status(err);
	return ERR_PTR(err);
}

/* This is a utility function that fills in a unionfs dentry.*/
int unionfs_partial_lookup(struct dentry *dentry)
{
	struct dentry *tmp;

	tmp = unionfs_lookup_backend(dentry, INTERPOSE_PARTIAL);
	if (!tmp)
		return 0;
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);
	/* need to change the interface */
	BUG_ON(tmp != dentry);
	return -ENOSYS;
}

/* The rest of these are utility functions for lookup. */
static int is_opaque_dir(struct dentry *dentry, int bindex)
{
	int err = 0;
	struct dentry *hidden_dentry;
	struct dentry *wh_hidden_dentry;
	struct inode *hidden_inode;
	uid_t saved_uid = current->fsuid;
	gid_t saved_gid = current->fsgid;

	print_entry_location();

	hidden_dentry = dtohd_index(dentry, bindex);
	hidden_inode = hidden_dentry->d_inode;

	BUG_ON(!S_ISDIR(hidden_inode->i_mode));

	current->fsuid = hidden_inode->i_uid;
	current->fsgid = hidden_inode->i_gid;
	wh_hidden_dentry = LOOKUP_ONE_LEN(UNIONFS_DIR_OPAQUE, hidden_dentry,
					  sizeof(UNIONFS_DIR_OPAQUE) - 1);
	current->fsuid = saved_uid;
	current->fsgid = saved_gid;
	if (IS_ERR(wh_hidden_dentry)) {
		err = PTR_ERR(wh_hidden_dentry);
		fist_dprint(1, "LOOKUP_ONE_LEN returned: %d\n", err);
		goto out;
	}
	if (wh_hidden_dentry->d_inode)
		err = 1;
	DPUT(wh_hidden_dentry);
      out:
	print_exit_status(err);
	return err;
}

static int is_validname(const char *name)
{
	if (!strncmp(name, WHPFX, WHLEN))
		return 0;
	if (!strncmp(name, UNIONFS_DIR_OPAQUE_NAME,
		     sizeof(UNIONFS_DIR_OPAQUE_NAME) - 1))
		return 0;
	return 1;
}

/* The dentry cache is just so we have properly sized dentries. */
static kmem_cache_t *unionfs_dentry_cachep;
int init_dentry_cache(void)
{
	unionfs_dentry_cachep =
	    kmem_cache_create("unionfs_dentry",
			      sizeof(struct unionfs_dentry_info), 0,
			      SLAB_RECLAIM_ACCOUNT, NULL, NULL);

	if (!unionfs_dentry_cachep)
		return -ENOMEM;
	return 0;
}

void destroy_dentry_cache(void)
{
	if (!unionfs_dentry_cachep)
		return;
	if (kmem_cache_destroy(unionfs_dentry_cachep))
		printk(KERN_ERR
		       "unionfs_dentry_cache: not all structures were freed\n");
	return;
}

void free_dentry_private_data(struct unionfs_dentry_info *udi)
{
	if (!udi)
		return;
	kmem_cache_free(unionfs_dentry_cachep, udi);
}

int new_dentry_private_data(struct dentry *dentry)
{
	int newsize;
	int oldsize = 0;

	spin_lock(&dentry->d_lock);
	if (!dtopd_nocheck(dentry)) {
		dtopd_lhs(dentry) = (struct unionfs_dentry_info *)
		    kmem_cache_alloc(unionfs_dentry_cachep, SLAB_ATOMIC);
		if (!dtopd_nocheck(dentry))
			goto out;
		init_MUTEX_LOCKED(&dtopd_nocheck(dentry)->udi_sem);
#ifdef TRACKLOCK
		printk("INITLOCK:%p\n", dentry);
#endif
		dtohd_ptr(dentry) = NULL;
	} else {
		oldsize = sizeof(struct dentry *) * dtopd(dentry)->udi_bcount;
	}

	dtopd_nocheck(dentry)->udi_bstart = -1;
	dtopd_nocheck(dentry)->udi_bend = -1;
	dtopd_nocheck(dentry)->udi_bopaque = -1;
	dtopd_nocheck(dentry)->udi_bcount = sbmax(dentry->d_sb);
	atomic_set(&dtopd_nocheck(dentry)->udi_generation,
		   atomic_read(&stopd(dentry->d_sb)->usi_generation));
	newsize = sizeof(struct dentry *) * sbmax(dentry->d_sb);

	/* Don't reallocate when we already have enough space. */
	/* It would be ideal if we could actually use the slab macros to
	 * determine what our object sizes is, but those are not exported.
	 */
	if (oldsize) {
		int minsize = malloc_sizes[0].cs_size;

		if (!newsize || ((oldsize < newsize) && (newsize > minsize))) {
			KFREE(dtohd_ptr(dentry));
			dtohd_ptr(dentry) = NULL;
		}
	}

	if (!dtohd_ptr(dentry) && newsize) {
		dtohd_ptr(dentry) = KMALLOC(newsize, GFP_ATOMIC);
		if (!dtohd_ptr(dentry))
			goto out;
	}

	if (oldsize > newsize)
		memset(dtohd_ptr(dentry), 0, oldsize);
	else
		memset(dtohd_ptr(dentry), 0, newsize);

	spin_unlock(&dentry->d_lock);
	return 0;

      out:
	free_dentry_private_data(dtopd_nocheck(dentry));
	dtopd_lhs(dentry) = NULL;
	spin_unlock(&dentry->d_lock);
	return -ENOMEM;
}

void update_bstart(struct dentry *dentry)
{
	int bindex;
	int bstart = dbstart(dentry);
	int bend = dbend(dentry);
	struct dentry *hidden_dentry;

	for (bindex = bstart; bindex <= bend; bindex++) {
		hidden_dentry = dtohd_index(dentry, bindex);
		if (!hidden_dentry)
			continue;
		if (hidden_dentry->d_inode) {
			set_dbstart(dentry, bindex);
			break;
		}
		DPUT(hidden_dentry);
		set_dtohd_index(dentry, bindex, NULL);
	}
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
