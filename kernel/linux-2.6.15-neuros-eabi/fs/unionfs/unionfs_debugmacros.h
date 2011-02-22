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
 *  $Id: unionfs_debugmacros.h,v 1.18 2006/02/15 09:07:28 jsipek Exp $
 */

#ifndef __UNIONFS_H_
#error This file should only be included from unionfs.h!
#endif

/* File to hidden file */

static inline struct file *ftohf_index(const struct file *f, int index)
{
	struct file *hidden_file;
	hidden_file = ftopd(f)->ufi_file[index];

	BUG_ON(hidden_file && (atomic_read(&hidden_file->f_count) <= 0));

	return hidden_file;
}

static inline struct file *ftohf(const struct file *f)
{
	BUG_ON(ftopd(f)->b_start < 0);
	return ftohf_index(f, fbstart(f));
}

static inline struct file *set_ftohf_index(struct file *f, int index,
					   struct file *val)
{
	BUG_ON(index < 0);
	BUG_ON(val && (atomic_read(&val->f_count) <= 0));

	ftopd(f)->ufi_file[index] = val;
	return val;
}

static inline struct file *set_ftohf(struct file *f, struct file *val)
{
	BUG_ON(ftopd(f)->b_start < 0);
	return set_ftohf_index(f, fbstart(f), val);
}

/* Inode to hidden inode. */

static inline struct inode *itohi_index(const struct inode *ino, int index)
{
	struct inode *hidden_inode;
	BUG_ON(index < 0);
	hidden_inode = itopd(ino)->uii_inode[index];
	if (hidden_inode)
		BUG_ON((atomic_read(&hidden_inode->i_count)) <= 0);
	return hidden_inode;
}

static inline struct inode *itohi(const struct inode *ino)
{
	BUG_ON(itopd(ino)->b_start < 0);
	return itohi_index(ino, itopd(ino)->b_start);
}

static inline struct inode *set_itohi_index(struct inode *ino, int index,
					    struct inode *val)
{
	BUG_ON(index < 0);
	BUG_ON(val && (atomic_read(&val->i_count) <= 0));

	itopd(ino)->uii_inode[index] = val;
	return val;
}

/* Superblock to hidden superblock */
static inline struct super_block *stohs_index(const struct super_block *f,
					      int index)
{
	return stopd(f)->usi_data[index].sb;
}

static inline struct super_block *stohs(const struct super_block *sb)
{
	BUG_ON(sbstart(sb) != 0);
	return stohs_index(sb, sbstart(sb));
}

static inline struct super_block *set_stohs_index(struct super_block *sb,
						  int index,
						  struct super_block *val)
{
	stopd(sb)->usi_data[index].sb = val;
	return val;
}

/* Superblock to hidden vfsmount  */
static inline struct vfsmount *stohiddenmnt_index(struct super_block *sb,
						  int index)
{
	BUG_ON(index < 0);
	return stopd(sb)->usi_data[index].hidden_mnt;
}

static inline void set_stohiddenmnt_index(struct super_block *sb, int index,
					  struct vfsmount *mnt)
{
	BUG_ON(index < 0);
	stopd(sb)->usi_data[index].hidden_mnt = mnt;
}

/* Get and put branches on the superblock. */
static inline void set_branch_count(struct super_block *sb, int index,
				    int count)
{
	BUG_ON(index < 0);
	atomic_set(&stopd(sb)->usi_data[index].sbcount, count);
}

static inline int branch_count(struct super_block *sb, int index)
{
	BUG_ON(index < 0);
	return atomic_read(&stopd(sb)->usi_data[index].sbcount);
}

static inline void branchget(struct super_block *sb, int index)
{
	BUG_ON(index < 0);
	atomic_inc(&stopd(sb)->usi_data[index].sbcount);
	BUG_ON(atomic_read(&stopd(sb)->usi_data[index].sbcount) < 0);
}

static inline void branchput(struct super_block *sb, int index)
{
	BUG_ON(index < 0);
	atomic_dec(&stopd(sb)->usi_data[index].sbcount);
	BUG_ON(atomic_read(&stopd(sb)->usi_data[index].sbcount) < 0);
}

/* Dentry to Hidden Dentry  */
static inline struct unionfs_dentry_info *__dtopd(const struct dentry *dent,
						  int check)
{
	struct unionfs_dentry_info *ret;

	ret = (struct unionfs_dentry_info *)(dent)->d_fsdata;
	/* We are really only interested in catching poison here. */
	if (ret && check) {
		if ((ret->udi_bend > ret->udi_bcount)
		    || (ret->udi_bend > sbmax(dent->d_sb))) {
			printk(KERN_EMERG
			       "udi_bend = %d, udi_count = %d, sbmax = %d\n",
			       ret->udi_bend, ret->udi_bcount,
			       sbmax(dent->d_sb));
		}
		BUG_ON(ret->udi_bend > ret->udi_bcount);
		BUG_ON(ret->udi_bend > sbmax(dent->d_sb));
	}

	return ret;
}

#define dtopd(dent) __dtopd(dent, 1)
#define dtopd_nocheck(dent) __dtopd(dent, 0)
#define dtopd_lhs(dent) ((dent)->d_fsdata)

/* Macros for locking a dentry. */
static inline void lock_dentry(struct dentry *d)
{
#ifdef TRACKLOCK
	printk("LOCK:%p\n", d);
#endif
	down(&dtopd(d)->udi_sem);
}

static inline void unlock_dentry(struct dentry *d)
{
#ifdef TRACKLOCK
	printk("UNLOCK:%p\n", d);
#endif
	up(&dtopd(d)->udi_sem);
}

static inline void verify_locked(const struct dentry *d)
{
#ifdef TRACKLOCK
	printk("MUST BE LOCKED:%p\n", d);
#endif
	BUG_ON(down_trylock(&dtopd(d)->udi_sem) == 0);
}

static inline int dbend(const struct dentry *dentry)
{
	verify_locked(dentry);
	return dtopd(dentry)->udi_bend;
}

static inline int set_dbend(const struct dentry *dentry, int val)
{
	verify_locked(dentry);

	BUG_ON((val < 0) && (val != -1));
	BUG_ON(val > dtopd_nocheck(dentry)->udi_bcount);
	BUG_ON(val > sbmax(dentry->d_sb));

	dtopd(dentry)->udi_bend = val;
	return dtopd(dentry)->udi_bend;
}

static inline int dbstart(const struct dentry *dentry)
{
	verify_locked(dentry);
	return dtopd(dentry)->udi_bstart;
}

static inline int set_dbstart(const struct dentry *dentry, int val)
{
	verify_locked(dentry);

	BUG_ON((val < 0) && (val != -1));
	BUG_ON(val > dtopd_nocheck(dentry)->udi_bcount);
	BUG_ON(val > sbmax(dentry->d_sb));

	dtopd_nocheck(dentry)->udi_bstart = val;
	return dtopd(dentry)->udi_bstart;
}

static inline int dbopaque(const struct dentry *dentry)
{
	verify_locked(dentry);
	return dtopd(dentry)->udi_bopaque;
}

static inline int set_dbopaque(const struct dentry *dentry, int val)
{
	verify_locked(dentry);

	BUG_ON((val < 0) && (val != -1));
	BUG_ON(val > dtopd_nocheck(dentry)->udi_bcount);
	BUG_ON(val > sbmax(dentry->d_sb));
	BUG_ON(val > dbend(dentry));

	dtopd_nocheck(dentry)->udi_bopaque = val;
	return dtopd(dentry)->udi_bopaque;
}

/* Dentry to hidden dentry functions */
#define dtohd_index(dent, index) __dtohd_index(dent, index, 1)
#define dtohd_index_nocheck(dent, index) __dtohd_index(dent, index, 0)
/* This pointer should not be generally used except for maintainence functions. */
#define dtohd_ptr(dent) (dtopd_nocheck(dent)->udi_dentry)
static inline struct dentry *__dtohd_index(const struct dentry *dent, int index,
					   int docheck)
{
	struct dentry *d;

	verify_locked(dent);
	BUG_ON(index < 0);
	if (docheck) {
		if (index > sbend(dent->d_sb)) {
			printk
			    ("Dentry index out of super bounds: index=%d, sbend=%d\n",
			     index, sbend(dent->d_sb));
			BUG_ON(index > sbend(dent->d_sb));
		}
		if (index > dtopd(dent)->udi_bcount) {
			printk
			    ("Dentry index out of array bounds: index=%d, count=%d\n",
			     index, dtopd(dent)->udi_bcount);
			printk("Generation of dentry: %d\n",
			       atomic_read(&dtopd(dent)->udi_generation));
			printk("Generation of sb: %d\n",
			       atomic_read(&stopd(dent->d_sb)->usi_generation));
			BUG_ON(index > dtopd(dent)->udi_bcount);
		}
	}
	d = dtopd(dent)->udi_dentry[index];

	BUG_ON(d && (atomic_read(&d->d_count) <= 0));

	return d;
}

static inline struct dentry *dtohd(const struct dentry *dent)
{
	struct dentry *d;
	int index;

	verify_locked(dent);
	BUG_ON(dbstart(dent) < 0);
	index = dbstart(dent);
	d = dtohd_index(dent, index);

	return d;
}

// Dentry to Hidden Dentry based on index
#define set_dtohd_index(dent, index, val) __set_dtohd_index(dent, index, val, 1)
#define set_dtohd_index_nocheck(dent, index, val) __set_dtohd_index(dent, index, val, 0)
static inline struct dentry *__set_dtohd_index(struct dentry *dent, int index,
					       struct dentry *val, int docheck)
{
#ifdef FIST_MALLOC_DEBUG
	struct dentry *old;
#endif

	verify_locked(dent);
	BUG_ON(index < 0);
	if (docheck) {
		if (index > sbend(dent->d_sb)) {
			printk
			    ("Dentry index out of super bounds: index=%d, sbend=%d\n",
			     index, sbend(dent->d_sb));
			BUG_ON(index > sbend(dent->d_sb));
		}
		if (index > dtopd(dent)->udi_bcount) {
			printk
			    ("Dentry index out of array bounds: index=%d, count=%d\n",
			     index, dtopd(dent)->udi_bcount);
			printk("Generation of dentry: %d\n",
			       atomic_read(&dtopd(dent)->udi_generation));
			printk("Generation of sb: %d\n",
			       atomic_read(&stopd(dent->d_sb)->usi_generation));
			BUG_ON(index > dtopd(dent)->udi_bcount);
		}
	}
#ifdef FIST_MALLOC_DEBUG
	old = dtopd(dent)->udi_dentry[index];
	record_set(dent, index, val, old, line, file);
#endif
	dtopd(dent)->udi_dentry[index] = val;

	return val;
}

extern int fist_get_debug_value(void);
extern int fist_set_debug_value(int val);
extern void fist_dprint_internal(const char *file, const char *function,
				 int line, int level, char *str, ...)
    __attribute__ ((format(__printf__, 5, 6)));

extern void fist_print_dentry(const char *, const struct dentry *);
extern void __fist_print_dentry(const char *, const struct dentry *, int);
extern void fist_print_generic_dentry(const char *, const struct dentry *);
extern void fist_print_generic_dentry3(const char *, const char *,
				       const struct dentry *);
extern void __fist_print_generic_dentry(const char *, const char *, const
					struct dentry *, int);
extern void fist_print_inode(const char *, const struct inode *);
extern void fist_print_generic_inode(const char *, const struct inode *);
extern void fist_print_file(const char *, const struct file *);
extern void fist_checkinode(const struct inode *, const char *);
extern void fist_print_sb(const char *str, const struct super_block *);

extern char *add_indent(void);
extern char *del_indent(void);

#define fist_dprint(level, str, args...) fist_dprint_internal(__FILE__, __FUNCTION__, __LINE__, level, KERN_DEBUG str, ## args)
#define print_entry(format, args...) fist_dprint(4, "%sIN:  %s %s:%d " format "\n", add_indent(), __FUNCTION__, __FILE__, __LINE__, ##args)
#define print_entry_location() fist_dprint(4, "%sIN:  %s %s:%d\n", add_indent(), __FUNCTION__, __FILE__, __LINE__)
#define print_exit_location() fist_dprint(5, "%s OUT: %s %s:%d\n", del_indent(), __FUNCTION__, __FILE__, __LINE__)
#define print_exit_status(status) fist_dprint(5, "%s OUT: %s %s:%d, STATUS: %d\n", del_indent(), __FUNCTION__, __FILE__, __LINE__, status)
#define print_exit_pointer(status) \
do { \
  if (IS_ERR(status)) \
    fist_dprint(5, "%s OUT: %s %s:%d, RESULT: %ld\n", del_indent(), __FUNCTION__, __FILE__, __LINE__, PTR_ERR(status)); \
  else \
    fist_dprint(5, "%s OUT: %s %s:%d, RESULT: 0x%p\n", del_indent(), __FUNCTION__, __FILE__, __LINE__, status); \
} while (0)

#define print_util_entry(format, args...) fist_dprint(6, "%sIN:  %s %s:%d" format "\n", add_indent(), __FUNCTION__, __FILE__, __LINE__, ##args)
#define print_util_entry_location() fist_dprint(6, "%sIN:  %s %s:%d\n", add_indent(), __FUNCTION__, __FILE__, __LINE__)
#define print_util_exit_location() fist_dprint(7, "%s OUT: %s %s:%d\n", del_indent(), __FUNCTION__, __FILE__, __LINE__)
#define print_util_exit_status(status) fist_dprint(7, "%s OUT: %s %s:%d, STATUS: %d\n", del_indent(), __FUNCTION__, __FILE__, __LINE__, status)
#define print_util_exit_pointer(status) \
do { \
  if (IS_ERR(status)) \
    fist_dprint(7, "%s OUT: %s %s:%d, RESULT: %ld\n", del_indent(), __FUNCTION__, __FILE__, __LINE__, PTR_ERR(status)); \
  else \
    fist_dprint(5, "%s OUT: %s %s:%d, RESULT: 0x%x\n", del_indent(), __FUNCTION__, __FILE__, __LINE__, PTR_ERR(status)); \
} while (0)

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
