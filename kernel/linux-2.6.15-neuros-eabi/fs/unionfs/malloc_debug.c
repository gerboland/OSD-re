#ifdef FIST_MALLOC_DEBUG

#include "unionfs.h"

/* for malloc debugging */
atomic_t unionfs_malloc_counter = ATOMIC_INIT(0);
atomic_t unionfs_mallocs_outstanding = ATOMIC_INIT(0);
atomic_t unionfs_dget_counter = ATOMIC_INIT(0);
atomic_t unionfs_dgets_outstanding = ATOMIC_INIT(0);
atomic_t unionfs_iget_counter = ATOMIC_INIT(0);
atomic_t unionfs_igets_outstanding = ATOMIC_INIT(0);

void *unionfs_kzalloc(size_t size, gfp_t flags, int line, const char *file)
{
	void *ptr = kzalloc(size, flags);
	if (ptr) {
		atomic_inc(&unionfs_malloc_counter);
		atomic_inc(&unionfs_mallocs_outstanding);
		printk("KZA:%d:%d:%p:%d:%s\n",
		       atomic_read(&unionfs_malloc_counter),
		       atomic_read(&unionfs_mallocs_outstanding), ptr, line,
		       file);
	}
	return ptr;
}
void *unionfs_kmalloc(size_t size, gfp_t flags, int line, const char *file)
{
	void *ptr = kmalloc(size, flags);
	if (ptr) {
		atomic_inc(&unionfs_malloc_counter);
		atomic_inc(&unionfs_mallocs_outstanding);
		printk("KM:%d:%d:%p:%d:%s\n",
		       atomic_read(&unionfs_malloc_counter),
		       atomic_read(&unionfs_mallocs_outstanding), ptr, line,
		       file);
	}
	return ptr;
}

void unionfs_kfree(void *ptr, int line, const char *file)
{
	atomic_inc(&unionfs_malloc_counter);
	if (ptr) {
		BUG_ON(IS_ERR(ptr));
		atomic_dec(&unionfs_mallocs_outstanding);
	}
	printk("KF:%d:%d:%p:%d:%s\n", atomic_read(&unionfs_malloc_counter),
	       atomic_read(&unionfs_mallocs_outstanding), ptr, line, file);
	kfree(ptr);
}

void record_set(struct dentry *upper, int index, struct dentry *ptr,
		struct dentry *old, int line, const char *file)
{
	atomic_inc(&unionfs_dget_counter);
	printk("DD:%d:%d:%d:%p:%d:%s %p, %d\n",
	       atomic_read(&unionfs_dget_counter),
	       atomic_read(&unionfs_dgets_outstanding),
	       old ? atomic_read(&old->d_count) : 0, old, line, file, upper,
	       index);
	atomic_inc(&unionfs_dget_counter);
	printk("DS:%d:%d:%d:%p:%d:%s %p, %d\n",
	       atomic_read(&unionfs_dget_counter),
	       atomic_read(&unionfs_dgets_outstanding),
	       ptr ? atomic_read(&ptr->d_count) : 0, ptr, line, file, upper,
	       index);
}

void record_path_lookup(struct nameidata *nd, int line, const char *file)
{
	struct dentry *ptr = nd->dentry;
	if (ptr) {
		atomic_inc(&unionfs_dget_counter);
		atomic_inc(&unionfs_dgets_outstanding);
		printk("DL:%d:%d:%d:%p:%d:%s\n",
		       atomic_read(&unionfs_dget_counter),
		       atomic_read(&unionfs_dgets_outstanding),
		       atomic_read(&ptr->d_count), ptr, line, file);
	}
}

void record_path_release(struct nameidata *nd, int line, const char *file)
{
	struct dentry *ptr = nd->dentry;

	atomic_inc(&unionfs_dget_counter);
	if (ptr)
		atomic_dec(&unionfs_dgets_outstanding);
	printk("DP:%d:%d:%d:%p:%d:%s\n", atomic_read(&unionfs_dget_counter),
	       atomic_read(&unionfs_dgets_outstanding),
	       ptr ? atomic_read(&ptr->d_count) : 0, ptr, line, file);
}

struct file *unionfs_dentry_open(struct dentry *ptr, struct vfsmount *mnt,
				 int flags, int line, const char *file)
{
	atomic_inc(&unionfs_dget_counter);
	if (ptr)
		atomic_dec(&unionfs_dgets_outstanding);
	printk("DO:%d:%d:%d:%p:%d:%s\n", atomic_read(&unionfs_dget_counter),
	       atomic_read(&unionfs_dgets_outstanding),
	       ptr ? atomic_read(&ptr->d_count) : 0, ptr, line, file);
	return dentry_open(ptr, mnt, flags);
}

struct dentry *unionfs_dget(struct dentry *ptr, int line, const char *file)
{
	ptr = dget(ptr);
	if (ptr) {
		atomic_inc(&unionfs_dget_counter);
		atomic_inc(&unionfs_dgets_outstanding);
		printk("DG:%d:%d:%d:%p:%d:%s\n",
		       atomic_read(&unionfs_dget_counter),
		       atomic_read(&unionfs_dgets_outstanding),
		       atomic_read(&ptr->d_count), ptr, line, file);
	}
	return ptr;
}

struct dentry *unionfs_dget_parent(struct dentry *child, int line,
				   const char *file)
{
	struct dentry *ptr;

	ptr = dget_parent(child);
	atomic_inc(&unionfs_dget_counter);
	atomic_inc(&unionfs_dgets_outstanding);
	printk("DG:%d:%d:%d:%p:%d:%s\n",
	       atomic_read(&unionfs_dget_counter),
	       atomic_read(&unionfs_dgets_outstanding),
	       atomic_read(&ptr->d_count), ptr, line, file);

	return ptr;
}

struct dentry *unionfs_lookup_one_len(const char *name, struct dentry *parent,
				      int len, int line, const char *file)
{
	struct dentry *ptr = lookup_one_len(name, parent, len);
	if (ptr && !IS_ERR(ptr)) {
		atomic_inc(&unionfs_dget_counter);
		atomic_inc(&unionfs_dgets_outstanding);
		printk("DL:%d:%d:%d:%p:%d:%s\n",
		       atomic_read(&unionfs_dget_counter),
		       atomic_read(&unionfs_dgets_outstanding),
		       atomic_read(&ptr->d_count), ptr, line, file);
	}
	return ptr;
}

void unionfs_dput(struct dentry *ptr, int line, const char *file)
{
	atomic_inc(&unionfs_dget_counter);
	if (ptr) {
		BUG_ON(IS_ERR(ptr));
		atomic_dec(&unionfs_dgets_outstanding);
	}
	printk("DP:%d:%d:%d:%p:%d:%s\n", atomic_read(&unionfs_dget_counter),
	       atomic_read(&unionfs_dgets_outstanding),
	       ptr ? atomic_read(&ptr->d_count) : 0, ptr, line, file);
	dput(ptr);
}

struct inode *unionfs_igrab(struct inode *inode, int line, char *file)
{
	atomic_inc(&unionfs_iget_counter);
	if (inode)
		atomic_inc(&unionfs_igets_outstanding);
	printk("IR:%d:%d:%d:%p:%d:%s\n", atomic_read(&unionfs_iget_counter),
	       atomic_read(&unionfs_igets_outstanding),
	       inode ? atomic_read(&inode->i_count) : 0, inode, line, file);
	return igrab(inode);
}

void unionfs_iput(struct inode *inode, int line, char *file)
{
	atomic_inc(&unionfs_iget_counter);
	if (inode)
		atomic_dec(&unionfs_igets_outstanding);
	printk("IP:%d:%d:%d:%p:%d:%s\n", atomic_read(&unionfs_iget_counter),
	       atomic_read(&unionfs_igets_outstanding),
	       inode ? atomic_read(&inode->i_count) : 0, inode, line, file);
	iput(inode);
}

struct inode *unionfs_iget(struct super_block *sb, unsigned long ino, int line,
			   char *file)
{
	struct inode *inode = iget(sb, ino);
	atomic_inc(&unionfs_iget_counter);
	if (inode)
		atomic_inc(&unionfs_igets_outstanding);
	printk("IG:%d:%d:%d:%p:%d:%s\n", atomic_read(&unionfs_iget_counter),
	       atomic_read(&unionfs_igets_outstanding),
	       inode ? atomic_read(&inode->i_count) : 0, inode, line, file);
	return inode;
}

#endif				/* FIST_MALLOC_DEBUG */
