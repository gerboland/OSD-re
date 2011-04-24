#ifndef _LINUX_INOTIFY_SYSCALLS_H
#define _LINUX_INOTIFY_SYSCALLS_H

#include <errno.h>
#include <sys/syscall.h>

#if defined (__arm__)
	//These syscall numbers are probably already defined in the system syscalls header.
	//But define them if they are not --nero
	#ifndef __NR_inotify_init
		#define __NR_inotify_init __NR_SYSCALL_BASE + 316
	#endif
	
	#ifndef __NR_inotify_add_watch
		#define __NR_inotify_add_watch __NR_SYSCALL_BASE + 317
	#endif

	#ifndef __NR_inotify_rm_watch
		#define __NR_inotify_rm_watch  __NR_SYSCALL_BASE + 318
	#endif
#else
	# error "Unsupported architecture!"
#endif

// Implement these syscalls as inline, so that the linker will always find them and
// won't look into glibc, that doesn't have them in our version.
// Resetting errno is done to avoid spurious error codes since syscall doesn't seem to reset it.

static inline int inotify_init (void)
{
	errno = 0;
	return syscall (__NR_inotify_init);
}

static inline int inotify_add_watch (int fd, const char *name, __u32 mask)
{
	errno = 0;
	return syscall (__NR_inotify_add_watch, fd, name, mask);
}

static inline int inotify_rm_watch (int fd, __u32 wd)
{
	errno = 0;
	return syscall (__NR_inotify_rm_watch, fd, wd);
}

#endif /* _LINUX_INOTIFY_SYSCALLS_H */
