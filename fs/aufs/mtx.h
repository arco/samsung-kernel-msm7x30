/*
 * Copyright (C) 2010-2011 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*
 * very ugly approach for aufs_mmap()
 * never include this file from other than f_op.c.
 * see f_op.c in detail.
 */

#ifndef __AUFS_MTX_H__
#define __AUFS_MTX_H__

#ifdef __KERNEL__

/* copied from ../kernel/mutex{,-debug}.h */
struct mutex;
struct thread_info;
#ifdef CONFIG_DEBUG_MUTEXES
static inline void mutex_set_owner(struct mutex *lock)
{
	lock->owner = current_thread_info();
}
#else
static inline void mutex_set_owner(struct mutex *lock)
{
#ifdef CONFIG_SMP
	lock->owner = current_thread_info();
#endif
}
#endif

#endif /* __KERNEL__ */
#endif /* __AUFS_MTX_H__ */
