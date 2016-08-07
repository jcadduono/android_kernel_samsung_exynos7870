/* sec_ext.h
 *
 * Copyright (C) 2014 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef SEC_EXT_H
#define SEC_EXT_H

#ifdef CONFIG_SEC_BOOTSTAT
extern void sec_bootstat_mct_start(u64 t);
extern void sec_bootstat_add(const char * c);
extern void sec_bootstat_add_initcall(const char *);

extern void sec_bootstat_get_cpuinfo(int *freq, int *online);
extern void sec_bootstat_get_thermal(int *temp, int size);
#else
#define sec_bootstat_mct_start(a)		do { } while(0)
#define sec_bootstat_add(a)			do { } while(0)
#define sec_bootstat_add_initcall(a)		do { } while(0)

#define sec_bootstat_get_cpuinfo(a,b)		do { } while(0)	
#define sec_bootstat_get_thermal(a,b)		do { } while(0)	
#endif /* CONFIG_SEC_BOOT_STAT */

#ifdef CONFIG_SEC_INITCALL_DEBUG
#define SEC_INITCALL_DEBUG_MIN_TIME		10000

extern void sec_initcall_debug_add(initcall_t fn, unsigned long long t);
#else
#define sec_initcall_debug_add(a,b)		do { } while(0)	
#endif /* CONFIG_SEC_INITCALL_DEBUG */

#endif
