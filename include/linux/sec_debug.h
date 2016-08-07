/*
* Samsung debugging features for Samsung's SoC's.
*
* Copyright (c) 2014 Samsung Electronics Co., Ltd.
*      http://www.samsung.com
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*/

#ifndef SEC_DEBUG_H 
#define SEC_DEBUG_H

#include <soc/samsung/exynos-pmu.h>
/* 
 * PMU register offset : MUST MODIFY ACCORDING TO SoC
 */
#define EXYNOS_PMU_INFORM2 0x0808
#define EXYNOS_PMU_INFORM3 0x080C
#define EXYNOS_PMU_PS_HOLD_CONTROL 0x330C

#ifdef CONFIG_SEC_DEBUG
extern int  sec_debug_setup(void);
extern void sec_debug_reboot_handler(void);
extern void sec_debug_panic_handler(void *buf, bool dump);
extern void sec_debug_post_panic_handler(void);

extern int  sec_debug_get_debug_level(void);
extern void sec_debug_disable_printk_process(void);

/* getlog support */
extern void sec_getlog_supply_kernel(void *klog_buf);
extern void sec_getlog_supply_platform(unsigned char *buffer, const char *name);

extern void sec_gaf_supply_rqinfo(unsigned short curr_offset, unsigned short rq_offset);
#else
#define sec_debug_setup()			(-1)
#define sec_debug_reboot_handler()		do { } while(0)
#define sec_debug_panic_handler(a,b)		do { } while(0)
#define sec_debug_post_panic_handler()		do { } while(0)	

#define sec_debug_get_debug_level()		0
#define sec_debug_disable_printk_process()	do { } while(0)

#define sec_getlog_supply_kernel(a)		do { } while(0)
#define sec_getlog_supply_platform(a,b)		do { } while(0)

static inline void sec_gaf_supply_rqinfo(unsigned short curr_offset, unsigned short rq_offset)
{
    return;
}
#endif /* CONFIG_SEC_DEBUG */

#ifdef CONFIG_SEC_DEBUG_MDM_SEPERATE_CRASH
extern int  sec_debug_is_enabled_for_ssr(void);
#endif

/* sec logging */
#ifdef CONFIG_SEC_AVC_LOG
extern void sec_debug_avc_log(char *fmt, ...);
#else
#define sec_debug_avc_log(a, ...)		do { } while(0)
#endif

#ifdef CONFIG_SEC_DEBUG_TSP_LOG
/**
 * sec_debug_tsp_log : Leave tsp log in tsp_msg file.
 * ( Timestamp + Tsp logs )
 * sec_debug_tsp_log_msg : Leave tsp log in tsp_msg file and
 * add additional message between timestamp and tsp log.
 * ( Timestamp + additional Message + Tsp logs )
 */
extern void sec_debug_tsp_log(char *fmt, ...);
extern void sec_debug_tsp_log_msg(char *msg, char *fmt, ...);
#if defined(CONFIG_TOUCHSCREEN_FTS)
extern void tsp_dump(void);
#elif defined(CONFIG_TOUCHSCREEN_SEC_TS)
extern void tsp_dump_sec(void);
#endif

#else
#define sec_debug_tsp_log(a, ...)		do { } while(0)
#endif

#ifdef CONFIG_SEC_DEBUG_LAST_KMSG
extern void sec_debug_save_last_kmsg(unsigned char* head_ptr, unsigned char* curr_ptr, size_t buf_size);
#else
#define sec_debug_save_last_kmsg(a, b, c)		do { } while(0)
#endif

/* Last KMsg magickey */
#define SEC_LKMSG_MAGICKEY 0x0000000a6c6c7546

#ifdef CONFIG_SEC_PARAM
#define CM_OFFSET		0x700234
#define CM_OFFSET_LIMIT		1
#define GSP_OFFSET		0x700238
#define GSP_OFFSET_LIMIT 	0
int set_param(unsigned long offset, char val);
#endif

#endif /* SEC_DEBUG_H */
