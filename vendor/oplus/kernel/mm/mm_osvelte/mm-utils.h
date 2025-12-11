#ifndef _OSVELTE_MM_UTILS_H
#define _OSVELTE_MM_UTILS_H

#define MM_LOG_LVL 1
#define MM_LOG_TAG "mm"
enum {
	MM_LOG_VERBOSE = 0,
	MM_LOG_INFO,
	MM_LOG_DEBUG,
	MM_LOG_ERR,
};

static inline char mm_loglvl_to_char(int l)
{
	switch (l) {
	case MM_LOG_VERBOSE:
		return 'V';
	case MM_LOG_INFO:
		return 'I';
	case MM_LOG_DEBUG:
		return 'D';
	case MM_LOG_ERR:
		return 'E';
	}
	return '?';
}

#define mm_log(l, f, ...) do {						\
	if (l >= MM_LOG_LVL) 						\
		printk(KERN_ERR "%s %5d %5d %c %-16s: %s:%d "f,		\
		       MM_LOG_TAG, current->tgid, current->pid,		\
		       mm_loglvl_to_char(l), current->comm, __func__,	\
		       __LINE__,  ##__VA_ARGS__);			\
} while (0)

#define mm_loge(f, ...)							\
	mm_log(MM_LOG_ERR, f, ##__VA_ARGS__)

#define mm_logi(f, ...)							\
	mm_log(MM_LOG_INFO, f, ##__VA_ARGS__)

#define mm_logd(f, ...)							\
	mm_log(MM_LOG_DEBUG, f, ##__VA_ARGS__)
#endif /* _OSVELTE_MM_UTILS_H */
