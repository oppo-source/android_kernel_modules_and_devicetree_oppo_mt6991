#include <linux/sched.h>
#include <../kernel/sched/sched.h>
#include "scx_minidump.h"

#ifdef CONFIG_ARCH_MEDIATEK
#if IS_ENABLED(CONFIG_MTK_AEE_IPANIC)
//#include "../../../drivers/misc/mediatek/aee/mrdump/mrdump_mini.h"
extern void oplus_mrdump_mini_add_misc(unsigned long addr, unsigned long size,
		unsigned long start, char *name);
#endif
#endif /* CONFIG_ARCH_MEDIATEK */


#ifdef CONFIG_ARCH_MEDIATEK
#if IS_ENABLED(CONFIG_MTK_AEE_IPANIC)
void hmbird_minidump_init(void)
{
	unsigned long vaddr = 0;
	unsigned long size = 0;
	struct hmbird_ops *hmbird_ops = get_hmbird_ops(this_rq());

	if (hmbird_ops && hmbird_ops->hmbird_get_md_info) {
		hmbird_ops->hmbird_get_md_info(&vaddr, &size);
	}
	if (vaddr) {
		oplus_mrdump_mini_add_misc(vaddr, size, 0, "load");
	}

	pr_info("hmbird_minidump_init.\n");
}
#else
void hmbird_minidump_init(void) {};
#endif /* CONFIG_MTK_AEE_IPANIC */
#else
void hmbird_minidump_init(void) {};
#endif /* CONFIG_ARCH_MEDIATEK */
