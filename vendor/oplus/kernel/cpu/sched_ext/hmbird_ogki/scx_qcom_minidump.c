#include <linux/sched.h>
#include <../kernel/sched/sched.h>
#include "scx_minidump.h"
#include "../hmbird_version.h"

#if IS_ENABLED(CONFIG_QCOM_MINIDUMP)
#include <soc/qcom/minidump.h>
#endif

#if IS_ENABLED(CONFIG_QCOM_MINIDUMP)
void hmbird_minidump_init(void)
{
        unsigned long vaddr = 0;
        unsigned long size = 0;
        struct md_region md_entry;
        int ret;
        struct hmbird_ops *hmbird_ops = get_hmbird_ops(this_rq());

	if (HMBIRD_OGKI_VERSION == get_hmbird_version_type()){
	        if (hmbird_ops && hmbird_ops->hmbird_get_md_info) {
        	        hmbird_ops->hmbird_get_md_info(&vaddr, &size);
        	}

        	scnprintf(md_entry.name, sizeof(md_entry.name), "md_hmbird");
        	md_entry.virt_addr = vaddr;
        	md_entry.phys_addr = virt_to_phys((void *)vaddr);
        	md_entry.size = size;
        	ret = msm_minidump_add_region(&md_entry);
        	if (ret < 0) {
                	pr_err("Failed to add hmbird minidump region, err = %d\n", ret);
        	}
        	pr_info("hmbird_minidump_init.\n");
	}
	else
		return;
}
#else
void hmbird_minidump_init(void) {};
#endif /* CONFIG_QCOM_MINIDUMP */

