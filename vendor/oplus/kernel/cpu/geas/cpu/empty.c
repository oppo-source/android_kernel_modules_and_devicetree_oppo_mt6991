/*
 * a simple kernel module: powermodel_proc
 */
#include <linux/init.h>
#include <linux/module.h>

static int __init geas_cpu_init(void)
{
	return 0;
}

static void __exit geas_cpu_exit(void)
{

}

module_init(geas_cpu_init);
module_exit(geas_cpu_exit);


MODULE_LICENSE("GPL");