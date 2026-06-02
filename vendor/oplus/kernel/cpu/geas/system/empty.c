/*
 * a simple kernel module: powermodel_proc
 */
#include <linux/init.h>
#include <linux/module.h>

static int __init geas_system_init(void)
{
	return 0;
}

static void __exit geas_system_exit(void)
{

}

module_init(geas_system_init);
module_exit(geas_system_exit);


MODULE_LICENSE("GPL");