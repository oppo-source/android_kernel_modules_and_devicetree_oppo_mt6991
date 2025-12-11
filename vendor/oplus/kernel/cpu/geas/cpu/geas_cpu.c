/*
 * a simple kernel module: powermodel_proc
 */
#include <linux/init.h>
#include <linux/module.h>

void enable_geas_cpu_periodly_running(void)
{


}

void disable_geas_cpu_periodly_running(void)
{


}

void geas_vote_cpu_for_frame(void)
{

}

void geas_vote_cpu_for_timer(void)
{

}

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