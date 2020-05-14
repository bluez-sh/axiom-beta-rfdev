#include <linux/init.h>
#include <linux/module.h>

#define DEVICE_NAME "rfdev"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Swaraj Hota");
MODULE_DESCRIPTION("A character driver used to program/debug routing fabrics \
        in AXIOM Beta main board");
MODULE_VERSION("0.1");

static int __init rfdev_init(void)
{
    printk(KERN_DEBUG "rfdev: init\n");
    return 0;
}

static void __exit rfdev_exit(void)
{
    printk(KERN_DEBUG "rfdev: exit\n");
}

module_init(rfdev_init);
module_exit(rfdev_exit);
