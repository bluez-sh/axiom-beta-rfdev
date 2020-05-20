#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>

#define DEVICE_NAME "rfdev"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Swaraj Hota");
MODULE_DESCRIPTION("A character driver used to program/debug routing fabrics \
in AXIOM Beta main board");
MODULE_VERSION("0.1");

static struct i2c_board_info __initdata rfdev_i2c_board_info[] = {
    { I2C_BOARD_INFO("pic16", 0x40), },
    { }
};

static int __init rfdev_init(void)
{
    printk(KERN_DEBUG "rfdev: init\n");
    i2c_register_board_info(2, rfdev_i2c_board_info, 
                    ARRAY_SIZE(rfdev_i2c_board_info));
    return 0;
}

static void __exit rfdev_exit(void)
{
    printk(KERN_DEBUG "rfdev: exit\n");
}

module_init(rfdev_init);
module_exit(rfdev_exit);
