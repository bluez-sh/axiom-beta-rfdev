#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#define DEV_NAME "rfdev"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Swaraj Hota");
MODULE_DESCRIPTION("A character driver used to program/debug routing fabrics \
in AXIOM Beta main board");
MODULE_VERSION("0.1");

struct rfdev_device {
        struct i2c_client *client;
};

static int rfdev_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
        struct rfdev_device *dev;

        printk(KERN_DEBUG "%s: probe called\n", DEV_NAME);

        if (!i2c_check_functionality(client->adapter,
                I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
                I2C_FUNC_SMBUS_I2C_BLOCK)) {
                printk(KERN_ERR
                        "%s: required i2c functionality is not supported\n",
                        __func__);
                return -ENODEV;
        }

        dev = kzalloc(sizeof(struct rfdev_device), GFP_KERNEL);
        if (!dev) {
                printk(KERN_ERR "%s: no memory\n", __func__);
                return -ENOMEM;
        }

        dev->client = client;
        i2c_set_clientdata(client, dev);

        return 0;
}

static int rfdev_remove(struct i2c_client *client)
{
        struct rfdev_device *dev = i2c_get_clientdata(client);
        printk(KERN_DEBUG "%s: remove called\n", DEV_NAME);
        return 0;
}

static const struct of_device_id rfdev_of_match[] = {
        { .compatible = "apertus,pic-rf-interface" },
        { },
};
MODULE_DEVICE_TABLE(of, rfdev_of_match);

static const struct i2c_device_id rfdev_idtable[] = {
        { "pic-rf-interface", 0 },
        { },
};
MODULE_DEVICE_TABLE(i2c, rfdev_idtable);

static struct i2c_driver rfdev_driver = {
        .probe    = rfdev_probe,
        .remove   = rfdev_remove,
        .id_table = rfdev_idtable,
        .driver   = {
                .name = "rfdev",
        },
};

static int __init rfdev_init(void)
{
        printk(KERN_DEBUG "%s: init\n", DEV_NAME);
        return i2c_add_driver(&rfdev_driver);
}

static void __exit rfdev_exit(void)
{
        printk(KERN_DEBUG "%s: exit\n", DEV_NAME);
        i2c_del_driver(&rfdev_driver);
}

module_init(rfdev_init);
module_exit(rfdev_exit);
