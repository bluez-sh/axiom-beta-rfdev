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

static const unsigned short int pic_num_addrs = 16;

struct rfdev_client {
        struct i2c_client *client;
};

struct rfdev_device {
        unsigned short int num_clients;
        struct rfdev_client client[];
};

static int rfdev_make_dummy_client(struct rfdev_device *rfdev, 
                                   unsigned int index)
{
        struct i2c_client *base_client, *dummy_client;
        struct device *dev;
        unsigned short int addr;

        base_client = rfdev->client[0].client;
        dev = &base_client->dev;
        addr = base_client->addr + index;

        dummy_client = i2c_new_dummy(base_client->adapter, addr);
        if (!dummy_client) {
                dev_err(dev, "address 0x%02x unavailable\n", addr);
                return -EADDRINUSE;
        }

        rfdev->client[rfdev->num_clients++].client = dummy_client;
        return 0;
}

static void rfdev_remove_dummy_clients(struct rfdev_device *rfdev)
{
        int i;

        for (i = 1; i < rfdev->num_clients; i++)
                i2c_unregister_device(rfdev->client[i].client);
}

static int rfdev_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
        struct device *dev = &client->dev;
        struct rfdev_device *rfdev;
        size_t rfdev_size;
        unsigned int i;
        int err;

        printk(KERN_DEBUG "%s: probe called\n", DEV_NAME);

        if (!i2c_check_functionality(client->adapter,
                I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
                I2C_FUNC_SMBUS_I2C_BLOCK)) {
                printk(KERN_ERR
                        "%s: required i2c functionality is not supported\n",
                        __func__);
                return -ENODEV;
        }

        rfdev_size = sizeof(*rfdev) + pic_num_addrs * sizeof(struct rfdev_client);
        rfdev = devm_kzalloc(dev, rfdev_size, GFP_KERNEL);
        if (!rfdev)
                return -ENOMEM;

        rfdev->client[0].client = client;
        rfdev->num_clients = 1;

        for (i = 1; i < pic_num_addrs; i++) {
                err = rfdev_make_dummy_client(rfdev, i);
                if (err) {
                        rfdev_remove_dummy_clients(rfdev);
                        return err;
                }
        }

        i2c_set_clientdata(client, rfdev);

        return 0;
}

static int rfdev_remove(struct i2c_client *client)
{
        struct rfdev_device *rfdev = i2c_get_clientdata(client);

        printk(KERN_DEBUG "%s: remove called\n", DEV_NAME);
        rfdev_remove_dummy_clients(rfdev);

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
