#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "rfdev.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Swaraj Hota");
MODULE_DESCRIPTION("A character driver used to program/debug routing fabrics \
in AXIOM Beta main board");
MODULE_VERSION("0.1");

#define DEV_NAME "rfdev"
#define PIC_NUM_ADDRS 16

static struct device *root_dev = NULL;

struct rfdev_client {
        struct i2c_client *client;
};

struct rfdev_device {
        unsigned short int num_clients;
        struct rfdev_client client[];
};

static ssize_t rfdev_show_idcode(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
        return scnprintf(buf, 10, "test\n");
}

static DEVICE_ATTR(idcode, 0444, rfdev_show_idcode, NULL);

static struct attribute *dev_attrs[] = {
        &dev_attr_idcode.attr,
        NULL,
};

static struct attribute_group dev_attr_group = {
        .attrs = dev_attrs,
};

static struct i2c_client *get_i2c_client(struct rfdev_device *rfdev,
                                         unsigned int pic_opr)
{
        return rfdev->client[pic_opr].client;
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
        int err, val;

        printk(KERN_DEBUG "%s: probe called\n", DEV_NAME);

        if (!i2c_check_functionality(client->adapter,
                I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA |
                I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) {
                printk(KERN_ERR
                        "%s: required i2c functionality is not supported\n",
                        __func__);
                return -ENODEV;
        }

        rfdev_size = sizeof(*rfdev) +
                        PIC_NUM_ADDRS * sizeof(struct rfdev_client);
        rfdev = devm_kzalloc(dev, rfdev_size, GFP_KERNEL);
        if (!rfdev)
                return -ENOMEM;

        rfdev->client[0].client = client;
        rfdev->num_clients = 1;

        for (i = 1; i < PIC_NUM_ADDRS; i++) {
                err = rfdev_make_dummy_client(rfdev, i);
                if (err) {
                        rfdev_remove_dummy_clients(rfdev);
                        return err;
                }
        }

        i2c_set_clientdata(client, rfdev);

        /* Test read/write from/to the PIC */
        i2c_smbus_write_byte(get_i2c_client(rfdev, PIC_WR_BUF_DATA), 0xaa);
        val = i2c_smbus_read_byte(get_i2c_client(rfdev, PIC_RD_BUF_DATA));
        if (val < 0)
                return val;

        if ((val & 0xff) == 0xaa)
                printk(KERN_DEBUG "%s: read/write test passed\n", __func__);
        else
                printk(KERN_DEBUG "%s: read/write test failed, \
                                received byte 0x%02x\n", __func__, val);

        /* Create sysfs entries */
        root_dev = root_device_register(DEV_NAME);
        if (!dev)
                return -ENOMEM;

        err = sysfs_create_group(&root_dev->kobj, &dev_attr_group);
        if (err)
                printk(KERN_ERR "%s: sysfs_create_group failure\n", __func__);

        return 0;
}

static int rfdev_remove(struct i2c_client *client)
{
        struct rfdev_device *rfdev = i2c_get_clientdata(client);

        printk(KERN_DEBUG "%s: remove called\n", DEV_NAME);
        rfdev_remove_dummy_clients(rfdev);
        if (root_dev)
                root_device_unregister(root_dev);

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
