// SPDX-License-Identifier: GPL-2.0
/*
 * Driver to Program/Debug Routing Fabrics in AXIOM Beta Main Board
 *
 * Copyright (C) 2020 Swaraj Hota <swarajhota353@gmail.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/fpga/fpga-mgr.h>
#include <crypto/internal/hash.h>

#include "rfdev.h"

#define DEV_NAME		"rfdev"
#define DIGEST_NAME		"md5"
#define DIGEST_SIZE		16
#define PIC_NUM_ADDRS		16
#define RF_MAX_TX_SIZE		33
#define RF_MAX_BSY_LOOP		128

/* Status register bits, errors and error mask */
#define BUSY	(31 - 12)
#define DONE	(31 - 8)
#define DVER	(31 - 27)
#define ENAB	(31 - 9)
#define FAIL	(31 - 13)
#define ERRBITS	(31 - 25)
#define ERRMASK	7

#define ENOERR	0 /* no error */
#define EID	1
#define ECMD	2
#define ECRC	3
#define EPREAM	4 /* preamble error */
#define EABRT	5 /* abort error */
#define EOVERFL	6 /* overflow error */
#define ESDMEOF	7 /* SDM EOF */

static struct fpga_manager *rfe_mgr;
static struct fpga_manager *rfw_mgr;

struct rfdev_client {
	struct i2c_client *client;
};

struct rfdev_device {
	unsigned char digest[DIGEST_SIZE];
	unsigned short int num_clients;
	struct rfdev_client client[];
};

struct sdesc {
	struct shash_desc shash;
	char ctx[];
};

static inline uint8_t get_err(unsigned long *status)
{
	return (*status >> ERRBITS) & ERRMASK;
}

static const char *get_err_string(uint8_t err)
{
	switch (err) {
	case ENOERR:	return "No Error";
	case EID:	return "ID ERR";
	case ECMD:	return "CMD ERR";
	case ECRC:	return "CRC ERR";
	case EPREAM:	return "Preamble ERR";
	case EABRT:	return "Abort ERR";
	case EOVERFL:	return "Overflow ERR";
	case ESDMEOF:	return "SDM EOF";
	}

	return "Default switch case";
}

static ssize_t parse_status_reg(unsigned long *status, char *buf)
{
	/* print debug if buf is NULL */
	if (!buf) {
		pr_debug("rf_status: 0x%08lx - done=%d, cfgena=%d, busy=%d, fail=%d, devver=%d, err=%s\n",
			*status, test_bit(DONE, status),
			test_bit(ENAB, status),
			test_bit(BUSY, status),
			test_bit(FAIL, status),
			test_bit(DVER, status),
			get_err_string(get_err(status)));
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE,
		"rf_status: 0x%08lx - done=%d, cfgena=%d, busy=%d, fail=%d, devver=%d, err=%s\n",
		*status, test_bit(DONE, status), test_bit(ENAB, status),
		test_bit(BUSY, status), test_bit(FAIL, status),
		test_bit(DVER, status), get_err_string(get_err(status)));
}

static int calc_hash(const unsigned char *data, size_t len,
		     unsigned char *digest)
{
	struct crypto_shash *alg;
	struct sdesc *sdesc;
	int size, ret;

	alg = crypto_alloc_shash(DIGEST_NAME, CRYPTO_ALG_TYPE_SHASH, 0);
	if (IS_ERR(alg)) {
		pr_info("can't allocate hash algorithm\n");
		return PTR_ERR(alg);
	}

	size = sizeof(struct shash_desc) + crypto_shash_descsize(alg);
	sdesc = kmalloc(size, GFP_KERNEL);
	if (!sdesc) {
		pr_info("can't allocate sdesc\n");
		return -ENOMEM;
	}
	sdesc->shash.tfm = alg;

	ret = crypto_shash_digest(&sdesc->shash, data, len, digest);
	if (ret < 0)
		pr_info("can't calculate digest\n");

	kfree(sdesc);
	crypto_free_shash(alg);
	return ret;
}

static unsigned char rev_byte(unsigned char b)
{
	b = (b & 0xf0) >> 4 | (b & 0x0f) << 4;
	b = (b & 0xcc) >> 2 | (b & 0x33) << 2;
	b = (b & 0xaa) >> 1 | (b & 0x55) << 1;
	return b;
}

static struct i2c_client *get_i2c_client(struct rfdev_device *rfdev,
					 unsigned int pic_opr)
{
	return rfdev->client[pic_opr].client;
};

static int i2c_pic_read(struct rfdev_device *rfdev,
			enum i2c_client_read_opr opr)
{
	int ret;

	ret = i2c_smbus_read_byte(get_i2c_client(rfdev, opr));
	pr_debug("smbus_read %02X -> %02X\n",
			get_i2c_client(rfdev, opr)->addr, ret);
	return ret;
}

static int i2c_pic_write(struct rfdev_device *rfdev,
			 enum i2c_client_write_opr opr,
			 uint8_t data,
			 uint8_t num_bits)
{
	int ret;

	if (!num_bits) {
		pr_debug("smbus_write %02X <- %02X\n",
				get_i2c_client(rfdev, opr)->addr, data);
		ret = i2c_smbus_write_byte(get_i2c_client(rfdev, opr), data);
	} else {
		pr_debug("smbus_write %02X <- %02X %02X\n",
				get_i2c_client(rfdev, opr)->addr,
				num_bits, data);
		ret = i2c_smbus_write_byte_data(
				get_i2c_client(rfdev, opr), num_bits, data);
	}
	return ret;
}

static int i2c_pic_write_block(struct rfdev_device *rfdev,
			       enum i2c_client_write_opr opr,
			       uint8_t cmd, uint8_t len,
			       const uint8_t *data)
{
	char data_str[RF_MAX_TX_SIZE * 3 + 2];
	unsigned int i, ptr = 0;

	for (i = 0; i < len; i++)
		ptr += scnprintf(data_str + ptr,
				sizeof(data_str) - ptr, "%02X ", data[i]);
	if (ptr > 0)
		data_str[ptr - 1] = '\0';

	pr_debug("smbus_write %02X <- %02X %s\n",
			get_i2c_client(rfdev, opr)->addr, cmd, data_str);
	return i2c_smbus_write_i2c_block_data(
			get_i2c_client(rfdev, opr), cmd, len, data);
}

static int get_idcode(struct rfdev_device *rfdev, uint32_t *idcode)
{
	int i;

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4); // goto Shift-IR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_IDCODE, 0);
	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4); // goto Shift-DR

	*idcode = 0;
	for (i = 3; i >= 0; i--) {
		int val = i2c_pic_read(rfdev, PIC_RD_TDO_IN_CONT);

		if (val < 0)
			return val;
		*idcode |= (val << (i * 8));
	}

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b011, 3);  // goto Run-Test

	return 0;
}

static int get_status(struct rfdev_device *rfdev, unsigned long *status)
{
	int i;

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-IR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_LSC_READ_STATUS, 0);
	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-DR

	*status = 0;
	for (i = 3; i >= 0; i--) {
		int val = i2c_pic_read(rfdev, PIC_RD_TDO_IN_CONT);

		if (val < 0)
			return val;
		*status |= (val << (i * 8));
	}

	parse_status_reg(status, NULL);
	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b011, 3);   // goto Run-Test

	return 0;
}

static int wait_not_busy(struct rfdev_device *rfdev)
{
	long val;
	int loop = 0;

	do {
		i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN,
				0b0011, 4);	// goto Shift-IR
		i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_LSC_CHECK_BUSY, 0);
		i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN,
				0b0011, 4);	// goto Shift-DR

		val = i2c_pic_read(rfdev, PIC_RD_TDO_IN);
		if (val < 0)
			return val;
		if (++loop >= RF_MAX_BSY_LOOP)
			return -EBUSY;

		pr_debug("%s: received 0x%02x\n", __func__, val & 0xff);

		i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN,
				0b01, 2);	// goto Run-Test
	} while (test_bit(7, (unsigned long *) &val));

	return 0;
}

static void reset_fpga(struct rfdev_device *rfdev)
{
	unsigned long status;

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0x7f, 0);	      // goto Run-Test

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-IR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_LSC_REFRESH, 0);
	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-DR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, 0x00, 0);
	pr_debug("%s: sent command 0x%02x", __func__, RF_LSC_REFRESH);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0b01, 0);	      // goto Run-Test
	if (wait_not_busy(rfdev) < 0)
		goto fail;

	get_status(rfdev, &status);
	if (!test_bit(DONE, &status))
		goto fail;

	goto out;

fail:
	pr_err("%s: failed to reset fpga\n");
out:
	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0xff, 0);	      // goto Reset
}

static ssize_t idcode_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	uint32_t idcode;
	int err;

	client = dev_get_drvdata(dev);
	rfdev  = i2c_get_clientdata(client);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0x7f, 0);	// goto Run-Test
	err = get_idcode(rfdev, &idcode);
	if (err)
		return err;

	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", idcode);
}

static ssize_t rf_status_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	unsigned long status;
	int err;

	client = dev_get_drvdata(dev);
	rfdev  = i2c_get_clientdata(client);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0x7f, 0);	// goto Run-Test
	err = get_status(rfdev, &status);
	if (err)
		return err;

	return parse_status_reg(&status, buf);
}

static ssize_t digest_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	unsigned int i, ptr = 0;

	client = dev_get_drvdata(dev);
	rfdev  = i2c_get_clientdata(client);

	for (i = 0; i < DIGEST_SIZE; i++)
		ptr += scnprintf(buf + ptr,
				PAGE_SIZE - ptr, "%02x", rfdev->digest[i]);
	buf[ptr]     = '\n';
	buf[ptr + 1] = '\0';

	return ptr + 1;
}

static DEVICE_ATTR_RO(idcode);
static DEVICE_ATTR_RO(rf_status);
static DEVICE_ATTR_RO(digest);

static struct attribute *dev_attrs[] = {
	&dev_attr_idcode.attr,
	&dev_attr_rf_status.attr,
	&dev_attr_digest.attr,
	NULL,
};

static struct attribute_group dev_attr_group = {
	.attrs = dev_attrs,
};

static const struct attribute_group *rfdev_attr_groups[] = {
	&dev_attr_group,
	NULL,
};

static int rfdev_fpga_ops_config_init(struct fpga_manager *mgr,
				      struct fpga_image_info *info,
				      const char *buf, size_t count)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	unsigned long status;
	int err;

	pr_debug("%s: called\n", __func__);

	client = dev_get_drvdata(&mgr->dev);
	rfdev  = i2c_get_clientdata(client);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0x7f, 0);	      // goto Run-Test

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-IR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_ISC_ENABLE, 0);
	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-DR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, 0x00, 0);
	pr_debug("%s: sent command 0x%02x", __func__, RF_ISC_ENABLE);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0b01, 0);	      // goto Run-Test
	err = wait_not_busy(rfdev);
	if (err)
		goto err;

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-IR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_ISC_ERASE, 0);
	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-DR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, 0x01, 0);
	pr_debug("%s: sent command 0x%02x", __func__, RF_ISC_ERASE);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0b01, 0);	      // goto Run-Test
	err = wait_not_busy(rfdev);
	if (err)
		goto err;
	get_status(rfdev, &status);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-IR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_LSC_BITSTREAM_BURST, 0);
	pr_debug("%s: sent command 0x%02x", __func__, RF_LSC_BITSTREAM_BURST);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-DR

	return 0;

err:
	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0xff, 0);
	return err;
}

static int rfdev_fpga_ops_config_write(struct fpga_manager *mgr,
				       const char *buf, size_t count)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	unsigned char rbuf[RF_MAX_TX_SIZE];
	unsigned int c, i, idx;
	int err;

	client = dev_get_drvdata(&mgr->dev);
	rfdev  = i2c_get_clientdata(client);

	calc_hash(buf, count, rfdev->digest);

	for (i = 0; count > 0; count -= c, i += c) {
		c = min(RF_MAX_TX_SIZE, count);

		for (idx = 0; idx < c; idx++)
			rbuf[idx] = rev_byte(buf[i + idx]);

		if (c == 1)
			err = i2c_pic_write(rfdev, PIC_WR_TDI_OUT_CONT,
					rbuf[0], 0);
		else
			err = i2c_pic_write_block(rfdev, PIC_WR_TDI_OUT_CONT,
					rbuf[0], c - 1, &rbuf[1]);
		if (err)
			return err;
	}

	return 0;
}

static int rfdev_fpga_ops_config_complete(struct fpga_manager *mgr,
					  struct fpga_image_info *info)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	unsigned long status;
	int i;

	pr_debug("%s: called\n", __func__);

	client = dev_get_drvdata(&mgr->dev);
	rfdev  = i2c_get_clientdata(client);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b011, 3);   // goto Run-Test

	// Idle time
	for (i = 0; i < 16; i++)
		i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0x00, 0);

	get_status(rfdev, &status);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-IR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_ISC_DISABLE, 0);
	pr_debug("%s: sent command 0x%02x", __func__, RF_ISC_DISABLE);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0b01, 0);        // goto Run-Test

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN, 0b0011, 4);  // goto Shift-IR
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_BYPASS, 0);
	pr_debug("%s: sent command 0x%02x", __func__, RF_BYPASS);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0b01, 0);        // goto Run-Test

	get_status(rfdev, &status);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0xff, 0);	      // goto Reset

	return 0;
}

static enum fpga_mgr_states rfdev_fpga_ops_state(struct fpga_manager *mgr)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	unsigned long status;

	pr_debug("%s: called\n", __func__);

	client = dev_get_drvdata(&mgr->dev);
	rfdev  = i2c_get_clientdata(client);

	i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0x7f, 0);	      // goto Run-Test

	get_status(rfdev, &status);
	if (!test_bit(BUSY, &status) && test_bit(DONE, &status) &&
	    get_err(&status) == ENOERR)
		return FPGA_MGR_STATE_OPERATING;

	return FPGA_MGR_STATE_UNKNOWN;
}

static const struct fpga_manager_ops rfdev_fpga_ops = {
	.write_init		= rfdev_fpga_ops_config_init,
	.write			= rfdev_fpga_ops_config_write,
	.write_complete		= rfdev_fpga_ops_config_complete,
	.state			= rfdev_fpga_ops_state,
	.groups			= rfdev_attr_groups,
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

	pr_debug("%s: called\n", __func__);

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s: required i2c functionality is not supported\n",
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
		if (err)
			goto dummy_clean_out;
	}

	i2c_set_clientdata(client, rfdev);

	/* Test read/write from/to the PIC */
	i2c_pic_write(rfdev, PIC_WR_BUF_DATA, 0xaa, 0);
	val = i2c_pic_read(rfdev, PIC_RD_BUF_DATA);
	if (val < 0) {
		err = val;
		goto dummy_clean_out;
	}

	if ((val & 0xff) != 0xaa) {
		pr_err("%s: read/write test failed, received byte 0x%02x\n",
				__func__, val);
		err = -EIO;
		goto dummy_clean_out;
	}

	/* Create and register fpga managers */
	rfe_mgr = devm_fpga_mgr_create(dev, "RFEast MachXO2 FPGA Manager",
					&rfdev_fpga_ops, NULL);
	if (!rfe_mgr) {
		err = -ENOMEM;
		goto dummy_clean_out;
	}
	dev_set_drvdata(&rfe_mgr->dev, client);
	err = fpga_mgr_register(rfe_mgr);
	if (err) {
		pr_err("%s: unable to register FPGA manager\n", __func__);
		goto dummy_clean_out;
	}

	rfw_mgr = devm_fpga_mgr_create(dev, "RFWest MachXO2 FPGA Manager",
					&rfdev_fpga_ops, NULL);
	if (!rfw_mgr) {
		err = -ENOMEM;
		goto dummy_clean_out;
	}
	dev_set_drvdata(&rfw_mgr->dev, client);
	err = fpga_mgr_register(rfw_mgr);
	if (err) {
		pr_err("%s: unable to register FPGA manager\n", __func__);
		fpga_mgr_unregister(rfe_mgr);
		goto dummy_clean_out;
	}

	return 0;

dummy_clean_out:
	rfdev_remove_dummy_clients(rfdev);
	return err;
}

static int rfdev_remove(struct i2c_client *client)
{
	struct rfdev_device *rfdev = i2c_get_clientdata(client);

	pr_debug("%s: remove called\n", DEV_NAME);

	reset_fpga(rfdev);
	if (rfe_mgr)
		fpga_mgr_unregister(rfe_mgr);
	if (rfw_mgr)
		fpga_mgr_unregister(rfw_mgr);
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
	.probe	  = rfdev_probe,
	.remove   = rfdev_remove,
	.id_table = rfdev_idtable,
	.driver   = {
		.name = "rfdev",
	},
};

static int __init rfdev_init(void)
{
	pr_debug("%s: init\n", DEV_NAME);
	return i2c_add_driver(&rfdev_driver);
}

static void __exit rfdev_exit(void)
{
	pr_debug("%s: exit\n", DEV_NAME);
	i2c_del_driver(&rfdev_driver);
}

module_init(rfdev_init);
module_exit(rfdev_exit);

MODULE_DESCRIPTION("Driver to program/debug routing fabrics in AXIOM Beta");
MODULE_AUTHOR("Swaraj Hota <swarajhota353@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
