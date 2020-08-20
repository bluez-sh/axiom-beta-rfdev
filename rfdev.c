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
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <crypto/internal/hash.h>

#include "rfdev.h"
#include "rfdev-uapi.h"

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
static struct miscdevice rfe_misc;
static struct miscdevice rfw_misc;

struct rfdev_client {
	struct i2c_client *client;
};

struct rfdev_device {
	uint8_t digest[DIGEST_SIZE];
	uint8_t tap_state;
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
		pr_debug("status: 0x%08lx - done=%d, cfgena=%d, busy=%d, fail=%d, devver=%d, err=%s\n",
			*status, test_bit(DONE, status),
			test_bit(ENAB, status),
			test_bit(BUSY, status),
			test_bit(FAIL, status),
			test_bit(DVER, status),
			get_err_string(get_err(status)));
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE,
		"done=%d, cfgena=%d, busy=%d, fail=%d, devver=%d, err=%s\n",
		test_bit(DONE, status), test_bit(ENAB, status),
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
	int ret;

	for (i = 0; i < len; i++)
		ptr += scnprintf(data_str + ptr,
				sizeof(data_str) - ptr, "%02X ", data[i]);
	if (ptr > 0)
		data_str[ptr - 1] = '\0';
	else
		data_str[0] = '\0';

	pr_debug("smbus_write %02X <- %02X %s\n",
			get_i2c_client(rfdev, opr)->addr, cmd, data_str);
	if (!len)
		ret = i2c_smbus_write_byte(get_i2c_client(rfdev, opr), cmd);
	else
		ret = i2c_smbus_write_i2c_block_data(
				get_i2c_client(rfdev, opr), cmd, len, data);
	return ret;
}

static int tap_advance(struct rfdev_device *rfdev, enum jtag_endstate endstate)
{
	struct jtag_path path;
	int err;

	if (rfdev->tap_state > JTAG_STATE_UPDATEIR) {
		err = i2c_pic_write(rfdev, PIC_WR_TMS_OUT, 0xff, 0);
		if (err)
			return err;
		rfdev->tap_state = JTAG_STATE_TLRESET;
	}

	path = path_table[rfdev->tap_state][endstate];
	if (path.len == 8)
		err = i2c_pic_write(rfdev, PIC_WR_TMS_OUT, path.seq, 0);
	else
		err = i2c_pic_write(rfdev, PIC_WR_TMS_OUT_LEN,
				path.seq, path.len);
	if (err)
		return err;
	rfdev->tap_state = endstate;
	return 0;
}

static int tap_advance_idle(struct rfdev_device *rfdev, int count)
{
	do {
		tap_advance(rfdev, JTAG_STATE_IDLE);
	} while (count-- > 0);
}

static int rf_cmd_in(struct rfdev_device *rfdev,
		     enum rf_jtag_cmd cmd,
		     const uint8_t *op, int num_op)
{
	tap_advance(rfdev, JTAG_STATE_SHIFTIR);
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, cmd, 0);
	rfdev->tap_state = JTAG_STATE_EXIT1IR;
	pr_debug("%s: sent command 0x%02x", __func__, cmd);

	if (!num_op || !op) {
		tap_advance_idle(rfdev, 4);
		return 0;
	}

	tap_advance(rfdev, JTAG_STATE_SHIFTDR);
	while (num_op-- > 1)
		i2c_pic_write(rfdev, PIC_WR_TDI_OUT_CONT, op[num_op], 0);

	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, op[0], 0);
	rfdev->tap_state = JTAG_STATE_EXIT1IR;

	tap_advance_idle(rfdev, 4);
	return 0;
}

static int rf_cmd_out(struct rfdev_device *rfdev,
		      enum rf_jtag_cmd cmd,
		      uint64_t *val, int num_bytes)
{
	tap_advance(rfdev, JTAG_STATE_SHIFTIR);
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, cmd, 0);
	rfdev->tap_state = JTAG_STATE_EXIT1IR;
	pr_debug("%s: sent command 0x%02x", __func__, cmd);

	tap_advance(rfdev, JTAG_STATE_SHIFTDR);

	*val = 0;
	while (num_bytes-- > 0) {
		int byte;

		if (!num_bytes)
			byte = i2c_pic_read(rfdev, PIC_RD_TDO_IN);
		else
			byte = i2c_pic_read(rfdev, PIC_RD_TDO_IN_CONT);
		if (byte < 0)
			return byte;
		*val = (*val << 8) | byte;
	}
	rfdev->tap_state = JTAG_STATE_EXIT1IR;
	tap_advance_idle(rfdev, 4);
	return 0;
}

static int rf_tdo_in(struct rfdev_device *rfdev,
		     uint8_t *data, size_t size)
{
	int i, byte;

	for (i = 0; i < size; i++) {
		if (i == size - 1)
			byte = i2c_pic_read(rfdev, PIC_RD_TDO_IN);
		else
			byte = i2c_pic_read(rfdev, PIC_RD_TDO_IN_CONT);
		if (byte < 0)
			return byte;
		data[i] = byte & 0xff;
	}
	rfdev->tap_state = JTAG_STATE_EXIT1IR;
	return 0;
}

static int rf_tdi_out(struct rfdev_device *rfdev,
		      const uint8_t *data, size_t size,
		      size_t num_bits)
{
	unsigned char rbuf[RF_MAX_TX_SIZE];
	unsigned int c, i, idx, rem_bits;
	int err;

	rem_bits = BITS_PER_BYTE * size - num_bits;

	for (i = 0; size > 0; size -= c, i += c) {
		c = min(RF_MAX_TX_SIZE, size);

		for (idx = 0; idx < c; idx++)
			rbuf[idx] = rev_byte(data[i + idx]);

		if (c == 1) {
			/* handle the last byte */
			if (rem_bits)
				err = i2c_pic_write(rfdev, PIC_WR_TDI_OUT_LEN,
						rbuf[0], rem_bits);
			else
				err = i2c_pic_write(rfdev, PIC_WR_TDI_OUT,
						rbuf[0], 0);
		} else if (size <= RF_MAX_TX_SIZE) {
			/* leave out the last byte for last transfer */
			err = i2c_pic_write_block(rfdev, PIC_WR_TDI_OUT_CONT,
					rbuf[0], c - 2, &rbuf[1]);
			c--;
		} else {
			err = i2c_pic_write_block(rfdev, PIC_WR_TDI_OUT_CONT,
					rbuf[0], c - 1, &rbuf[1]);
		}
		if (err)
			return err;
	}
	rfdev->tap_state = JTAG_STATE_EXIT1IR;
	return 0;
}

static int get_status(struct rfdev_device *rfdev, unsigned long *status)
{
	uint64_t tmp;
	int err;

	err = rf_cmd_out(rfdev, RF_LSC_READ_STATUS, &tmp, 4);
	if (err)
		return err;
	*status = (uint32_t) tmp;
	parse_status_reg(status, NULL);
	return 0;
}

static int wait_not_busy(struct rfdev_device *rfdev)
{
	uint64_t val;
	int loop = 0;
	int err;

	do {
		err = rf_cmd_out(rfdev, RF_LSC_CHECK_BUSY, &val, 1);
		if (err)
			return err;
		pr_debug("%s: received 0x%02x\n", __func__, (uint8_t) val);

		if (++loop >= RF_MAX_BSY_LOOP)
			return -EBUSY;
	} while (test_bit(7, (unsigned long *) &val));

	return 0;
}

static void reset_fpga(struct rfdev_device *rfdev)
{
	unsigned long status;

	rf_cmd_in(rfdev, RF_LSC_REFRESH, (uint8_t []) {0x00}, 1);
	if (wait_not_busy(rfdev) < 0)
		goto fail;

	get_status(rfdev, &status);
	// TODO: find a way to check if reset was successful
	/*if (!test_bit(DONE, &status))*/
		/*goto fail;*/

	goto out;

fail:
	pr_err("%s: failed to reset fpga\n", __func__);
out:
	tap_advance(rfdev, JTAG_STATE_TLRESET);
}

static ssize_t idcode_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	uint64_t idcode;
	int err;

	client = dev_get_drvdata(dev);
	rfdev  = i2c_get_clientdata(client);

	err = rf_cmd_out(rfdev, RF_IDCODE, &idcode, 4);
	if (err)
		return err;

	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", (uint32_t) idcode);
}

static ssize_t stat_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	unsigned long status;
	int err;

	pr_debug("%s: called\n", __func__);

	client = dev_get_drvdata(dev);
	rfdev  = i2c_get_clientdata(client);

	err = get_status(rfdev, &status);
	if (err)
		return err;

	return scnprintf(buf, PAGE_SIZE, "0x%08lx\n", status);
}

static ssize_t statstr_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	unsigned long status;
	int err;

	client = dev_get_drvdata(dev);
	rfdev  = i2c_get_clientdata(client);

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

static ssize_t traceid_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	uint64_t traceid;
	int err;

	client = dev_get_drvdata(dev);
	rfdev  = i2c_get_clientdata(client);

	err = rf_cmd_out(rfdev, RF_UIDCODE_PUB, &traceid, 8);
	if (err)
		return err;

	return scnprintf(buf, PAGE_SIZE, "0x%016llx\n", traceid);
}

static ssize_t usercode_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	uint64_t usercode;
	int err;

	client = dev_get_drvdata(dev);
	rfdev  = i2c_get_clientdata(client);

	err = rf_cmd_out(rfdev, RF_USERCODE, &usercode, 4);
	if (err)
		return err;

	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", (uint32_t) usercode);
}

static DEVICE_ATTR_RO(idcode);
static DEVICE_ATTR_RO(stat);
static DEVICE_ATTR_RO(statstr);
static DEVICE_ATTR_RO(digest);
static DEVICE_ATTR_RO(traceid);
static DEVICE_ATTR_RO(usercode);

static struct attribute *dev_attrs[] = {
	&dev_attr_idcode.attr,
	&dev_attr_stat.attr,
	&dev_attr_statstr.attr,
	&dev_attr_digest.attr,
	&dev_attr_traceid.attr,
	&dev_attr_usercode.attr,
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

	rf_cmd_in(rfdev, RF_ISC_ENABLE, (uint8_t []) {0x00}, 1);
	err = wait_not_busy(rfdev);
	if (err)
		goto err;

	rf_cmd_in(rfdev, RF_ISC_ERASE,  (uint8_t []) {0x01}, 1);
	err = wait_not_busy(rfdev);
	if (err)
		goto err;

	get_status(rfdev, &status);

	tap_advance(rfdev, JTAG_STATE_SHIFTIR);
	i2c_pic_write(rfdev, PIC_WR_TDI_OUT, RF_LSC_BITSTREAM_BURST, 0);
	rfdev->tap_state = JTAG_STATE_EXIT1IR;
	pr_debug("%s: sent command 0x%02x", __func__, RF_LSC_BITSTREAM_BURST);

	tap_advance(rfdev, JTAG_STATE_SHIFTDR);
	return 0;

err:
	tap_advance(rfdev, JTAG_STATE_TLRESET);
	return err;
}

static int rfdev_fpga_ops_config_write(struct fpga_manager *mgr,
				       const char *buf, size_t count)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;

	client = dev_get_drvdata(&mgr->dev);
	rfdev  = i2c_get_clientdata(client);

	calc_hash(buf, count, rfdev->digest);
	return rf_tdi_out(rfdev, buf, count, BITS_PER_BYTE * count);
}

static int rfdev_fpga_ops_config_complete(struct fpga_manager *mgr,
					  struct fpga_image_info *info)
{
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	unsigned long status;

	pr_debug("%s: called\n", __func__);

	client = dev_get_drvdata(&mgr->dev);
	rfdev  = i2c_get_clientdata(client);

	tap_advance_idle(rfdev, 100);
	get_status(rfdev, &status);

	rf_cmd_in(rfdev, RF_ISC_DISABLE, NULL, 0);
	rf_cmd_in(rfdev, RF_BYPASS,	 NULL, 0);

	get_status(rfdev, &status);

	tap_advance(rfdev, JTAG_STATE_TLRESET);
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

static int rf_jtag_xfer(struct rfdev_device *rfdev,
			struct jtag_xfer *xfer,
			uint8_t *data, size_t size)
{
	int ret;

	if (xfer->direction == JTAG_WRITE_XFER) {
		if (xfer->type == JTAG_SDR_XFER)
			tap_advance(rfdev, JTAG_STATE_SHIFTDR);
		else if (xfer->type == JTAG_SIR_XFER)
			tap_advance(rfdev, JTAG_STATE_SHIFTIR);
		else
			return -EINVAL;

		ret = rf_tdi_out(rfdev, data, size, xfer->length);
	} else if (xfer->direction == JTAG_READ_XFER) {
		if (xfer->type == JTAG_SDR_XFER)
			tap_advance(rfdev, JTAG_STATE_SHIFTDR);
		else if (xfer->type == JTAG_SIR_XFER)
			tap_advance(rfdev, JTAG_STATE_SHIFTIR);
		else
			return -EINVAL;

		ret = rf_tdo_in(rfdev, data, size);
	} else {
		pr_err("xfer direction: both read-write not implemented\n");
		return -EINVAL;
	}

	tap_advance(rfdev, xfer->endstate);
	return ret;
}

static int rfdev_open(struct inode *inode, struct file *file)
{
	if (MINOR(inode->i_rdev) == (unsigned int) rfw_misc.minor)
		file->private_data = &rfw_mgr->dev;
	else if (MINOR(inode->i_rdev) == (unsigned int) rfe_misc.minor)
		file->private_data = &rfe_mgr->dev;
	else
		return -ENODEV;
	return nonseekable_open(inode, file);
}

static long rfdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct device *dev = file->private_data;
	struct i2c_client *client;
	struct rfdev_device *rfdev;
	struct jtag_end_tap_state endstate;
	struct jtag_xfer xfer;
	uint8_t *xfer_data;
	size_t data_size;
	int err = 0;

	client = dev_get_drvdata(dev);
	rfdev  = i2c_get_clientdata(client);

	if (!arg)
		return -EINVAL;

	switch (cmd) {
	case JTAG_GIOCENDSTATE:
		err = put_user(rfdev->tap_state, (uint32_t __user *)arg);
		break;
	case JTAG_SIOCSTATE:
		if (copy_from_user(&endstate, (const void __user *)arg,
					sizeof(struct jtag_end_tap_state)))
			return -EFAULT;

		if (endstate.endstate > JTAG_STATE_UPDATEIR)
			return -EINVAL;

		if (endstate.reset > JTAG_FORCE_RESET)
			return -EINVAL;

		if (endstate.reset == JTAG_FORCE_RESET)
			err = tap_advance(rfdev, JTAG_STATE_TLRESET);

		do {
			err = tap_advance(rfdev, endstate.endstate);
		} while (endstate.tck-- > 0);
		break;
	case JTAG_IOCXFER:
		if (copy_from_user(&xfer, (const void __user *)arg,
					sizeof(struct jtag_xfer)))
			return -EFAULT;

		if (xfer.length >= JTAG_MAX_XFER_DATA_LEN)
			return -EINVAL;

		if (xfer.direction > JTAG_READ_WRITE_XFER)
			return -EINVAL;

		if (xfer.endstate > JTAG_STATE_UPDATEIR)
			return -EINVAL;

		data_size = DIV_ROUND_UP(xfer.length, BITS_PER_BYTE);
		xfer_data = memdup_user(u64_to_user_ptr(xfer.tdio), data_size);
		if (IS_ERR(xfer_data))
			return -EFAULT;

		err = rf_jtag_xfer(rfdev, &xfer, xfer_data, data_size);
		if (err) {
			kfree(xfer_data);
			return err;
		}

		err = copy_to_user(u64_to_user_ptr(xfer.tdio),
					(void *)xfer_data, data_size);
		kfree(xfer_data);
		if (err)
			return -EFAULT;

		if (copy_to_user((void __user *)arg, (void *)&xfer,
					sizeof(struct jtag_xfer)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}
	return err;
}

static const struct file_operations rfdev_fops = {
	.owner			= THIS_MODULE,
	.open			= rfdev_open,
	.unlocked_ioctl		= rfdev_ioctl,
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
	rfdev->tap_state = 0xff;	// unknown state

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

	/* Register fpga managers and miscdevices */
	if (!id->driver_data) {
		rfw_mgr = devm_fpga_mgr_create(dev,
					"RFWest MachXO2 FPGA Manager",
					&rfdev_fpga_ops, NULL);
		if (!rfw_mgr) {
			err = -ENOMEM;
			goto dummy_clean_out;
		}
		dev_set_drvdata(&rfw_mgr->dev, client);
		err = fpga_mgr_register(rfw_mgr);
		if (err) {
			pr_err("%s: unable to register RFWest FPGA manager\n",
					__func__);
			goto dummy_clean_out;
		}

		rfw_misc.minor = MISC_DYNAMIC_MINOR;
		rfw_misc.name  = "rfw";
		rfw_misc.fops  = &rfdev_fops;
		err = misc_register(&rfw_misc);
		if (err) {
			pr_err("%s: can't register rfw miscdevice", __func__);
			fpga_mgr_unregister(rfw_mgr);
			goto dummy_clean_out;
		}

	} else {
		rfe_mgr = devm_fpga_mgr_create(dev,
					"RFEast MachXO2 FPGA Manager",
					&rfdev_fpga_ops, NULL);
		if (!rfe_mgr) {
			err = -ENOMEM;
			goto dummy_clean_out;
		}
		dev_set_drvdata(&rfe_mgr->dev, client);
		err = fpga_mgr_register(rfe_mgr);
		if (err) {
			pr_err("%s: unable to register RFEast FPGA manager\n",
					__func__);
			goto dummy_clean_out;
		}

		rfe_misc.minor = MISC_DYNAMIC_MINOR;
		rfe_misc.name  = "rfe";
		rfe_misc.fops  = &rfdev_fops;
		err = misc_register(&rfe_misc);
		if (err) {
			pr_err("%s: can't register rfe miscdevice", __func__);
			fpga_mgr_unregister(rfe_mgr);
			goto dummy_clean_out;
		}
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
	if (rfw_mgr) {
		misc_deregister(&rfw_misc);
		fpga_mgr_unregister(rfw_mgr);
		rfw_mgr = NULL;
	}
	if (rfe_mgr) {
		misc_deregister(&rfe_misc);
		fpga_mgr_unregister(rfe_mgr);
		rfe_mgr = NULL;
	}
	rfdev_remove_dummy_clients(rfdev);

	return 0;
}

static const struct of_device_id rfdev_of_match[] = {
	{ .compatible = "apertus,pic-rfw-interface" },
	{ .compatible = "apertus,pic-rfe-interface" },
	{ },
};
MODULE_DEVICE_TABLE(of, rfdev_of_match);

static const struct i2c_device_id rfdev_idtable[] = {
	{ "pic-rfw-interface", 0 },
	{ "pic-rfe-interface", 1 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, rfdev_idtable);

static struct i2c_driver rfdev_driver = {
	.probe	  = rfdev_probe,
	.remove   = rfdev_remove,
	.id_table = rfdev_idtable,
	.driver   = {
		.name = "rfdev",
		.of_match_table = of_match_ptr(rfdev_of_match),
		.owner = THIS_MODULE,
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
