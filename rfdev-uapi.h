/* SPDX-License-Identifier: GPL-2.0 */
/*
 * User API Header File for Routing Fabrics in AXIOM Beta Main Board
 * Defines JTAG interface for Lattice MachXO2 FPGA
 *
 * Copyright (C) 2020 Swaraj Hota <swarajhota353@gmail.com>
 */

#ifndef RFDEV_UAPI_H
#define RFDEV_UAPI_H

#include <linux/types.h>

#define JTAG_MAX_XFER_DATA_LEN 65535

enum jtag_endstate {
	JTAG_STATE_TLRESET = 0,
	JTAG_STATE_IDLE,
	JTAG_STATE_SELECTDR,
	JTAG_STATE_CAPTUREDR,
	JTAG_STATE_SHIFTDR,
	JTAG_STATE_EXIT1DR,
	JTAG_STATE_PAUSEDR,
	JTAG_STATE_EXIT2DR,
	JTAG_STATE_UPDATEDR,
	JTAG_STATE_SELECTIR,
	JTAG_STATE_CAPTUREIR,
	JTAG_STATE_SHIFTIR,
	JTAG_STATE_EXIT1IR,
	JTAG_STATE_PAUSEIR,
	JTAG_STATE_EXIT2IR,
	JTAG_STATE_UPDATEIR,
};

enum jtag_reset {
	JTAG_NO_RESET = 0,
	JTAG_FORCE_RESET = 1,
};

enum jtag_xfer_type {
	JTAG_SIR_XFER = 0,
	JTAG_SDR_XFER = 1,
};

enum jtag_xfer_direction {
	JTAG_READ_XFER = 1,
	JTAG_WRITE_XFER = 2,
	JTAG_READ_WRITE_XFER = 3,
};

struct jtag_end_tap_state {
	__u8	reset;
	__u8	endstate;
	__u8	tck;
};

struct jtag_xfer {
	__u8	type;
	__u8	direction;
	__u8	endstate;
	__u8	padding;
	__u32	length;
	__u64	tdio;
};

/* ioctl interface */
#define __RFDEV_IOC_MAGIC '['
#define JTAG_SIOCSTATE	_IOW(__RFDEV_IOC_MAGIC, 0, struct jtag_end_tap_state)
#define JTAG_GIOCENDSTATE _IOR(__RFDEV_IOC_MAGIC, 1, enum jtag_endstate)
#define JTAG_IOCXFER	_IOWR(__RFDEV_IOC_MAGIC, 2, struct jtag_xfer)

#endif
