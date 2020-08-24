/*
 * Sample code to upload bitstream using rfdev's uapi interface
 *
 * Copyright (C) 2020 Swaraj Hota <swarajhota353@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdint.h>

#include "rfdev-uapi.h"

// MachXO2 commands
#define BYPASS			0xFF
#define ISC_DISABLE		0x26
#define ISC_ENABLE		0xC6
#define ISC_ERASE		0x0E
#define LSC_BITSTREAM_BURST	0x7A
#define LSC_READ_STATUS		0x3C

#define BUF_SIZE 64

unsigned char buf[BUF_SIZE];
unsigned char *bitstream;

char *rfname;
char *bitfilename;

void exit_cleanup() {
	free(bitstream);
	exit(1);
}

unsigned char rev_byte(unsigned char b)
{
	b = (b & 0xf0) >> 4 | (b & 0x0f) << 4;
	b = (b & 0xcc) >> 2 | (b & 0x33) << 2;
	b = (b & 0xaa) >> 1 | (b & 0x55) << 1;
	return b;
}

void cmd_in(int fd, uint8_t cmd, uint8_t *op, int num_op)
{
	struct jtag_xfer xfer;
	int i;

	buf[0] = cmd;
	xfer.type	= JTAG_SIR_XFER;
	xfer.direction 	= JTAG_WRITE_XFER;
	xfer.endstate 	= JTAG_STATE_EXIT1IR;
	xfer.length 	= 8;
	xfer.tdio 	= (uint32_t) &buf;
	if (ioctl(fd, JTAG_IOCXFER, &xfer) < 0) {
		printf("failed xfer sir cmd: %d\n", cmd);
		exit_cleanup();
	}

	if (!num_op || !op)
		return;

	for (i = 0; i < num_op; i++)
		buf[num_op - i - 1] = op[i];

	xfer.type 	= JTAG_SDR_XFER;
	xfer.direction 	= JTAG_WRITE_XFER;
	xfer.endstate 	= JTAG_STATE_IDLE;
	xfer.length 	= num_op * 8;
	xfer.tdio 	= (uint32_t) &buf;
	if (ioctl(fd, JTAG_IOCXFER, &xfer) < 0) {
		printf("failed xfer sdr cmd: %d\n", cmd);
		exit_cleanup();
	}
}

uint64_t cmd_out(int fd, uint8_t cmd, int len)
{
	struct jtag_xfer xfer;
	uint64_t val = 0;
	int i;

	buf[0] = cmd;
	xfer.type	= JTAG_SIR_XFER;
	xfer.direction 	= JTAG_WRITE_XFER;
	xfer.endstate 	= JTAG_STATE_IDLE;
	xfer.length 	= 8;
	xfer.tdio 	= (uint32_t) &buf;
	if (ioctl(fd, JTAG_IOCXFER, &xfer) < 0) {
		printf("failed xfer sir cmd: %d\n", cmd);
		exit_cleanup();
	}

	xfer.type 	= JTAG_SDR_XFER;
	xfer.direction 	= JTAG_READ_XFER;
	xfer.endstate 	= JTAG_STATE_IDLE;
	xfer.length 	= len;
	xfer.tdio 	= (uint32_t) &buf;
	if (ioctl(fd, JTAG_IOCXFER, &xfer) < 0) {
		printf("failed xfer sdr cmd: %d\n", cmd);
		exit_cleanup();
	}

	for (i = 0; i < len / 8; i++)
		val = (val << 8) | buf[i];
	return val;
}

int main(int argc, char **argv)
{
	struct jtag_end_tap_state reset_idle = {
		JTAG_FORCE_RESET, JTAG_STATE_IDLE, 0,
	};
	struct jtag_end_tap_state idle = {
		JTAG_NO_RESET, JTAG_STATE_IDLE, 4,
	};
	struct jtag_xfer xfer;
	unsigned int c;
	long fsize;
	int fd, i;

	if (argc != 3) {
		printf("Usage: %s <dev_file> <bit_file>\n", argv[0]);
		exit(0);
	}
	
	rfname 	    = argv[1];
	bitfilename = argv[2];

	FILE *fp = fopen(bitfilename, "rb");
	if (!fp) {
		printf("cannot open: %s\n", bitfilename);
		exit_cleanup();
	}

	fseek(fp, 0, SEEK_END);
	fsize = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	bitstream = malloc(fsize * sizeof(unsigned char));
	if (!bitstream) {
		printf("cannot allocate memory for bitstream\n");
		exit(1);
	}

	fread(bitstream, sizeof(unsigned char), fsize, fp);
	fclose(fp);

	for (i = 0; i < fsize; i++)
		bitstream[i] = rev_byte(bitstream[i]);

	if (chmod(rfname, strtol("0666", 0, 8)) < 0) {
		printf("cannot chmod: %s\n", rfname);
		exit_cleanup();
	}

	fd = open(rfname, O_NONBLOCK);
	if (fd < 0) {
		printf("cannot open: %s\n", rfname);
		exit_cleanup();
	}

	if (ioctl(fd, JTAG_SIOCSTATE, &reset_idle) < 0) {
		printf("failed to set state reset_idle\n");
		exit_cleanup();
	}

	cmd_in(fd, ISC_ENABLE, (uint8_t []) {0x00}, 1);
	ioctl(fd, JTAG_SIOCSTATE, &idle);
	usleep(100000);

	cmd_in(fd, ISC_ERASE,  (uint8_t []) {0x01}, 1);
	ioctl(fd, JTAG_SIOCSTATE, &idle);
	usleep(100000);

	printf("status: 0x%08x\n", (uint32_t) cmd_out(fd, LSC_READ_STATUS, 32));

	cmd_in(fd, LSC_BITSTREAM_BURST, NULL, 0);

	for (i = 0; fsize > 0; fsize -= c, i += c) {
		c = BUF_SIZE < fsize ? BUF_SIZE : fsize;

		memcpy(buf, &bitstream[i], c);

		if (fsize <= BUF_SIZE)
			xfer.endstate = JTAG_STATE_IDLE;
		else
			xfer.endstate = JTAG_STATE_SHIFTDR;

		xfer.type 	= JTAG_SDR_XFER;
		xfer.direction 	= JTAG_WRITE_XFER;
		xfer.length 	= c * 8;
		xfer.tdio 	= (uint32_t) &buf;
		if (ioctl(fd, JTAG_IOCXFER, &xfer) < 0) {
			printf("failed xfer sdr bitstream\n");
			exit_cleanup();
		}
	}
	ioctl(fd, JTAG_SIOCSTATE, &idle);
	usleep(1000000);
	
	printf("status: 0x%08x\n", (uint32_t) cmd_out(fd, LSC_READ_STATUS, 32));

	cmd_in(fd, ISC_DISABLE, NULL, 0);
	ioctl(fd, JTAG_SIOCSTATE, &idle);
	cmd_in(fd, BYPASS,	NULL, 0);
	ioctl(fd, JTAG_SIOCSTATE, &idle);

	printf("status: 0x%08x\n", (uint32_t) cmd_out(fd, LSC_READ_STATUS, 32));

	close(fd);
	free(bitstream);
	return 0;
}
