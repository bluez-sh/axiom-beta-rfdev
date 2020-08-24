#!/bin/env python3

# Copyright (C) 2015 Herbert Poetzl
# Copyright (C) 2020 Swaraj Hota

import sys
from smbus import SMBus

i2c0 = SMBus(0)

ioa = i2c0.read_byte_data(0x23, 0x14)
i2c0.write_byte_data(0x23, 0x14, ioa&~0x10)
i2c0.write_byte_data(0x23, 0x14, ioa|0x10)
print("Pulled up reset for bus A")

iob = i2c0.read_byte_data(0x22, 0x14)
i2c0.write_byte_data(0x22, 0x14, iob&~0x10)
i2c0.write_byte_data(0x22, 0x14, iob|0x10)
print("Pulled up reset for bus B")
