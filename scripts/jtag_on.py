#!/bin/env python3

# Copyright (C) 2020 Swaraj Hota

from smbus import SMBus
from axiom_jtag import *

i2c = SMBus(3)
jtag = JTag(i2c, base=0x40, debug=True)
jtag.on()

i2c = SMBus(4)
jtag = JTag(i2c, base=0x40, debug=True)
jtag.on()

print("JTAG turned on for i2c3 and i2c4");
