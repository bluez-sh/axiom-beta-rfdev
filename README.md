# axiom-beta-rfdev
A Linux Kernel Driver for programming/debugging the Lattice MachXO2 FPGAs (used as routing fabrics) in AXIOM Beta Main Board.
The driver integrates with Linux FPGA Manager Framework to allow easy programming of MachXO2's SRAM.

## Currently supported features
- Upload a compressed bitstream (produced by Lattice Diamod tools) into MachXO2's SRAM <br>
<code> cat bitstream.bit > /sys/class/fpga_manager/<i>fpga#</i>/firmware </code>, where <i>fpga#</i> could be <i>fpga0</i>, <i>fpga1</i> etc.
- sysfs entry to read out idcode <br>
<code> cat /sys/class/fpga_manager/<i>fpga#</i>/idcode </code>
- sysfs entry to read out status register <br>
<code> cat /sys/class/fpga_manager/<i>fpga#</i>/rf_status </code>
