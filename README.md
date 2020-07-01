# axiom-beta-rfdev
A Linux Kernel Driver for programming/debugging the Lattice MachXO2 FPGAs (used as routing fabrics) in AXIOM Beta Main Board.
The driver integrates with Linux FPGA Manager Framework to allow easy programming of MachXO2's SRAM.

## Currently supported features
- Upload a compressed bitstream (produced by Lattice Diamod tools) into MachXO2's SRAM<br>
<code># echo bitstream.bit > /sys/class/fpga_manager/<i>fpga#</i>/firmware </code> where,<br>
  - <code><i>fpga#</i></code> could be <i>fpga0</i>, <i>fpga1</i> etc.
  - <code>bitstream.bit</code> must be in FPGA Manager's search path (<code>/lib/firmware</code> by default)
- sysfs entry to read out idcode <br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/idcode</code>
- sysfs entry to read out status register <br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/rf_status</code>

## Requires
A devicetree entry must be added to system i2c bus (2) in AXIOM Beta:
```
pic@40 {
    compatible = "apertus,pic-rf-interface";
};
```
