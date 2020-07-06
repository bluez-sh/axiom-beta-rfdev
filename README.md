# axiom-beta-rfdev
A Linux Kernel Driver for programming/debugging the Lattice MachXO2 FPGAs (used as routing fabrics) in AXIOM Beta Main Board.
The driver integrates with Linux FPGA Manager Framework to allow easy programming of MachXO2's SRAM.<br><br>
Here is an overview of the setup:<br>
Zynq SoC <--- *I2C* ---> PIC16 MCU <--- *JTAG* ---> MachXO2 FPGA<br><br>
Linux runs on hardened ARM cores in Zynq SoC, which is where the driver comes in. PIC16 responds to a range of I2C commands and translates them into JTAG commands for MachXO2. 

## Currently supported features
- Upload a compressed bitstream (produced by Lattice Diamod tools) into MachXO2's SRAM<br>
<code># echo bitstream.bit > /sys/class/fpga_manager/<i>fpga#</i>/firmware </code> where,<br>
  - <code><i>fpga#</i></code> could be <i>fpga0</i>, <i>fpga1</i> etc.
  - <code>bitstream.bit</code> must be in FPGA Manager's search path (<code>/lib/firmware</code> by default)
- sysfs entry to read out idcode <br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/idcode</code>
- sysfs entry to read out status register <br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/rf_status</code>
- sysfs entry to show md5 digest of the uploaded bitstream<br>
<code>$ cat /sys/class/fpga_maanger/<i>fpga#</i>/digest</code>

## Requires
A devicetree entry must be added to system i2c bus (2) in AXIOM Beta:
```
pic@40 {
    compatible = "apertus,pic-rf-interface";
};
```
## Build Instructions
1. <code>git clone --recursive https://github.com/apertus-open-source-cinema/axiom-firmware</code>
2. <code>cd axiom-firmware</code>
3. <code>./makefiles/docker-make.sh build/linux-v5.2.14.git/arch/arm/boot/zImage</code> where, <code>5.2.14</code> should be replaced with latest kernel version used in AXIOM Beta (see here: <code>axiom-firmware/boot/kernel.config</code>). This command should build the kernel once.
4. <code>cd build</code>
5. <code>git clone https://github.com/Swaraj1998/axiom-beta-rfdev</code> (this repo)
6. <code>../makefiles/docker-make.sh build-shell</code> (get a shell inside the build container)
<br><br>Inside the build-shell:
7. <code>cd build/axiom-beta-rfdev</code> (you just cloned it)
8. <code>make</code> (or <code>make clean</code>)

Note: <code>make native</code> & <code>make clean_native</code> compiles against currently running kernel in your system (just used for testing build)
