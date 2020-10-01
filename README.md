# axiom-beta-rfdev
> Note: This repository contains all the source code produced by me under GSoC 2020 with apertus° Association. For detailed project report, please read [GSoC2020_final_report.pdf](https://github.com/Swaraj1998/axiom-beta-rfdev/blob/master/GSoC2020_final_report.pdf) in this repository.<br>

A Linux Kernel Driver for programming/debugging the Lattice MachXO2 FPGAs (used as routing fabrics) in AXIOM Beta Main Board.
The driver integrates with Linux FPGA Manager Framework to allow easy programming of MachXO2's SRAM.<br><br>
Here is an overview of the setup:<br>
Zynq SoC <--- *I2C* ---> PIC16 MCU <--- *JTAG* ---> MachXO2 FPGA<br><br>
Linux runs on hardened ARM cores in Zynq SoC, which is where the driver comes in. PIC16 responds to a range of I2C commands and translates them into JTAG commands for MachXO2.

## Features
- Upload a compressed bitstream (produced by Lattice Diamod tools) into MachXO2's SRAM<br>
<code># echo bitstream.bit > /sys/class/fpga_manager/<i>fpga#</i>/firmware </code> where,<br>
  - <code><i>fpga#</i></code> could be <i>fpga0</i>, <i>fpga1</i> etc.
  - <code>bitstream.bit</code> must be in FPGA Manager's search path (<code>/lib/firmware</code> by default)
- sysfs entry to read out idcode <br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/idcode</code>
- sysfs entry to read out status register <br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/stat</code>
- sysfs entry to show human readable string for status<br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/statstr</code>
- sysfs entry to show md5 digest of the uploaded bitstream<br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/digest</code>
- sysfs entry to read out traceid (unique fpga identifier)<br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/traceid</code>
- sysfs entry to read out usercode<br>
<code>$ cat /sys/class/fpga_manager/<i>fpga#</i>/usercode</code>
- RFWest and RFEast can be used seperately for AXIOM Betas with both old and new power board versions
- A User API based on ioctl is added which exposes a JTAG interface from the FPGAs
  - Can be used by an application like OpenOCD to do SVF replays for programming or debugging<br>
  - The two devices can be accessed as: <code>/dev/rfjtag0</code> and <code>/dev/rfjtag1</code>
  - The code was made generic to allow registration of multiple MachXO2 devices in the setup (as */dev/rfjtag#*) and configurable through devicetree entries

Following ioctl commands are defined:<br>
```
JTAG_SIOCSTATE    -> transit to given tap state
JTAG_GIOCENDSTATE -> get current tap state
JTAG_IOCXFER      -> send SIR/SDR transaction
```

## Requirements
Appropriate devicetree overlay must be loaded in AXIOM Beta:<br>
- <code>dt-overlay/pic_rf_mux_overlay.dts</code> for power board with PCA9540 mux<br>
- <code>dt-overlay/pic_rf_mux_rst_overlay.dts</code> for power board with PCA9543 mux<br>
- <code>dt-overlay/pic_rf_switch_overlay.dts</code> for power board with TS3A4751 analog switch

Run the following scripts before loading the driver (in firmware 2.0):
```
# the scripts directory in this repo
cd scripts

# initialize power and load the icsp bitstream into microzed's fpga
sudo ./opt/axiom-firmware/software/scripts/axiom_prep_iscp.sh

# pull up the #MCLR lines of both the PIC16s
sudo python pull_up_reset.py

# load the appropriate device tree overlay
sudo ./load_overlay.sh ../dt-overlays/pic_rf_switch_overlay.dts

# configure the jtag out ports of both the PIC16s
sudo python jtag_on.py
```

## OpenOCD Usage
The patch for OpenOCD can be found inside <code>openocd-patch</code> directory in this repo.<br>
After applying the patch configure OpenOCD with <code>--enable-rfdev-jtag=yes</code> option and build.<br>
A sample .cfg file for OpenOCD then will look like this:
```
# select the driver
adapter driver rfdev_jtag

# specify the device file
rfdev_set_device /dev/rfjtag0

# speed must be specified but is not used
adapter speed 1
```

Then run with something like:<br>
<code>./openocd -f openocd.cfg -c init -c "svf count.svf"</code>

## Build Instructions
1. <code>git clone --recursive https://github.com/apertus-open-source-cinema/axiom-firmware</code>
2. <code>cd axiom-firmware</code>
3. <code>./makefiles/docker-make.sh build/linux-v5.2.14.git/arch/arm/boot/zImage</code> where, <code>5.2.14</code> should be replaced with latest kernel version used in AXIOM Beta (see here: <code>axiom-firmware/makefiles/host/bootfs.mk</code>). This command should build the kernel once.
4. <code>cd build</code>
5. <code>git clone https://github.com/Swaraj1998/axiom-beta-rfdev</code> (this repo)
6. <code>../makefiles/docker-make.sh build-shell</code> (get a shell inside the build container)
<br><br>Inside the build-shell:
7. <code>cd build/axiom-beta-rfdev</code> (you just cloned it)
8. <code>make</code> (or <code>make clean</code>)

Note: <code>make native</code> & <code>make clean_native</code> compiles against currently running kernel in your system (just used for testing build)
