# Plugin esp32s3 and detect the correct USB Port

## Check esp32s3 in which port

Now, connect the ESP32 development board to your laptop or jetson and check which serial port the board is using.

Typically, the serial port name appears differently depending on the operating system:

**Linux**: starts with `/dev/tty` (For jetson or your linux laptop)

**macOS**: starts with `/dev/cu.` (For your personal macbook)
```
<team-user>@<jetson-host> test % ls -l /dev/cu*
crw-rw-rw-  1 root  wheel  0x9000001  3 Oct 13:43 /dev/cu.Bluetooth-Incoming-Port
crw-rw-rw-  1 root  wheel  0x9000003 20 Oct 20:33 /dev/cu.usbmodem1101
```

**if you are using macOS you only need to know your port is /dev/cu.usbmodem1101**


## Below is for jetson only

### Check lsusb see if we see ESP32s3 board connected
```
root@<jetson-host>:$HOME/esp/esp-idf# lsusb
Bus 002 Device 002: ID 0bda:0489 Realtek Semiconductor Corp. 4-Port USB 3.0 Hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 13d3:3549 IMC Networks Bluetooth Radio
Bus 001 Device 005: ID 093a:2521 Pixart Imaging, Inc. Optical Mouse
Bus 001 Device 004: ID 258a:0001 SINO WEALTH USB KEYBOARD
Bus 001 Device 002: ID 0bda:5489 Realtek Semiconductor Corp. 4-Port USB 2.0 Hub
Bus 001 Device 006: ID 303a:1001 **Espressif USB JTAG/serial debug unit**
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

### Below is our esp32s3
```
Bus 001 Device 006: ID 303a:1001 Espressif USB JTAG/serial debug unit
```

### use dmesg to confirm the esp32s3 device port name
```
root@<jetson-host>:$HOME/esp/esp-idf# dmesg | grep tty
[    0.000000] Kernel command line: root=/dev/mmcblk0p1 rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 firmware_class.path=/etc/firmware fbcon=map:0 nospectre_bhb video=efifb:off console=tty0 bl_prof_dataptr=2031616@0x271E10000 bl_prof_ro_ptr=65536@0x271E00000
[    0.000387] printk: console [tty0] enabled
[    0.069021] 31d0000.serial: ttyAMA0 at MMIO 0x31d0000 (irq = 117, base_baud = 0) is a SBSA
[    0.201220] printk: console [ttyTCU0] enabled
[    3.109580] printk: console [tty0]: printing thread started
[    3.109638] printk: console [ttyTCU0]: printing thread started
[    3.165632] 3100000.serial: ttyTHS1 at MMIO 0x3100000 (irq = 112, base_baud = 0) is a TEGRA_UART
[    3.166189] 3140000.serial: ttyTHS2 at MMIO 0x3140000 (irq = 113, base_baud = 0) is a TEGRA_UART
[    8.703407] systemd[1]: Created slice Slice /system/serial-getty.
[ 2770.910658] cdc_acm 1-1:1.0: **ttyACM0: USB ACM device**
root@<jetson-host>:$HOME/esp/esp-idf#
```

### Below is esp32s3 device port name
```
cdc_acm 1-1:1.0: ttyACM0: USB ACM device
```

### If installed esp-idf then we need to reload the rules, give the device permission to plugdev group
**some examples**
```
sudo cp -n $HOME/.espressif/tools/openocd-esp32/v0.12.0-esp32-**20250707**/openocd-esp32/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d
sudo cp -n $HOME/.espressif/tools/openocd-esp32/v0.12.0-esp32-**20241016**/openocd-esp32/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d

<team-user>@<jetson-host>:~$ ls -l /etc/udev/rules.d
total 128
-rw-r--r-- 1 root root   204 Jan  7  2025 10-nv-jetson-dm.rules
-rw-r--r-- 1 root root 23513 Oct 25 11:20 70-snap.chromium.rules
-rw-r--r-- 1 root root   758 Oct 25 10:59 70-snap.cups.rules
-rw-r--r-- 1 root root 63357 Oct 25 10:59 70-snap.snapd.rules
-rw-r--r-- 1 root root   175 Jan  7  2025 91-xorg-conf-tegra.rules
-rw-r--r-- 1 root root   777 Jan  7  2025 99-nv-l4t-usb-device-mode.rules
-rw-r--r-- 1 root root  1448 Jan  7  2025 99-nv-l4t-usb-host-config.rules
-rw-r--r-- 1 root root   907 Jan  7  2025 99-nv-ufs-mount.rules
-rw-r--r-- 1 root root  1114 Jan  7  2025 99-nv-wifibt.rules
-rw-r--r-- 1 root root  5092 Jan  7  2025 99-tegra-devices.rules
-rw-r--r-- 1 root root   130 Jan  7  2025 99-tegra-mmc-ra.rules
<team-user>@<jetson-host>:~$
```

### Check esp32s3 device user/group permission before change
```
<team-user>@<jetson-host>:~$ ls -l /dev/ttyACM*
crw-rw---- 1 root **dialout** 166, 0 Oct 25 15:08 /dev/ttyACM0

lsusb
Bus 002 Device 002: ID 0bda:0489 Realtek Semiconductor Corp. 4-Port USB 3.0 Hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 13d3:3549 IMC Networks Bluetooth Radio
Bus 001 Device 005: ID 093a:2521 Pixart Imaging, Inc. Optical Mouse
Bus 001 Device 004: ID 258a:0001 SINO WEALTH USB KEYBOARD
Bus 001 Device 002: ID 0bda:5489 Realtek Semiconductor Corp. 4-Port USB 2.0 Hub
Bus 001 Device 009: ID 303a:1001 Espressif USB JTAG/serial debug unit
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

### Copy the rules and reload to fix the device user/group permission
```
sudo cp -n $HOME/.espressif/tools/openocd-esp32/v0.12.0-esp32-20250707/openocd-esp32/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Check esp32s3 device user/group permission againa after change
```
<team-user>@<jetson-host>:~$ sudo udevadm trigger
<team-user>@<jetson-host>:~$ ls -l /dev/ttyACM*
crw-rw----+ 1 root **plugdev** 166, 0 Oct 25 16:40 /dev/ttyACM0
<team-user>@<jetson-host>:~$ lsusb
Bus 002 Device 002: ID 0bda:0489 Realtek Semiconductor Corp. 4-Port USB 3.0 Hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 13d3:3549 IMC Networks Bluetooth Radio
Bus 001 Device 005: ID 093a:2521 Pixart Imaging, Inc. Optical Mouse
Bus 001 Device 004: ID 258a:0001 SINO WEALTH USB KEYBOARD
Bus 001 Device 002: ID 0bda:5489 Realtek Semiconductor Corp. 4-Port USB 2.0 Hub
Bus 001 Device 009: ID 303a:1001 Espressif USB JTAG/serial debug unit
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

### To prevent error, at linux, add user to dialout group

The currently logged-in user must have permission to read from and write to the serial port over USB.

In most Linux distributions, you can grant this permission by adding the user to the dialout group with the following command:

```
sudo usermod -a -G dialout <team-github-account>
```
