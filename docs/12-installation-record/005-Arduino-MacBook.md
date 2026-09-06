## Macbook - Install Arduino IDE from scratch
```
brew install --cask arduino-ide
```

### `brew info arduino-ide`
```
==> arduino-ide: 2.3.6
https://www.arduino.cc/en/software
Installed
/opt/homebrew/Caskroom/arduino-ide/2.3.6 (525MB)
From: https://github.com/Homebrew/homebrew-cask/blob/HEAD/Casks/a/arduino-ide.rb
==> Name
Arduino IDE
==> Description
Electronics prototyping platform
==> Artifacts
Arduino IDE.app (App)
==> Analytics
install: 626 (30 days), 1,867 (90 days), 7,096 (365 days)
```

**Go to your Launchpad and double click "Arduino IDE".**

`"Arduino IDE" is an app downloaded from the internet. Are you sure you want to open it?`

**Choose "Open"**

### Install ESP32 Library in Arduino IDE
https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html

Copy below link

https://espressif.github.io/arduino-esp32/package_esp32_index.json

Go to Arduino IDE, open setting
![](images/arduino-open-setting.png)

Paste the link at "Additional boards manager URLs" and click "ok"
![](images/arduino-paste-manager-urls.png)

Installing ESP32 Board Package in Arduino IDE
![](images/arduino-selet-board-manager.png)

![](images/arduino-installing-esp32-board-package.png)

ESP32 Board Package we are using "esp32 by Espressif Systems" (version 3.3.4), click "INSTALL"

![](images/arduino-esp32-package-install-process.png)

![](images/arduino-esp32-package-install-completed.png)

### Install micro-ROS for Arduino library.
Download micro_ros for Arduino ESP32 Core (with minor patch) (version 2.0.8-humble)

Go to
https://github.com/micro-ROS/micro_ros_arduino

Select branch "humble"
https://github.com/micro-ROS/micro_ros_arduino/tree/humble

Go to
https://github.com/micro-ROS/micro_ros_arduino/releases
Find v2.0.8-humble

Expand "Assets"

Download "Source code (zip)"

Link:
https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.8-humble.zip

Save the zip file to $HOME/Documents/Arduino

```
<team-user>@<jetson-host> Arduino % pwd
$HOME/Documents/Arduino
<team-user>@<jetson-host> Arduino % ls
micro_ros_arduino-2.0.8-humble.zip
<team-user>@<jetson-host> Arduino % ls -l
total 32912
-rw-r--r--@ 1 <local-user>  staff  16849977 22 Nov 14:40 micro_ros_arduino-2.0.8-humble.zip
<team-user>@<jetson-host> Arduino %
```

Install micro_ros_arduino-2.0.8-humble.zip at Arduino
![](images/add-mircoROS-zip-to-arduino-lib.png)

![](images/add-mircoROS-zip-to-arduino-lib-02.png)

Click Open, then at Arduino IDE terminal see message "Library Installed"
![](images/add-mircoROS-zip-to-arduino-lib-succ.png)

Go back to $HOME/Documents/Arduino, you will see libraries/micro_ros_arduino folder
This is a precompiled library

```
<team-user>@<jetson-host> Arduino % ls -l
total 32912
drwxr-xr-x  3 <local-user>  staff        96 22 Nov 14:46 libraries
-rw-r--r--@ 1 <local-user>  staff  16849977 22 Nov 14:40 micro_ros_arduino-2.0.8-humble.zip
<team-user>@<jetson-host> Arduino % ls -l libraries
total 0
drwxr-xr-x  16 <local-user>  staff  512 22 Nov 14:46 micro_ros_arduino
<team-user>@<jetson-host> Arduino %
```

### Install U8g2 library for OLED display
U8g2 library for Arduino IDE (version 2.35.30)

![](images/arduino-tools-select-manage-libraries.png)

search U8g2
![](images/arduino-search-library-u8g2.png)

Click "INSTALL", wait to completed
![](images/u8g2-lib-installed-arduino.png)

### Install arduino-audio-driver library

[arduino-audio-driver Github Repo](https://github.com/pschatzmann/arduino-audio-driver)

![](images/arduino-audio-driver-lib-github-download-zip.png)

![](images/arduino-audio-driver-lib-github-download-zip-02.png)

Rename the github download zip file

```bash
mv arduino-audio-driver-main.zip arduino-audio-driver.zip
```

![](images/IDE-add-arduino-audio-driver-lib-zip-menu.png)

![](images/IDE-select-arduino-audio-driver-lib-zip-screenshot.png)

![](images/arduino-audio-driver-lib-installed.png)

### C++ code for Arduino IDE/ESP32s3/MicroROS

Our code is at the folder:

`./newcode/esp32-code`

You need to update line 269:

`set_microros_wifi_transports("<WIFI_SSID>", "<WIFI_PASSWORD>", "<AGENT_IP>", 8888)`

<WIFI_SSID> is `<WIFI SSID>`

<WIFI_PASSWORD> is  `<WIFI Password>`

192.168.1.79 is `<Micro ROS Agent IP>`

8888 is `<Micro ROS Agent Port>`

### How to load our esp32 code in Arduino IDE

![](images/arduino-open-file-01.png)

Code can be at any folder, below is my code path

Remember Arduino IDE can only open "File" not "Directory"

So your **filename must same as Directory name**,  then it will open the whole folder

`$HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino`

![](images/arduino-open-file-02.png)

Click "Open", it would open a new "Arduino IDE" window and include the whole folder code

![](images/succ-load-esp32s3-code-arduino.png)

### Arduino IDE prepare for build / flash code to ESP32s3
**Every time before building and flashing the code, you must select the Board and the Port.**

Select ESP32 Board at Arduino IDE
![](images/select-esp32-board-arduino.png)

![](images/esp32-board-selected-at-arduino.png)

Make sure your esp32s3 is using usb to connect to your macbook

```
<team-user>@<jetson-host> newcode % ls -l /dev/cu*
crw-rw-rw-  1 root  wheel  0x9000001  3 Oct 13:43 /dev/cu.Bluetooth-Incoming-Port
crw-rw-rw-  1 root  wheel  0x9000003 22 Nov 14:24 /dev/cu.usbmodem1101
```

`/dev/cu.usbmodem1101` is ESP32s3 Port

Then select port at Arduino IDE
![](images/select-esp32s3-port-arduino.png)

**Set partition scheme**
Our code is too large for the default partition scheme, so we will have to change the partition scheme.
Go to the menu, select **Tools** -> **Partition Scheme** -> *Minimal SPIFFS (1.9MB APP with OTA/128KB SPIFFS)*.
As an alternative, you can also choose *Huge APP (3MB No OTA/1MB SPIFFS)*.

### Verify Code

![](images/arduino-ide-verify-code.png)

compiling
![](images/arduino-compiling.png)

Error:

![](images/arduino-compile-verify-error-undefined-reference.png)

```
Library micro_ros_arduino has been declared precompiled:
**Precompiled library in "$HOME/Documents/Arduino/libraries/micro_ros_arduino/src/esp32s3" not found**
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z12publish_textPKcb+0x1c): undefined reference to `rcl_publish'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z4loopv+0xc): **undefined reference** to `rclc_executor_spin_some'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0x9c): undefined reference to `rmw_uros_set_custom_transport'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xa0): undefined reference to `rcutils_get_default_allocator'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xa4): undefined reference to `rclc_support_init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xac): undefined reference to `rclc_node_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xb0): undefined reference to `rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__String'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xb4): undefined reference to `rclc_publisher_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xb8): undefined reference to `std_msgs__msg__String__init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xc0): undefined reference to `rclc_subscription_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xc4): undefined reference to `rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int32'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xc8): undefined reference to `std_msgs__msg__Int32__init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xcc): undefined reference to `rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt16'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xd0): undefined reference to `std_msgs__msg__UInt16__init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xd4): undefined reference to `rclc_timer_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xd8): undefined reference to `rclc_executor_init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xdc): undefined reference to `rclc_executor_add_subscription'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o:(.literal._Z5setupv+0xe0): undefined reference to `rclc_executor_add_timer'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o: in function `_Z12publish_textPKcb':
$HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:72:(.text._Z12publish_textPKcb+0x4c): undefined reference to `rcl_publish'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:74:(.text._Z12publish_textPKcb+0x84): undefined reference to `rcl_publish'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o: in function `_Z15publish_encodert':
$HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:215:(.text._Z15publish_encodert+0x1d): undefined reference to `rcl_publish'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o: in function `_Z4loopv':
$HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:365:(.text._Z4loopv+0x20): undefined reference to `rclc_executor_spin_some'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o: in function `_Z5setupv':
$HOME/Documents/Arduino/libraries/micro_ros_arduino/src/micro_ros_arduino.h:147:(.text._Z5setupv+0xae): undefined reference to `rmw_uros_set_custom_transport'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Library/Caches/arduino/sketches/CF768F34FC47A0606F882E35BAFB02FE/sketch/esp32-code.ino.cpp.o: in function `_Z5setupv':
$HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:273:(.text._Z5setupv+0xbf): undefined reference to `rcutils_get_default_allocator'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:276:(.text._Z5setupv+0xe0): undefined reference to `rclc_support_init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:279:(.text._Z5setupv+0x148): undefined reference to `rclc_node_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:282:(.text._Z5setupv+0x156): undefined reference to `rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__String'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:282:(.text._Z5setupv+0x166): undefined reference to `rclc_publisher_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:287:(.text._Z5setupv+0x17b): undefined reference to `std_msgs__msg__String__init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:294:(.text._Z5setupv+0x194): undefined reference to `rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__String'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:294:(.text._Z5setupv+0x1a4): undefined reference to `rclc_subscription_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:299:(.text._Z5setupv+0x1ba): undefined reference to `std_msgs__msg__String__init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:305:(.text._Z5setupv+0x1d2): undefined reference to `rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int32'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:305:(.text._Z5setupv+0x1e2): undefined reference to `rclc_subscription_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:310:(.text._Z5setupv+0x1f3): undefined reference to `std_msgs__msg__Int32__init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:313:(.text._Z5setupv+0x1f9): undefined reference to `rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt16'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:313:(.text._Z5setupv+0x20a): undefined reference to `rclc_publisher_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:318:(.text._Z5setupv+0x21b): undefined reference to `std_msgs__msg__UInt16__init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:321:(.text._Z5setupv+0x221): undefined reference to `rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int32'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:321:(.text._Z5setupv+0x232): undefined reference to `rclc_subscription_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:326:(.text._Z5setupv+0x243): undefined reference to `std_msgs__msg__Int32__init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:330:(.text._Z5setupv+0x257): undefined reference to `rclc_timer_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:337:(.text._Z5setupv+0x272): undefined reference to `rclc_timer_init_default'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:345:(.text._Z5setupv+0x28c): undefined reference to `rclc_executor_init'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:349:(.text._Z5setupv+0x2a8): undefined reference to `rclc_executor_add_subscription'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:352:(.text._Z5setupv+0x2c4): undefined reference to `rclc_executor_add_subscription'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:355:(.text._Z5setupv+0x2e0): undefined reference to `rclc_executor_add_subscription'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:357:(.text._Z5setupv+0x2f3): undefined reference to `rclc_executor_add_timer'
$HOME/Library/Arduino15/packages/esp32/tools/esp-x32/2507/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/bin/ld: $HOME/Documents/test/robot/doc/newcode/esp32-code/esp32-code.ino:360:(.text._Z5setupv+0x306): undefined reference to `rclc_executor_add_timer'
collect2: error: ld returned 1 exit status
exit status 1

Compilation error: exit status 1
```

### Path the micro_ros library at Arduino Libary
how to resolve arduino **undefined reference** issue

The actual error is:

`Precompiled library in "$HOME/Documents/Arduino/libraries/micro_ros_arduino/src/esp32s3" not found`

```
<team-user>@<jetson-host> Arduino % ls $HOME/Documents/Arduino/libraries/micro_ros_arduino/src/esp32/
libmicroros.a
<team-user>@<jetson-host> Arduino %

cd $HOME/Documents/Arduino/libraries/micro_ros_arduino/src

cp -pa esp32 esp32s3

<team-user>@<jetson-host> Arduino % cd $HOME/Documents/Arduino/libraries/micro_ros_arduino/src/
<team-user>@<jetson-host> src % cp -pa esp32 esp32s3
<team-user>@<jetson-host> src % ls -ld esp32s3
drwxr-xr-x  3 <local-user>  staff  96 22 Nov 14:47 esp32s3
<team-user>@<jetson-host> src % ls -ld esp32
drwxr-xr-x  3 <local-user>  staff  96 22 Nov 14:47 esp32
<team-user>@<jetson-host> src % ls -l esp32s3
total 27640
-rwxr-xr-x  1 <local-user>  staff  141486
```

![](images/fix-arduino-undefined-reference-issue.jpeg)

### Verify Code again

![](images/arduino-ide-verify-again-code-and-succ.png)

No error show up, below is the output:

```
Library micro_ros_arduino has been declared precompiled:
Using precompiled library in $HOME/Documents/Arduino/libraries/micro_ros_arduino/src/esp32s3
Sketch uses 1247295 bytes (95%) of program storage space. Maximum is 1310720 bytes.
Global variables use 73000 bytes (22%) of dynamic memory, leaving 254680 bytes for local variables. Maximum is 327680 bytes.
```

### Upload Code (Flash)

![](images/arduino-upload-code-01.png)

![](images/arduino-upload-code-02-succ.png)

### If you face issue: : Failed to connect to ESP32-S3: No serial data received.
**Then your esp32s3 needs to enter the bootloader mode (download mode)**

Hold down the BOOT button and then pressing and release RESET button while still holding the BOOT button

IMPORTANT NOTE: First time you try to upload the code to the board you purchased, you might see this error:

`A fatal error occurred: Failed to connect to ESP32-S3: No serial data received.`

The solution for this problem is to hold down the BOOT button and then pressing and release RESET button while still holding the BOOT button. This will initiate Firmware Download mode, and you only need to do it once.

Some tutorials suggest that you should hold BOOT button while uploading the code. However, this is not necessary in Arduino IDE (at least not for the board we are using).

### Arduino IDE upload code (flash esp32s3) output history:
```
Library micro_ros_arduino has been declared precompiled:
Using precompiled library in $HOME/Documents/Arduino/libraries/micro_ros_arduino/src/esp32s3
Sketch uses 1247295 bytes (95%) of program storage space. Maximum is 1310720 bytes.
Global variables use 73000 bytes (22%) of dynamic memory, leaving 254680 bytes for local variables. Maximum is 327680 bytes.
esptool v5.1.0
Serial port /dev/cu.usbmodem1101:
Connecting...
Connected to ESP32-S3 on /dev/cu.usbmodem1101:
Chip type:          ESP32-S3 (QFN56) (revision v0.1)
Features:           Wi-Fi, BT 5 (LE), Dual Core + LP Core, 240MHz, Embedded PSRAM 8MB (AP_3v3)
Crystal frequency:  40MHz
USB mode:           USB-Serial/JTAG
MAC:                f4:12:fa:fd:93:d8

Uploading stub flasher...
Running stub flasher...
Stub flasher running.
Changing baud rate to 921600...
Changed.

Configuring flash size...
Flash will be erased from 0x00000000 to 0x00004fff...
Flash will be erased from 0x00008000 to 0x00008fff...
Flash will be erased from 0x0000e000 to 0x0000ffff...
Flash will be erased from 0x00010000 to 0x00140fff...
Compressed 20224 bytes to 13061...

Writing at 0x00000000 [                              ]   0.0% 0/13061 bytes...

Writing at 0x00004f00 [==============================] 100.0% 13061/13061 bytes...
Wrote 20224 bytes (13061 compressed) at 0x00000000 in 0.4 seconds (419.1 kbit/s).
Hash of data verified.
Compressed 3072 bytes to 146...

Writing at 0x00008000 [                              ]   0.0% 0/146 bytes...

Writing at 0x00008c00 [==============================] 100.0% 146/146 bytes...
Wrote 3072 bytes (146 compressed) at 0x00008000 in 0.1 seconds (390.2 kbit/s).
Hash of data verified.
Compressed 8192 bytes to 47...

Writing at 0x0000e000 [                              ]   0.0% 0/47 bytes...

Writing at 0x00010000 [==============================] 100.0% 47/47 bytes...
Wrote 8192 bytes (47 compressed) at 0x0000e000 in 0.1 seconds (590.2 kbit/s).
Hash of data verified.
Compressed 1247440 bytes to 754162...

Writing at 0x00010000 [                              ]   0.0% 0/754162 bytes...

Writing at 0x0001c313 [                              ]   2.2% 16384/754162 bytes...

Writing at 0x00027826 [>                             ]   4.3% 32768/754162 bytes...

Writing at 0x0002f6c5 [>                             ]   6.5% 49152/754162 bytes...

Writing at 0x000420e0 [=>                            ]   8.7% 65536/754162 bytes...

Writing at 0x0004e861 [==>                           ]  10.9% 81920/754162 bytes...

Writing at 0x000544dc [==>                           ]  13.0% 98304/754162 bytes...

Writing at 0x00059bc8 [===>                          ]  15.2% 114688/754162 bytes...

Writing at 0x0005f73d [====>                         ]  17.4% 131072/754162 bytes...

Writing at 0x00065119 [====>                         ]  19.6% 147456/754162 bytes...

Writing at 0x0006ab22 [=====>                        ]  21.7% 163840/754162 bytes...

Writing at 0x0006faf6 [======>                       ]  23.9% 180224/754162 bytes...

Writing at 0x00074ec9 [======>                       ]  26.1% 196608/754162 bytes...

Writing at 0x0007a269 [=======>                      ]  28.2% 212992/754162 bytes...

Writing at 0x0007f629 [========>                     ]  30.4% 229376/754162 bytes...

Writing at 0x00084f7b [========>                     ]  32.6% 245760/754162 bytes...

Writing at 0x0008a144 [=========>                    ]  34.8% 262144/754162 bytes...

Writing at 0x0008f540 [==========>                   ]  36.9% 278528/754162 bytes...

Writing at 0x00094d6b [==========>                   ]  39.1% 294912/754162 bytes...

Writing at 0x0009a74d [===========>                  ]  41.3% 311296/754162 bytes...

Writing at 0x0009fed5 [============>                 ]  43.4% 327680/754162 bytes...

Writing at 0x000a53ce [============>                 ]  45.6% 344064/754162 bytes...

Writing at 0x000aa9b5 [=============>                ]  47.8% 360448/754162 bytes...

Writing at 0x000afbd2 [=============>                ]  50.0% 376832/754162 bytes...

Writing at 0x000b4fb6 [==============>               ]  52.1% 393216/754162 bytes...

Writing at 0x000ba792 [===============>              ]  54.3% 409600/754162 bytes...

Writing at 0x000bfd46 [===============>              ]  56.5% 425984/754162 bytes...

Writing at 0x000c524a [================>             ]  58.7% 442368/754162 bytes...

Writing at 0x000ca5c0 [=================>            ]  60.8% 458752/754162 bytes...

Writing at 0x000cfd37 [=================>            ]  63.0% 475136/754162 bytes...

Writing at 0x000d69c4 [==================>           ]  65.2% 491520/754162 bytes...

Writing at 0x000deae9 [===================>          ]  67.3% 507904/754162 bytes...

Writing at 0x000e67a8 [===================>          ]  69.5% 524288/754162 bytes...

Writing at 0x000eec05 [====================>         ]  71.7% 540672/754162 bytes...

Writing at 0x000f50d4 [=====================>        ]  73.9% 557056/754162 bytes...

Writing at 0x000fa461 [=====================>        ]  76.0% 573440/754162 bytes...

Writing at 0x001008e9 [======================>       ]  78.2% 589824/754162 bytes...

Writing at 0x00105ea6 [=======================>      ]  80.4% 606208/754162 bytes...

Writing at 0x0010b6df [=======================>      ]  82.6% 622592/754162 bytes...

Writing at 0x001131f0 [========================>     ]  84.7% 638976/754162 bytes...

Writing at 0x0011c34f [=========================>    ]  86.9% 655360/754162 bytes...

Writing at 0x00123b57 [=========================>    ]  89.1% 671744/754162 bytes...

Writing at 0x00128c33 [==========================>   ]  91.2% 688128/754162 bytes...

Writing at 0x0012e70c [===========================>  ]  93.4% 704512/754162 bytes...

Writing at 0x001347ba [===========================>  ]  95.6% 720896/754162 bytes...

Writing at 0x0013a3f0 [============================> ]  97.8% 737280/754162 bytes...

Writing at 0x0014064b [============================> ]  99.9% 753664/754162 bytes...

Writing at 0x001408d0 [==============================] 100.0% 754162/754162 bytes...
Wrote 1247440 bytes (754162 compressed) at 0x00010000 in 9.8 seconds (1014.4 kbit/s).
Hash of data verified.

Hard resetting via RTS pin...
```
