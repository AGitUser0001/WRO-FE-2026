# Jetson initial setup

## Ref document link here:
https://www.jetson-ai-lab.com/initial_setup_jon.html#

## Ref video link here:
https://www.youtube.com/watch?v=-PjMC0gyH9s&t=258s

## Prerequisite:
1. 256G microSD
2. parriot NVMe Gen3 256g
3. car reader
4. Monitor
5. keyboard and mouse (USB)
6. DisplayPort to HDMI(Or DVI) or any ports fit for your Monitor

**You may begin directly from the following section and omit the preceding content.**
Firmware 36.x

![](images/start-from-firmware-36x-section.png)

## Download jetson jetpack package
Jetson Orin Nano Developer Kit

JetPack 6.

10.88G

Extracting it...

![](images/download-jetson-jetpack.png)

## Install Balena Etcher

https://etcher.balena.io/

Install and Open

## Use Balena Etcher to flash jetpack image to SD card

Selct "Flash from file" - sb-blob.img

Select target "<your 256G microSD>"

Click "Flash!"

Until you see "Flash Completed!"

Close Balena Etcher

Eject "microSD and card reader"

## Jetson hardware connect

![](images/jetson-hardware-connect.jpeg)

jetson connect to display and usb keyboard mounse

Display Port connect to Monitor / Display

USB port connect to Keyboard / Mouse

## Jetson GUI install

![](images/jetson-install-01.jpeg)

![](images/jetson-install-02.jpeg)

![](images/jetson-install-03.jpeg)

![](images/jetson-install-04.jpeg)

![](images/jetson-install-06.jpeg)

![](images/jetson-install-07.jpeg)

**you may facing chrome not able to install issue at here**

![](images/jetson-install-08.jpeg)

**just skip it**

![](images/jetson-install-09.jpeg)

jetson user:
`<team-github-account>`
jetson password:
`<...>`

### login in the system and finished the initial setup

![](images/jetson-install-10.jpeg)

![](images/jetson-install-11.jpeg)

![](images/jetson-install-12.jpeg)


### Install chromium after you login
open a terminal

input:
```
sudo apt update
sudo apt install chromium-browser
```

verify chromium installed
snap list chromium

### Command line run chromium
Open a new terminal
```
snap run chromium --no-sandbox --disable-gpu-sandbox --use-gl=egl
```

### Install software updated

![](images/jetson-software-update-01.jpeg)


![](images/jetson-software-update-02.jpeg)

After install software upated the system need to **reboot**

## Jetson config SSD and move Docker folder to SSD disk
https://www.jetson-ai-lab.com/tips_ssd-docker.html

The SSD is used as a data disk and for Docker storage; it is not used as the system disk.

You may encounter during this process:
```
Failed to restart docker.service: Unit is masked
```
and
```
Failed to restart docker.socket: Unit is masked
```
To fix this, do:
```
sudo systemctl unmask docker
sudo systemctl unmask docker.socket
```

## Jetson config a static network interface address
jetson static ip address setting

### Open GUI network setting
Click the grid icon in the bottom-left corner, then type “network” in the search bar at the top and open Advanced Network.

Edit your Wi-Fi SSID name, go to IPv4 Settings, and change the Method from Automatic (DHCP) to Manual.

### Get jetson current ip address:
open a new terminal, input:

```
ifconfig
```

get address and netmask

input ip route get gateway

### Go back to "Advanced Network" setting and config ip / networkmask / gateway
address add `192.168.2.106` (This is my home's example, you should change to your home's setting）

netmask：`255.255.255.0`

gateway: `192.168.2.1` (This is my home's example, you should change to your home's setting）

### Check your home dns server setting:
```
nmcli dev show wlP1p1s0 | grep DNS
```

### Go back to "Advanced Network" setting and config DNS Server
Multiple dns use comma to separate:
```
192.168.2.1,207.164.234.129
```
(This is my home's example, you should change to your home's setting）


### SSH to Jetson using local hostname
```
ssh <your-jetson-username>@<your-jetson-hostname>.local
ssh <team-user>@<jetson-host>.local
```

