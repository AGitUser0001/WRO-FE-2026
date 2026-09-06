# Miscellaneous Instructions

## Load 'cp210x' module
Edit /etc/modules and insert this line:
```
# CP210X Driver
cp210x
```

This loads the driver for the LiDAR serial to USB converter.

## Load 'cdc_acm' module
Edit /etc/modules and insert this line:
```
# IMU
cdc_acm
```

## Setup Ethernet
Run `nmcli device status`:
```bash
<team-user>@<jetson-host>:~$ nmcli device status
DEVICE            TYPE      STATE                   CONNECTION
wlP1p1s0          wifi      connected               <...>
docker0           bridge    connected (externally)  docker0
enP8p1s0          ethernet  disconnected            --         <-- Ethernet
p2p-dev-wlP1p1s0  wifi-p2p  disconnected            --
l4tbr0            bridge    unmanaged               --
can0              can       unmanaged               --
usb0              ethernet  unmanaged               --
usb1              ethernet  unmanaged               --
lo                loopback  unmanaged               --
```

Run:
```bash
sudo nmcli con add type ethernet ifname enP8p1s0 con-name eth-static \
  ipv4.method manual ipv4.addresses 192.168.10.1/24
sudo nmcli con up eth-static
```

The command format is:
```bash
sudo nmcli con add type ethernet ifname <ETHERNET_DEVICE> con-name eth-static \
  ipv4.method manual ipv4.addresses <AGENT_IP>/24
```


## Add user to dialout group
```bash
sudo usermod -a -G dialout $USER
```

This allows the user to read/write /dev/ttyX without 'sudo' or 'sudo chmod 777 /dev/ttyX'
