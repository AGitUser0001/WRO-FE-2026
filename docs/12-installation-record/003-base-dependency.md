# Jetson install Base Dependencies

## Install the basic dependency packages required for C++ compilation on the Jetson
**run as user <team-github-account>**
```
sudo apt-get update
sudo apt-get install -y build-essential

sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

## Check jetson/ubuntu default python3 version
```
root@ubunturos2:$HOME# python3 -V
Python 3.10.12
```

## Jetson install vscode

```
apt install software-properties-common apt-transport-https wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg

sudo install -D -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/packages.microsoft.gpg

vi /etc/apt/sources.list.d/vscode.list
deb [arch=arm64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main
apt update
apt install code

code --version
```

## jetson run vscode(code)
### Open VS Code from the graphical interface
Click the grid icon (the app launcher) in the bottom left corner.
In the search bar at the top, type code.
You will see Visual Studio Code in the results — double-click to open it.

