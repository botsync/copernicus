#!/bin/bash
find /etc/udev/rules.d -name "50-myusb.rules" -delete
touch /etc/udev/rules.d/50-myusb.rules
cat > /etc/udev/rules.d/50-myusb.rules <<EOL
KERNEL=="ttyACM[0-9]*",MODE="0666",
KERNEL=="ttyUSB[0-9]*",MODE="0666",
KERNEL=="video[0-9]*",MODE="0666",
SUBSYSTEM=="tty", ACTION=="add", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="2146400", SYMLINK+="rdc",
SUBSYSTEM=="tty", ACTION=="add", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="2147010", SYMLINK+="pb"
EOL
cat /etc/udev/rules.d/50-myusb.rules


