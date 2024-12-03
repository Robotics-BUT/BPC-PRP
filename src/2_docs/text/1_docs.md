# RPi Installation

## Required HW

 - Respberry Pi 4B (8GB RAM recommended)
 - Micro SD Card (minimal 16GB, 64GB recommended)
 - SD Card to USB Adapter

## Installation

Install RPi-Imager on your Linux PC

```
sudo apt install rpi-imager
```

Run RPi-Imager

```
rpi-imager
```
<img src="../images/rpi_install_imager.png" width="300">

Select Device (Raspberry Pi 4)

<img src="../images/rpi_install_select_device.png" width="300">

Select OS (Others General Purpose OS -> Ubuntu -> Ubuntu Server 24.04.1 LTS (64b))

<img src="../images/rpi_install_select_os_1.png" width="300">

<img src="../images/rpi_install_select_os_2.png" width="300">

<img src="../images/rpi_install_select_os_3.png" width="300">

Select target device (SD Card)

<img src="../images/rpi_install_select_msd.png" width="300">

Click Next -> Edit Settings

<img src="../images/rpi_install_settings.png" width="300">

Setup user, password, WiFi and SSH

<img src="../images/rpi_install_config_wifi.png" width="300">

<img src="../images/rpi_install_config_ssh.png" width="300">

Confirm and wait until writing is finished.

<img src="../images/rpi_install_writing.png" width="300">

Now put the SD card into the RPi and power on.

In case of problems, please visit [Official Documentation](https://www.raspberrypi.com/documentation/computers/getting-started.html#install-using-imager) 

## OS Configuration

Connect to the raspberry pi

```
ssh <user>@<ip_address>
```

Install basics

```
sudo apt update
sudo apt install -y git vim raspi-config htop locales software-properties-common curl libopencv-dev build-essential wiringpi v4l-utils i2c-tools
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```

Optional: Add swap space if RAM is too small (4GB and less)

```
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# check system
htop 
```

Install ROS2

```
locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

#ros
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-jazzy-cv-bridge ros-jazzy-image-transport

source /opt/ros/jazzy/setup.bash

# Colcon
curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | sudo bash
sudo apt install -y python3-colcon-common-extensions
```

Install Python tools and dependences

```
# Python-pip
sudo apt install -y python3-pip

# Dependences
pip3 install board rpi_ws281x adafruit-circuitpython-neopixel adafruit-circuitpython-bme280 adafruit-circuitpython-ads1x15 adafruit-circuitpython-mpu6050 adafruit-circuitpython-ds3231 RPLCD smbus2 pynmea2 --break-system-packages 
```

Now the system is installed.

## Setting Up Robot

TODO ...