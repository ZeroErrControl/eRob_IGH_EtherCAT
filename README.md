# IGH EtherCAT (Linux)
---
## Table of Contents
1. [EtherCAT Driver ROS2 Stack](#ethercat-driver-ros2-stack)
2. [Quickstart](#quickstart)
   - [Installation](#installation)
   - [Prerequisites](#prerequisites)
---

#  IGH EtherCAT (Linux)
---
#  EtherCAT Driver ROS2 Stack

EtherCAT provides applications with the capacity of reliable, real-time communication between systems and is therefore a common industrial standard. In order to simplify the development/deployment of new application using EtherCAT modules, this stack allows to combine them with ros2_control. This driver proposes a generic ways to parametrize and assemble Hardware Interfaces based on EtherCAT modules that can be defined using parameter files.

Project GitHub repository: [ethercat_driver_ros2](https://github.com/ICube-Robotics/ethercat_driver_ros2)

---
## Quickstart

### Installation

---
#### Prerequisites
Before you begin, make sure you have the following:
- **Ubuntu 20.04\22.04\24.04** installed on your system.
---

#### Installing EtherLab

The proposed development builds upon the IgH EtherCAT Master. Installation steps are summarized here:
- Verify that you can run unsigned kernel modules
Etherlab is a kernel module that is not signed by default. To allow the kernel to load unsigned modules, you need to disable secure boot.

**Verify if secure boot is enabled (you need to install ‘’mokutil’’ first):**
``` bash
sudo apt-get install mokutil
mokutil --sb-state
```

it should print:
``` bash
SecureBoot disabled
```

---
if it prints:
``` bash
SecureBoot enabled
```


- Then you need to disable secure boot. To do so:

    - reboot your computer and enter the BIOS settings.

    - In the security tab, disable secure boot.

    - Save and exit.
---

- Install required tools:

``` bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git autoconf libtool pkg-config make build-essential net-tools
```

- Setup sources for the EtherCAT Master:

``` bash
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
git checkout stable-1.5
sudo rm /usr/bin/ethercat
sudo rm /etc/init.d/ethercat
./bootstrap  # to create the configure script
```

- Configure, build and install libs and kernel modules:

``` bash
/configure --prefix=/usr/local/etherlab  --disable-8139too --disable-eoe --enable-generic

make all modules
sudo make modules_install install
sudo depmod
```

Configure system:

``` bash
$ sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
$ sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
$ sudo mkdir -p /etc/sysconfig
$ sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
```

:::tip

These 4 steps may be needed every time the Linux kernel is updated. Before re-doing the 4 steps, you can try the following lighter steps:

Go into the folder where the EtherCAT project was cloned, from **Step 2 (Setup sources)**, do:

```bash
cd ethercat
sudo rm /usr/bin/ethercat /etc/init.d/ethercat
./bootstrap
```
Then complete Step 3 (Configure, build and install…) entirely. From Step 4 (Configure system), run:

```bash
sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
```
:::

- Create a new `udev` rule:

``` bash
sudo gedit /etc/udev/rules.d/99-EtherCAT.rules
```

containing:

``` bash
KERNEL=="EtherCAT[0-9]*", MODE="0666"
```

- Configure the network adapter for EtherCAT:

``` bash
sudo gedit /etc/sysconfig/ethercat
```
In the configuration file specify the mac address of the network card to be used and its driver

``` bash
MASTER0_DEVICE="ff:ff:ff:ff:ff:ff"  # mac address
DEVICE_MODULES="generic"
```

- Now you can start the EtherCAT master:

``` bash 
sudo /etc/init.d/ethercat start
```
it should print
``` bash 
Starting EtherCAT master 1.5.3  done
```

You can check connected slaves:
``` bash 
ethercat slaves
```
It should print information of connected slave device:

``` bash 
$ <id>  <alias>:<position>  <device_state>  +  <device_name>
```

Example:
``` bash 
0  0:0  PREOP  +  <device_0_name>
0  0:1  PREOP  +  <device_1_name>
```

### Building `ethercat_driver_ros2`
- 1. Install ROS2 packages. The current development is based of `ros2 humble`. Installation steps are described in the [ROS2 Humble Documentation](https://docs.ros.org/en/humble/Installation.html).

- 2. Source your ROS2` environment:

``` bash
source /opt/ros/humble/setup.bash
```
:::tip

The ROS2 environment needs to be sources in every used terminal. 
If only one distribution of ROS2 is used, it can be added to the ~/.bashrc file.

:::

- 3. Install colcon and its extensions :
``` bash
sudo apt install python3-colcon-common-extensions
```

- 4. Create a new ROS2 workspace:
``` bash
mkdir ~/ros2_ws/src
```

- 5. Pull relevant packages, install dependencies, compile, and source the workspace by using:
``` bash
cd ~/ros2_ws
git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git src/ethercat_driver_ros2
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```
