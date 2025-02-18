# EtherCAT CSV Mode Motor Control Demo

## Introduction
This is a motor control demo program based on IGH EtherCAT Master, demonstrating how to control a servo motor in CSV (Cyclic Synchronous Velocity) mode using EtherCAT communication.

## System Requirements
- Ubuntu 20.04/22.04/24.04
- IGH EtherCAT Master (v1.5.3)
- Servo drive supporting CiA402 protocol

## Installation

### 1. Install Required Tools
```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git autoconf libtool pkg-config make build-essential net-tools
```

### 2. Install IGH EtherCAT Master
```bash
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
git checkout stable-1.5
./bootstrap
./configure --prefix=/usr/local/etherlab --disable-8139too --disable-eoe --enable-generic
make all modules
sudo make modules_install install
sudo depmod
```

### 3. Configure System
```bash
sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
sudo mkdir -p /etc/sysconfig
sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
```

### 4. Network Configuration
1. Create udev rule:
```bash
echo 'KERNEL=="EtherCAT[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-EtherCAT.rules
```

2. Configure EtherCAT network adapter:
```bash
sudo gedit /etc/sysconfig/ethercat
```
Modify MAC address and driver in the configuration file:
```
MASTER0_DEVICE="xx:xx:xx:xx:xx:xx"  # Replace with your network card's MAC address
DEVICE_MODULES="generic"
```

## Usage

### 1. Start EtherCAT Master
```bash
sudo /etc/init.d/ethercat start
```

### 2. Build the Program
```bash
git clone https://github.com/ZeroErrControl/eRob_IGH_EtherCAT.git
cd eRob_IGH_EtherCAT
mkdir build
cd build
cmake ..
make
```

### 3. Run the Program
```bash
./igh_driver
```

### Program Features
- Automatic state machine transition for slave devices (from INIT to OPERATION ENABLED)
- In OPERATION ENABLED state, the motor will run at target velocity 10000 (units depend on drive configuration)
- Real-time display of actual motor velocity and status information

### Important Notes
1. Ensure EtherCAT master is properly started before running the program
2. Verify network configuration and slave device detection
3. Check slave status using:
```bash
ethercat slaves
```

## Safety Precautions
- Ensure the motor is securely mounted before first run
- Start with lower velocity values for testing
- Make sure emergency stop measures are in place

## Troubleshooting
If you encounter issues, check:
1. EtherCAT master status
2. Network connection
3. Slave device status
4. Program execution permissions (sudo required)

## License
This project is open source and provided for reference only. Please use with caution. The author is not responsible for any damages or losses.
