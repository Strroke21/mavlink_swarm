# ROS 2 Humble Setup Guide

## I. ROS 2 Humble Setup

### **1. Set Locale**

Before installing ROS 2, ensure your system locale is set to UTF-8.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### **2. Setup Sources**

To add the ROS 2 apt repository, enable the Ubuntu Universe repository.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### **3. Install ROS 2 Packages**

Update your apt repository caches and install ROS 2 packages.

```bash
sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
```

Install development tools.

```bash
sudo apt install ros-dev-tools
```

### **4. Environment Setup**

To start working with ROS 2, source the setup script in each terminal session.

```bash
source /opt/ros/humble/setup.bash
```

### **5. Try Some Examples**

**Talker-Listener Example**

Open a terminal and start a talker node:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py talker
```

Open another terminal and start a listener node:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener