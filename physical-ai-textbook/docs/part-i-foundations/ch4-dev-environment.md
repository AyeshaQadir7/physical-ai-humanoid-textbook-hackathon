---
title: Development Environment Setup
description: Complete guide to setting up the development environment for Physical AI & Humanoid Robotics
hide_table_of_contents: false
---

# Development Environment Setup

## Overview

Setting up a proper development environment is crucial for success in Physical AI and Humanoid Robotics. This guide will walk you through installing and configuring all necessary tools, frameworks, and dependencies required for the course. The environment will support ROS 2, Gazebo, Unity, NVIDIA Isaac, and other essential tools.

## System Requirements

### Hardware Requirements
- **CPU**: Multi-core processor (Intel i7 or equivalent AMD Ryzen recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: NVIDIA GPU with CUDA support (RTX 3070 or equivalent/above)
- **Storage**: 100GB+ available space (SSD recommended)
- **Network**: Reliable internet connection for downloads and updates

### Operating System
- **Primary**: Ubuntu 22.04 LTS (Long Term Support)
- **Alternative**: Ubuntu 20.04 LTS (with some compatibility considerations)

## Installing Ubuntu 22.04 LTS

### Option 1: Native Installation
1. Download Ubuntu 22.04 LTS ISO from [ubuntu.com](https://ubuntu.com/download/desktop)
2. Create a bootable USB drive using:
   - **Windows**: Rufus or Etcher
   - **Linux**: `dd` command or Startup Disk Creator
   - **macOS**: Etcher or Terminal with `dd` command
3. Boot from USB and follow installation wizard
4. Allocate at least 100GB for the Ubuntu partition

### Option 2: Virtual Machine
1. Install VirtualBox or VMware Workstation
2. Create a new VM with:
   - 8GB+ RAM allocation
   - 100GB+ virtual hard disk
   - Enable hardware virtualization in BIOS
3. Install Ubuntu 22.04 LTS in the VM

### Option 3: Dual Boot (Recommended)
- Partition your hard drive to accommodate both operating systems
- Install Ubuntu alongside your existing OS
- Use GRUB bootloader for OS selection

## Essential System Configuration

### Update System Packages
```bash
sudo apt update && sudo apt upgrade -y
```

### Install Basic Development Tools
```bash
sudo apt install build-essential cmake git curl wget vim htop python3-pip python3-dev
```

### Install Additional Utilities
```bash
sudo apt install terminator gnome-tweaks gdebi-core
```

## ROS 2 Humble Hawksbill Installation

### Set up Locale
```bash
locale  # Check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Add ROS 2 Repository
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 Packages
```bash
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### Environment Setup
Add to your `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

## Gazebo Installation

### Install Gazebo Garden
```bash
curl -sSL http://get.gazebosim.org | sh
```

### Alternative Installation Method
```bash
sudo apt update
sudo apt install gazebo
```

## NVIDIA GPU and CUDA Setup

### Install NVIDIA Drivers
```bash
sudo apt install ubuntu-drivers-common
sudo ubuntu-drivers autoinstall
# Or manually install from NVIDIA's website
```

### Install CUDA Toolkit
```bash
wget https://developer.download.nvidia.com/compute/cuda/12.1.0/local_installers/cuda_12.1.0_530.30.02_linux.run
sudo sh cuda_12.1.0_530.30.02_linux.run
```

### Verify GPU Access
```bash
nvidia-smi
```

## Python Environment Management

### Install Python Virtual Environment Tools
```bash
pip3 install virtualenv virtualenvwrapper
```

### Set Up Virtual Environment
```bash
mkvirtualenv robotics_env
workon robotics_env
```

### Install Essential Python Packages
```bash
pip install numpy scipy matplotlib pandas jupyter notebook
pip install opencv-python open3d
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

## Unity Hub and Unity Installation

### Install Unity Hub
1. Download Unity Hub from [unity.com](https://unity.com/download)
2. Install using the .deb package:
```bash
sudo gdebi unity-hub-amd64-3.4.1.deb  # Replace with actual filename
```

### Install Unity 2022.3 LTS
1. Open Unity Hub
2. Sign in with Unity ID
3. Go to Installs tab
4. Click "Add" and select Unity 2022.3 LTS
5. Install with Linux Build Support and Visual Scripting

## NVIDIA Isaac Installation

### Install Isaac ROS Common
```bash
# Add NVIDIA package repository
curl -sSL https://repo.download.nvidia.com/config/ubuntu-22.04/+pubkey.gpg | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-archive-keyring.gpg] https://repo.download.nvidia.com/isaac/2023-2/ubuntu-22.04/amd64/ /" | sudo tee /etc/apt/sources.list.d/nvidia-isaac.list
sudo apt update

# Install Isaac ROS packages
sudo apt install nvidia-isaac-common
sudo apt install nvidia-isaac-sight
sudo apt install nvidia-isaac-isaac_ros_benchmark
```

### Install Isaac Sim
1. Visit [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
2. Download and install following NVIDIA's instructions
3. Ensure CUDA and GPU drivers are properly configured

## Git Configuration

### Set Up Git Identity
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
git config --global core.editor vim
```

### Set Up SSH Keys for GitHub
```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

### Add SSH Key to GitHub
1. Copy SSH key: `cat ~/.ssh/id_ed25519.pub`
2. Go to GitHub Settings > SSH and GPG keys
3. Add new SSH key with the copied content

## Workspace Organization

### Create ROS 2 Workspace Structure
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Project Directory Structure
```
~/physical-ai-projects/
├── ros2_projects/
├── simulation_projects/
├── unity_projects/
├── isaac_projects/
├── documentation/
└── research/
```

## Testing the Environment

### Test ROS 2 Installation
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

### Test Gazebo Installation
```bash
gazebo
```

### Test Python Environment
```bash
python3 -c "import torch; print(f'PyTorch CUDA available: {torch.cuda.is_available()}')"
python3 -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"
```

### Test NVIDIA GPU
```bash
nvidia-smi
nvcc --version
```

## Troubleshooting Common Issues

### ROS 2 Installation Issues
- **Issue**: Package not found during ROS 2 installation
- **Solution**: Ensure locale is set to UTF-8 and repository is properly added

### GPU/CUDA Issues
- **Issue**: CUDA not detected by PyTorch
- **Solution**: Verify NVIDIA drivers are properly installed and CUDA toolkit matches PyTorch requirements

### Unity Installation Issues
- **Issue**: Unity Hub won't start
- **Solution**: Install missing dependencies: `sudo apt install libfuse2`

### Gazebo Issues
- **Issue**: Gazebo crashes on startup
- **Solution**: Ensure proper graphics drivers and OpenGL support

## Environment Verification Script

Create a verification script to test all installations:

```bash
#!/bin/bash
# save as ~/verify_environment.sh

echo "=== ROS 2 Check ==="
source /opt/ros/humble/setup.bash
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 installed: $(ros2 --version)"
else
    echo "✗ ROS 2 not found"
fi

echo -e "\n=== NVIDIA GPU Check ==="
if command -v nvidia-smi &> /dev/null; then
    echo "✓ NVIDIA GPU available:"
    nvidia-smi --query-gpu=name,memory.total,driver-version --format=csv,noheader,nounits
else
    echo "✗ NVIDIA GPU not found"
fi

echo -e "\n=== Python Packages Check ==="
python3 -c "
import sys
print('✓ Python version:', sys.version)

try:
    import torch
    print('✓ PyTorch version:', torch.__version__)
    print('✓ CUDA available:', torch.cuda.is_available())
except ImportError:
    print('✗ PyTorch not installed')

try:
    import cv2
    print('✓ OpenCV version:', cv2.__version__)
except ImportError:
    print('✗ OpenCV not installed')

try:
    import numpy as np
    print('✓ NumPy version:', np.__version__)
except ImportError:
    print('✗ NumPy not installed')
"

echo -e "\n=== Gazebo Check ==="
if command -v gazebo &> /dev/null; then
    echo "✓ Gazebo available: $(gazebo --version 2>&1 | head -1)"
else
    echo "✗ Gazebo not found"
fi

echo -e "\nEnvironment verification complete!"
```

Make it executable and run:
```bash
chmod +x ~/verify_environment.sh
~/verify_environment.sh
```

## Next Steps

Once your environment is properly set up:

1. **Clone the course repository** with example code and exercises
2. **Run the verification script** to ensure all components work together
3. **Complete the first ROS 2 tutorial** to verify your setup
4. **Join the course Discord/Slack** for community support

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [Unity Learn](https://learn.unity.com/)
- [NVIDIA Isaac Documentation](https://docs.nvidia.com/isaac/)
- [Ubuntu Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop)

Your development environment is now ready for the Physical AI & Humanoid Robotics course. In the next modules, you'll begin working with these tools to build increasingly complex robot systems.