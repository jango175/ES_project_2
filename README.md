# How to use?

1. Download and unpack the dataset to `/home/mp4d/Downloads`: https://nextcloud.sdu.dk/index.php/s/wZg4FLSxgiigJTL

2. To install OpenCV use these commands:
```bash
sudo apt update
sudo apt upgrade
sudo apt install libopencv-dev
```

3. Clone following repo to a different ros workspace:
```bash
git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

4. Move read_write_node.cpp to the ros workspace:
```bash
move read_write_node.cpp to src/DynamixelSDK/dynamixel_sdk_examples/src
```

5. Build workspace  and setup environment:
```bash
colcon build --symlink-install
. install/local_setup.bash

Allow non-root access to ttyACM with:
sudo usermod -aG dialout mp4d

run following command:
ros2 run dynamixel_sdk_examples read_write_node
```

6. Add submodules with:
```bash
git submodules update --init --recursive
```

7. Build project with:
```bash
bash build.sh
```

8. Start program with:
```bash
./FPGA_AI
```

# Vivado design
![Screenshot from 2024-12-27 16-19-16](https://github.com/user-attachments/assets/218a0d08-246e-4eb9-b414-54be787470bc)

# Sources
* https://github.com/nhma20/FPGA_AI/tree/ultra96-v2
