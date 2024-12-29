# How to use?

1. Download and unpack the dataset to `/home/mp4d/Downloads`: https://nextcloud.sdu.dk/index.php/s/wZg4FLSxgiigJTL

2. To install OpenCV use these commands:
```bash
sudo apt update
sudo apt upgrade
sudo apt install libopencv-dev
```

3. Add submodules with:
```bash
git submodules update --init --recursive
```

4. Build project with:
```bash
bash build.sh
```

5. Start program with:
```bash
./FPGA_AI
```

# Vivado design
![Screenshot from 2024-12-27 16-19-16](https://github.com/user-attachments/assets/218a0d08-246e-4eb9-b414-54be787470bc)

# Sources
* https://github.com/nhma20/FPGA_AI/tree/ultra96-v2
