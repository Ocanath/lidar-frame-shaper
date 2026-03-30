# LiDAR UDP Forwarder

This software reads VLP-16 LiDAR data while simultaneously obtaining data from a DARTT motor controller. 


## Build Instructions

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

**Note: if you did not clone with the --recursive flag, you will need to initialize the submodules.**

```bash
git submodule update --init --recursive
```

## Notes on IP settings, phyical properties

- The unit has a 1.6:1 ratio via the belt.

- The vlp16 default ip is currently set to 192.168.1.201. You can access it on port 80

- You can configure static ip via nmcli using the script in scripts. Tested on my tower and it works. Set it to 192.168.1.79 

- White/Orange on RJ45 goes to Light Orange on the LIDAR
- Solid Orange on RJ45 goes to Dark Orange on the LiDAR
- Green on the RJ45 goes to Dark Blue on the LiDAR
- Green/White on the RJ45 goes to Light Blue on the LiDAR
