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
