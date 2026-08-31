# diffsim_hardware

Real-time depth capture and processing on a SBC + Intel RealSense D435.

## Prerequisites

- OpenCV 4 with the `opencv_contrib` `rgbd` module (used for visual odometry). Check with:
  ```
  pkg-config --exists opencv4 && echo ok
  test -f /usr/include/opencv4/opencv2/rgbd.hpp && echo "rgbd module present"
  ```
- librealsense2 — see below if not already installed.

## Installing librealsense2 from source

Prebuilt packages lag behind and often don't target aarch64/Raspberry Pi well, so build from source.

1. Install build dependencies:
   ```
   sudo apt-get update
   sudo apt-get install -y git cmake build-essential \
       libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
       libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
   ```

2. Clone and configure:
   ```
   git clone https://github.com/realsenseai/librealsense.git
   cd librealsense
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true
   ```

3. Build (this takes a while on a Pi):
   ```
   make -j$(nproc)
   ```

4. Install to `/usr/local` and register it with the dynamic linker so it's found at build/run time (no `LD_LIBRARY_PATH` fiddling needed — `/usr/local/lib` is already on the default search path):
   ```
   sudo make install
   sudo ldconfig
   ```

5. Set up udev rules so the camera is accessible without root:
   ```
   cd ..
   ./scripts/setup_udev_rules.sh
   ```
   Unplug and replug the camera after this step.

6. Verify:
   ```
   pkg-config --modversion realsense2
   rs-enumerate-devices    # should list the D435
   ```

## Building this project

```
make
```

or, to build from an out-of-source `build/` directory:

```
cd build && make
```

Either way the binary lands at `build/depthcam`. `make clean` removes it.

```
./build/depthcam
```
