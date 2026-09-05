# diffsim_hardware

Real-time depth capture and processing on a SBC + Intel RealSense D435.

## Prerequisites

- OpenCV 4. Check with:
  ```
  pkg-config --exists opencv4 && echo ok
  ```
- The `opencv_contrib` `rgbd` module, only if building with visual odometry (`USE_VO=1`, see below). Check with:
  ```
  test -f /usr/include/opencv4/opencv2/rgbd.hpp && echo "rgbd module present"
  ```
- librealsense2 — see below if not already installed.

## Installing OpenCV (with the `rgbd` contrib module) from source

Only needed if building with visual odometry (`USE_VO=1`, see below). The distro package (`libopencv-dev`) doesn't include `opencv_contrib`, so the `rgbd` module used for visual odometry has to be built from source.

1. Install build dependencies:
   ```
   sudo apt-get update
   sudo apt-get install -y build-essential cmake git pkg-config \
       libjpeg-dev libpng-dev libtiff-dev \
       libavcodec-dev libavformat-dev libswscale-dev \
       libgtk-3-dev
   ```

2. Clone OpenCV and opencv_contrib at matching versions:
   ```
   git clone --branch 4.10.0 https://github.com/opencv/opencv.git
   git clone --branch 4.10.0 https://github.com/opencv/opencv_contrib.git
   ```

3. Configure. `BUILD_LIST` trims the build down to just what this project uses (core/imgproc/highgui/videoio/calib3d/features2d plus the `rgbd` contrib module) — building all of opencv_contrib on a Pi can take several hours, this cuts it down substantially:
   ```
   cd opencv
   mkdir build && cd build
   cmake .. \
       -DCMAKE_BUILD_TYPE=Release \
       -DCMAKE_INSTALL_PREFIX=/usr/local \
       -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
       -DBUILD_LIST=core,imgproc,imgcodecs,highgui,videoio,calib3d,features2d,rgbd \
       -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF
   ```

4. Build (still slow on a Pi — budget well over an hour):
   ```
   make -j$(nproc)
   ```

5. Install and register with the dynamic linker:
   ```
   sudo make install
   sudo ldconfig
   ```

6. Verify:
   ```
   pkg-config --modversion opencv4
   test -f /usr/local/include/opencv4/opencv2/rgbd.hpp && echo "rgbd module present"
   ```

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

By default this builds without visual odometry. To build it in (requires the `rgbd` contrib module above):

```
make USE_VO=1
```

By default the build also shows a live depth-preview window (`cv::imshow`). To leave that code out entirely — for headless builds with no GUI backend available — add `NO_DISPLAY=1`:

```
make USE_VO=1 NO_DISPLAY=1
```

Even without `NO_DISPLAY`, the window is skipped automatically at runtime if no `DISPLAY` environment variable is set, so a plain `ssh` session (without `-X`/`-Y`) won't fail trying to open a window.

The binary lands at `build/depthcam`. `make clean` removes it.

## Running

```
./build/depthcam
```

Optional flags:

- `--record session.bag` — capture the raw camera stream to a file while running live, so a run can be reviewed later.
- `--replay session.bag` — run the whole pipeline (VO, policy input, display) off a previously recorded file instead of a live camera.

```
./build/depthcam --record session.bag   # on the robot
./build/depthcam --replay session.bag   # later, to review
```
