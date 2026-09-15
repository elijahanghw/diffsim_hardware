# diffsim_hardware

Real-time depth capture and processing on a SBC + Intel RealSense D435.

It downsamples the D435 depth stream, runs the on-board CNN depth encoder to
produce the 64-float feature vector the navigation policy consumes, and
optionally bridges to an indiflight flight controller over pi-protocol serial —
relaying motion-capture pose,
position setpoints and keyboard input to the FC, and transmitting the CNN
features as `NN_INPUT_CHUNK`. Visual odometry can run on the side.

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

## Offline sync (target with no internet)

If the companion computer (e.g. a Radxa) can't reach GitHub, move the repo with a
git bundle — one integrity-checked file carrying full history, over USB or LAN.
`scripts/offline_bundle.sh` wraps both ends:

```sh
# on a machine with the up-to-date repo:
scripts/offline_bundle.sh create                          # -> ~/diffsim_hardware.bundle
scripts/offline_bundle.sh create --send radxa@radxa-cubie-a7z:~   # also scp it across

# copy the .bundle over (USB or scp), then on the offline machine:
scripts/offline_bundle.sh apply ~/diffsim_hardware.bundle              # update an existing clone
scripts/offline_bundle.sh apply ~/diffsim_hardware.bundle --clone ~/diffsim_hardware  # fresh clone
```

Run `scripts/offline_bundle.sh --help` for options. After applying, run
`make clean` before rebuilding — git doesn't preserve mtimes, so stale objects can
otherwise linger and cause link errors.

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

To build in the flight-controller bridge (see [Flight-controller bridge](#flight-controller-bridge) below), add `USE_RELAY=1`:

```
make USE_RELAY=1 USE_VO=1 NO_DISPLAY=1
```

This links in `src/relay/` (the FC bridge) and the vendored pi-protocol in `src/pi_protocol/`. No extra system packages are needed. The default build (`USE_RELAY=0`) leaves it out, along with the pi-protocol dependency.

The binary lands at `build/depthcam`. `make clean` removes it.

## Running

```
./build/depthcam
```

Optional flags:

- `--record session.bag` — capture the raw camera stream to a file while running live, so a run can be reviewed later.
- `--replay session.bag` — run the whole pipeline (VO, policy input, display) off a previously recorded file instead of a live camera.
- `--fc-serial <dev>` — *(USE_RELAY builds only)* open the flight-controller serial link, e.g. `--fc-serial /dev/ttyDB`. Omit it to run without the FC link (just capture + encode). See below.
- `--fc-baud <rate>` — *(USE_RELAY builds only)* FC serial baud rate. Defaults to `500000`.
- `--log <path.csv>` — log time-stamped mocap and VO poses to a CSV for offline accuracy comparison. See [Logging poses](#logging-poses).
- `--no-depth-filter` — disable the depth post-processing on the CNN input (on by default). See [Depth filtering](#depth-filtering).
- `--view <stage>` — which stage the preview window shows, for isolating latency/choppiness: `raw` (640×480 sensor depth), `filtered` (after the filter chain), `small` (64×48 min-pooled), or `cnn` (the 12×16 encoder input, default).
- `--rate <hz>` — loop rate, default 20 (the policy rate). Raise it (e.g. `--rate 30`) to test whether the cap is the choppiness.

Note: the color stream is only enabled in `USE_VO` builds (VO is its only consumer). Depth + color at 640×480@30 is ~370 Mbps and does not fit USB2 — a depth-only build fits comfortably, but any `USE_VO` build needs a **USB3** connection (`rs-enumerate-devices | grep "Usb Type"` should report `3.x`).

```
./build/depthcam --record session.bag                     # on the robot
./build/depthcam --replay session.bag                     # later, to review
./build/depthcam --fc-serial /dev/ttyDB                   # bridge to the FC at 500000 baud
./build/depthcam --fc-serial /dev/ttyDB --fc-baud 921600
```

## Depth filtering

The policy was trained on clean, dense, ray-traced depth, but the raw D435 stream
is noisy and full of holes (dropouts on textureless / dark / reflective surfaces,
object edges, and anything closer than the sensor's blind zone). Fed raw, those
holes read as free space (the build maps invalid pixels to max range), so real
obstacles can vanish from the policy's view — one reason avoidance is weaker on
hardware than in HITL.

To close that gap, the CNN depth input runs through the librealsense
post-processing chain before downsampling: depth→disparity, spatial then temporal
smoothing, disparity→depth, and hole-filling (nearest-from-around, so a dropout
next to an obstacle reads near rather than through it). This is **on by default**;
`--no-depth-filter` turns it off for A/B comparison. The visual-odometry path
deliberately uses the *unfiltered* depth.

The filters are stateful across frames (the temporal one keeps history) and add a
few ms per frame — watch the loop-overrun log if the SBC is tight.

The 640×480 → 64×48 downsample that follows is a **hole-aware min-pool**: each
10×10 block takes the nearest valid (non-zero) depth, so the closest obstacle in
every cell survives and dropouts are ignored. (A plain nearest-neighbour resize
sampled one pixel per block and could land on a hole or skip a thin/near object.)

## Flight-controller bridge

In a `USE_RELAY=1` build, passing `--fc-serial` turns this program into the
on-board bridge to the indiflight flight controller — the drone-side replacement
for the standalone relay that HITL uses. It owns the FC serial port (single
writer) and, on a background thread:

- reads `EKF_INPUTS` from the FC for the `time_us` timestamp base;
- relays UDP inputs to the FC: motion-capture pose on port **5005** →
  `FAKE_GPS` + `EXTERNAL_POSE`, position setpoints on **5006** → `POS_SETPOINT`,
  keyboard input on **5007** → `KEYBOARD`;
- transmits each CNN feature vector as two `NN_INPUT_CHUNK` messages (64 floats,
  32 per chunk).

Unlike the HITL relay, it does **not** listen for features on the network: they
come straight from the on-board CNN. The camera/CNN loop hands each new vector to
the relay thread, which transmits it on its own schedule, so the serial link
never stalls the capture loop.

`SIGUSR1` prints the last received pi-protocol messages; `SIGUSR2` prints parser
stats.

## Logging poses

`--log poses.csv` records the mocap pose (received by the relay), the VO estimate,
and a marker for every processed depth frame — all on one shared timeline, so you
can plot them together, gauge VO accuracy against the mocap ground truth, and sync
either against a recorded depth `.bag`:

```
./build/depthcam --fc-serial /dev/ttyDB --record session.bag --log poses.csv
```

Columns: `t_s,source,x,y,z,qw,qx,qy,qz,frame,rs_ts_ms`. `t_s` is seconds since the
log opened — a single monotonic timeline shared by all rows, so everything aligns
directly. `source` is one of:

- **`mocap`** — ground-truth pose (`x..qz` filled). Needs a `USE_RELAY` build with
  `--fc-serial` and mocap packets on UDP 5005.
- **`vo`** — visual-odometry pose (`x..qz` filled). Needs a `USE_VO` build; only
  appears when VO produces a pose.
- **`depth`** — one row per processed depth frame (`frame`, `rs_ts_ms` filled),
  where `frame` is the RealSense frame number and `rs_ts_ms` its hardware
  timestamp. Always logged.

### Syncing with the depth video

The `depth` rows are the bridge between a `--record` `.bag` and the pose timeline:
each `.bag` frame (matched by its frame number) has a `t_s`, and from there you
find the nearest `mocap`/`vo` pose. So to build an overlay video, iterate the
`.bag` frames, look up each frame number in the log to get its `t_s`, and
interpolate the pose at that time.

Frames differ between the pose sources, so **align them before comparing**: mocap
is NED (as forwarded to the FC as `EXTERNAL_POSE`); VO reports position in its own
frame with an FRD body quaternion. A quick trajectory plot:

```python
import pandas as pd, matplotlib.pyplot as plt
df = pd.read_csv("poses.csv")
for src in ("mocap", "vo"):
    g = df[df.source == src]
    plt.plot(g.x, g.y, label=src)
plt.legend(); plt.axis("equal"); plt.show()

# pose at a given depth frame number N:
frames = df[df.source == "depth"]
t = frames.loc[frames.frame == N, "t_s"].iloc[0]
vo = df[df.source == "vo"]
row = vo.iloc[(vo.t_s - t).abs().argmin()]   # nearest VO pose to that frame
```

### pi-protocol

The serial wire format is defined by pi-protocol. The **generated** C/H that the
build compiles is checked in at `src/pi_protocol/`, so a normal build needs no
codegen step. The library it is generated from is vendored at `ext/pi-protocol/`.
If the protocol changes, edit `ext/pi-protocol/` and regenerate:

```
make regen-pi        # needs python3 + jinja2 (ext/pi-protocol/python/requirements.txt)
```

The `config.yaml` version and message set must match the firmware — a mismatch
silently corrupts framing. See `src/pi_protocol/README.md` for details.
