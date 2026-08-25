# ws_offboard_control

Multi-drone **PX4 offboard control** with **ROS 2** — a single ROS node flies *N*
drones at once (one process, a multi-threaded executor with one callback group
per drone). It is validated two ways:

- **Simulation:** Gazebo SITL, several drones taking off simultaneously to
  different targets, with a live RViz 3-D path plot.
- **Hardware:** a Holybro **S500** with a **Raspberry Pi** companion computer
  wired to the flight controller's **TELEM2** port.

---

## Repository layout

| path | contents |
|------|----------|
| `src/multi_sim/` | the ROS 2 package — all C++/Python nodes |
| `simulation/` | SITL launch scripts, a custom Gazebo "city" world, RViz path visualization |
| `hardware_experiment/` | notes/configs for real-drone flight tests |
| `src/px4_msgs/`, `src/px4_ros_com/` | PX4 ROS 2 message + bridge dependencies (vendored) |

### Nodes in `multi_sim`

| node | what it does |
|------|--------------|
| `mission_setpoints` | multi-drone mission FSM: arm → ascend → cruise to target → land |
| `arm_test` | **force**-arm test — bypasses preflight (indoor / no-GPS only, **props off**) |
| `arm_safe` | **normal** arm with full preflight checks, hold, disarm |
| `hover_test` | offboard hover: arm → climb to `alt` → hold → auto-land (outdoor / GPS) |

---

## 1. What you need

- **Companion computer:** Raspberry Pi running **Ubuntu 22.04 (Jammy)**.
- **Flight controller:** Pixhawk-class board running **PX4 v1.15+**, with the
  Pi connected to **TELEM2**.
- **For simulation:** any Ubuntu 22.04 PC (a laptop is fine).

> **Ubuntu / ROS version note.** Ubuntu 22.04's codename is **Jammy**, and the
> matching ROS 2 release is **Humble** — that's what this project uses. Don't
> confuse it with **Jazzy**, which is a *different* ROS 2 release for Ubuntu
> 24.04. On 22.04 you install **Humble** (`ros-humble-*`).

---

## 2. Prepare a fresh Ubuntu (do this first)

On a brand-new Ubuntu install, update the system and install the basic tools
**before** touching ROS. On a headless Raspberry Pi, run these over SSH.

```bash
# 1) update everything
sudo apt update && sudo apt full-upgrade -y

# 2) essential build / dev tools used throughout this guide
sudo apt install -y \
  build-essential cmake git wget curl gnupg2 lsb-release \
  python3-pip python3-venv python3-dev \
  net-tools nano

# 3) (Raspberry Pi, headless) make sure the SSH server is up so you can log in
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
```

Reboot if the upgrade pulled a new kernel:

```bash
sudo reboot
```

---

## 3. Install ROS 2 Humble (Debian packages)

This follows the official guide:
<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>
(prebuilt `.deb` packages via apt — no building from source).

```bash
# --- 3a. Set the locale to UTF-8 ---
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# --- 3b. Enable the "universe" repository ---
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# --- 3c. Add the ROS 2 apt repository (official ros2-apt-source package) ---
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# --- 3d. Install ROS 2 Humble ---
sudo apt update && sudo apt upgrade -y

# On the Raspberry Pi (no GUI needed):
sudo apt install -y ros-humble-ros-base
# On a PC that will run RViz / the Gazebo demos, use the full desktop instead:
#   sudo apt install -y ros-humble-desktop

# build tools + rosdep
sudo apt install -y ros-dev-tools python3-colcon-common-extensions
```

**Source ROS 2 in every new shell** (and make it automatic):

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**Check it works:**

```bash
ros2 --help          # should print the ros2 CLI help, not "command not found"
```

> **If apt says `Unable to locate package ros-humble-ros-base`**, the repository
> step (3c) didn't take. Re-run 3c, confirm `sudo apt update` finishes with **no
> errors**, then retry 3d.
>
> **On Ubuntu 24.04 instead?** The same steps work — the `.deb` in 3c picks your
> codename automatically; just replace `humble` with `jazzy` in 3d and when
> sourcing.

---

## 4. Install the micro-XRCE-DDS Agent

This is the uORB ↔ ROS 2 bridge that talks to PX4. Build it from source once:

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Verify:

```bash
MicroXRCEAgent --help
```

> **If the build fails** while fetching a Fast-DDS branch (e.g. `invalid
> reference: 2.12.x`), that upstream branch was removed. Pin it to a tag:
> `sed -i 's#2\.12\.x#v2.12.2#g' ../CMakeLists.txt` then re-run `cmake .. && make`.

---

## 5. Get and build this workspace

```bash
cd ~
git clone https://github.com/lovesh1711/ws_offboard_control.git
cd ws_offboard_control

# pull in any ROS dependencies of the packages (first time only)
sudo rosdep init           # ignore "already exists" if you've done this before
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build
colcon build
source install/local_setup.bash
echo "source ~/ws_offboard_control/install/local_setup.bash" >> ~/.bashrc
```

`build/`, `install/`, and `log/` are generated by colcon and are git-ignored.

---

## 6. Simulation (Gazebo SITL)

The simulation runs on a **PC** (not the Pi) and needs PX4-Autopilot + Gazebo.

**Install PX4-Autopilot** (once):

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh          # installs toolchain (re-login afterwards)
make px4_sitl gz_x500                  # builds SITL + fetches Gazebo
```

> The scripts expect PX4 at `~/PX4-Autopilot`. Override with `PX4_DIR=/path ...`.

**Run the demo** (3 drones fanning into two quadrants, custom city world):

```bash
cd ~/ws_offboard_control
./simulation/record_demo.sh
```
Follow the on-screen prompt: frame the Gazebo camera, (optionally) start Gazebo's
built-in Video Recorder, then press **ENTER** to launch the mission.

**Live 3-D path plot in RViz** (separate terminal):

```bash
./simulation/run_rviz.sh
```
Shows a color-coded trajectory per drone, a moving "Drone i" label, and
"Target i (x, y)" markers.

Other helpers: `./simulation/run_mission.sh` (interactive launcher),
`./simulation/stop_mission.sh` (kill everything),
`./simulation/record_window.sh` (record a single window).

---

## 7. Hardware setup (real drone)

**Wiring:** connect the Pi UART to the flight controller's **TELEM2**
(TX↔RX, RX↔TX, GND↔GND).

**On the Pi — enable the GPIO serial port** (`/dev/ttyAMA0`):
disable the serial login console and Bluetooth so the UART is free
(`sudo raspi-config` → *Interface* → *Serial*: login shell **No**, hardware
**Yes**; and add `dtoverlay=disable-bt` to `/boot/firmware/config.txt`), then reboot.

**In QGroundControl — set PX4 parameters** on the flight controller:

| parameter | value |
|-----------|-------|
| `UXRCE_DDS_CFG` | TELEM2 |
| `SER_TEL2_BAUD` | 921600 |

Reboot the flight controller after changing these.

---

## 8. Run on the drone: arm test & hover test

Power the drone on its battery. Open **two SSH sessions** to the Pi
(e.g. `ssh <user>@<pi-ip>`).

**Terminal 1 — start the agent** (serial link to the FC):

```bash
source /opt/ros/humble/setup.bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

**Terminal 2 — verify the link:**

```bash
source /opt/ros/humble/setup.bash
source ~/ws_offboard_control/install/local_setup.bash
ros2 topic list | grep fmu          # you should see /fmu/out/vehicle_status_v1, etc.
```

### Arm test (on the ground, **props off**)

Normal arming with all safety/preflight checks (won't arm on a bad GPS/EKF):

```bash
ros2 run multi_sim arm_safe --ros-args -p hold_s:=5.0
```

Force-arm (bypasses checks — **indoor / no-GPS bench test only, props off**):

```bash
ros2 run multi_sim arm_test --ros-args -p hold_s:=5.0
```

### Hover test (outdoor, GPS, clear area, RC + kill switch ready)

Arm → climb to `alt` metres over the takeoff spot → hover `hold_s` seconds →
auto-land + disarm:

```bash
ros2 run multi_sim hover_test --ros-args -p alt:=3.0 -p hold_s:=5.0
```

> **Safety:** the hover runs in **OFFBOARD** mode. Keep an RC transmitter bound
> with a mapped **kill switch** — flipping to any manual mode (Position /
> Altitude / Stabilized) instantly takes control back from the companion.

---

## Troubleshooting

| symptom | fix |
|---------|-----|
| `apt: Unable to locate package ros-humble-*` | the ROS 2 apt repo wasn't added — redo §3c, ensure `sudo apt update` has no errors |
| `ros2: command not found` | you didn't source ROS: `source /opt/ros/humble/setup.bash` |
| `ros2 topic list` shows topics but `echo` hangs | PX4 publishes **best-effort** QoS: `ros2 topic echo <topic> --qos-reliability best_effort` |
| topics listed but no data / build type errors | `px4_msgs` must match your PX4 firmware; this repo vendors a compatible version |
| status topic missing | PX4 1.17 renamed it to `/fmu/out/vehicle_status_v1` (the nodes here already use it) |
| agent build fails on a Fast-DDS branch | pin the branch to a tag — see §4 |
| drone arms but won't take off in `hover_test` | needs a valid GPS/EKF position estimate — run **outdoors**; check status in QGC |

---

## Parameters cheat-sheet

| node | parameter | default | meaning |
|------|-----------|---------|---------|
| `arm_safe` / `arm_test` | `sysid` | 1 | must match `MAV_SYS_ID` |
| | `hold_s` | 5.0 | seconds armed before disarm |
| `arm_test` | `force` | true | bypass preflight (force-arm magic number) |
| `hover_test` | `alt` | 3.0 | hover height (m) above takeoff |
| | `hold_s` | 5.0 | seconds to hover |
| | `sysid` | 1 | must match `MAV_SYS_ID` |
