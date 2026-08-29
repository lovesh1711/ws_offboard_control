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

- **Per drone:** a flight controller (Pixhawk-class, **PX4 1.17 recommended**)
  with a **Raspberry Pi** companion (**Ubuntu 22.04 / Jammy**) wired to **TELEM2**.
- **For multiple drones:** repeat the above per drone and keep them **all on the
  same PX4 firmware** — your laptop is the central station (see §9).
- **For simulation:** any Ubuntu 22.04 PC (a laptop is fine).

> **Ubuntu / ROS version note.** Ubuntu 22.04's codename is **Jammy**, and the
> matching ROS 2 release is **Humble** — that's what this project uses. Don't
> confuse it with **Jazzy**, which is a *different* ROS 2 release for Ubuntu
> 24.04. On 22.04 you install **Humble** (`ros-humble-*`)

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

> **If `apt update` / `apt upgrade` fails with `Network is unreachable` on IPv6
> addresses** (e.g. `ports.ubuntu.com:80 (2620:2d:4002:1::10c) ... Network is
> unreachable`, ending in `no longer has a Release file`), your network doesn't
> route IPv6. This is a common cause of a *failed / incomplete ROS 2 install* on
> a Raspberry Pi, because apt can't fetch the base Ubuntu packages. Force apt to
> use IPv4 and re-run:
>
> ```bash
> echo 'Acquire::ForceIPv4 "true";' | sudo tee /etc/apt/apt.conf.d/99force-ipv4
> sudo apt update && sudo apt full-upgrade -y
> ```
>
> Do this **before** installing ROS 2 in §3.

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
>
> **If `make` fails with `Could not resolve host: github.com` or IPv6
> `Network is unreachable`** — `make` git-clones the agent's dependencies, so a
> broken-IPv6/DNS network stops it (same root cause as §2). Prefer IPv4, fix DNS
> if needed, then rebuild from a clean `build/`:
> ```bash
> sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1
> ping -4 -c3 github.com     # if it can't resolve: sudo resolvectl dns <iface> 8.8.8.8 1.1.1.1
> cd ~/Micro-XRCE-DDS-Agent && rm -rf build && mkdir build && cd build && cmake .. && make
> ```
> Or skip the source build entirely: `sudo snap install micro-xrce-dds-agent`.

---

## 5. Get and build this workspace

```bash
cd ~
git clone https://github.com/lovesh1711/ws_offboard_control.git
cd ws_offboard_control
```

### ⚠️ Match `px4_msgs` to your flight-controller firmware (critical)

`px4_msgs` (the ROS 2 message definitions) **must match the PX4 version on your
board**. If they don't, every topic will still *appear* in `ros2 topic list`, but
**no data is delivered** — you'll see errors like:

```
[RTPS_READER_HISTORY Error] Change payload size of '220' bytes is larger than
the history payload size of '207' bytes and cannot be resized.
```

(220 = what the firmware sends, 207 = what the stale message definition expects.)

**This repo vendors `px4_msgs` at `release/1.17`.** If all your flight controllers
run **PX4 1.17** (recommended — keep the whole fleet on one version), do nothing;
skip straight to the build. **Only if your firmware is a different version**,
replace it with the matching branch:

| PX4 firmware | px4_msgs branch |
|--------------|-----------------|
| 1.14.x | `release/1.14` |
| 1.15.x | `release/1.15` |
| 1.16.x | `release/1.16` |
| **1.17.x** | **`release/1.17` (vendored default)** |

```bash
# ONLY if your firmware isn't 1.17:
cd ~/ws_offboard_control/src
rm -rf px4_msgs
git clone -b release/1.15 https://github.com/PX4/px4_msgs.git   # <-- match YOUR version
rm -rf px4_msgs/.git
cd ~/ws_offboard_control
```

Then build:

```bash
# pull in ROS dependencies (first time only)
sudo rosdep init           # ignore "already exists" if you've done this before
rosdep update

# --- On the Raspberry Pi (hardware): build ONLY the flight packages ---
# The gz_truth package needs Gazebo (gz-transport), which is NOT installed on the
# Pi, so a full `colcon build` fails there. Build just what the drone needs:
rosdep install --from-paths src/px4_msgs src/multi_sim --ignore-src -r -y
colcon build --packages-select px4_msgs multi_sim

# --- On the simulation PC (has Gazebo via the PX4 setup): build everything ---
#   rosdep install --from-paths src --ignore-src -r -y
#   colcon build

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

**On the Pi — enable the GPIO serial port** (`/dev/ttyAMA0`). This is
**essential and easy to miss**: by default `ttyAMA0` is wired to **Bluetooth**,
not the GPIO pins, so the agent opens the wrong UART and receives **0 bytes**
(the FC shows `uxrce_dds_client status` → `Running, disconnected`). You must free
`ttyAMA0` from Bluetooth and turn off the serial login console.

On Raspberry Pi OS: `sudo raspi-config` → *Interface* → *Serial*: login shell
**No**, hardware **Yes**. On **Ubuntu** (no `raspi-config`), do it by hand:

```bash
# 1) enable UART + move PL011 (ttyAMA0) to the GPIO pins by disabling Bluetooth
sudo tee -a /boot/firmware/config.txt >/dev/null <<'EOF'
enable_uart=1
dtoverlay=disable-bt
EOF
sudo systemctl disable hciuart

# 2) remove the serial console from the kernel cmdline — open the file and delete
#    any "console=serial0,115200" / "console=ttyAMA0,115200" token (ONE long line,
#    leave the rest intact)
sudo nano /boot/firmware/cmdline.txt

sudo reboot
```

After reboot, `ls -l /dev/serial0` should point to `ttyAMA0`, and the agent on
`/dev/ttyAMA0` will see the flight controller.

Also add your user to the `dialout` group so the agent (and ROS nodes) can open
the serial port **without `sudo`**:

```bash
sudo usermod -aG dialout $USER
newgrp dialout      # apply to the current shell (or log out and back in)
```

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

**Reading topic data — QoS gotchas.** PX4 publishes **best-effort**, so a plain
`ros2 topic echo` shows nothing; you must add `--qos-reliability best_effort`.
Low-rate / publish-on-change topics (e.g. `vehicle_status`) also need
`--qos-durability transient_local` to see the last cached sample. (`ros2 topic
hz` in Humble does **not** accept `--qos-*` flags, so use `echo`.)

```bash
# high-rate topic — streams continuously (best proof the link works end-to-end):
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort

# low-rate topic — add transient_local to grab the cached value:
ros2 topic echo /fmu/out/vehicle_status_v1 --qos-reliability best_effort --qos-durability transient_local
```

If these print `RTPS_READER_HISTORY ... payload size` errors instead of data,
`px4_msgs` doesn't match your firmware — see §5.

### Arm test (on the ground, **props off**)

> **Indoors (no GPS), `arm_safe` will refuse to arm** — the preflight checks
> require a healthy GPS/EKF. That's correct behaviour, not a fault. For an indoor
> bench check use `arm_test` (force-arm); use `arm_safe`/`hover_test` **outdoors
> with GPS lock** (`vehicle_local_position` shows `xy_global: true`).

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

## 9. Multiple drones from one laptop (central station)

The whole point of this project: your **laptop is the ground station**, and it
flies *N* real drones at once — exactly like the Gazebo demo.

```
              Wi-Fi LAN  ·  one ROS 2 graph  ·  ROS_DOMAIN_ID = 0
   ┌────────────────┬──────────────────────┬──────────────────────┐
[ Laptop ]      [ Pi #1 ]               [ Pi #2 ]
 mission_        MicroXRCEAgent          MicroXRCEAgent
 setpoints        (serial → FC)           (serial → FC)
 node                │                        │
 (commands       [ FC #1 / PX4 ]         [ FC #2 / PX4 ]
  both)           /uav_0/fmu/...          /uav_1/fmu/...
```

Each drone's Pi runs an agent that bridges its FC onto Wi-Fi; the laptop joins the
same ROS 2 graph and runs **one** `mission_setpoints` node addressing each drone
by **namespace**. (ROS 2 discovery is distributed — every machine sees *all*
topics from *all* drones; each Pi showing `uav_0` **and** `uav_1` is normal.)

> **Keep every drone on the same PX4 firmware — 1.17 recommended.** Mixed
> versions name topics differently (1.16 uses `/fmu/out/vehicle_local_position`,
> 1.17 uses `..._v1`) and can need different `px4_msgs`. One uniform version means
> one `px4_msgs`, one set of commands, and `UXRCE_DDS_NS_IDX` available on all
> (it exists only on 1.17+).

### Per-drone PX4 params (QGC) — unique per vehicle, reboot each FC

| parameter | Drone 1 | Drone 2 | purpose |
|-----------|---------|---------|---------|
| `MAV_SYS_ID` | 1 | 2 | unique vehicle id |
| `UXRCE_DDS_KEY` | 1 | 2 | unique DDS client key |
| `UXRCE_DDS_NS_IDX` | 0 | 1 | **namespace** → `/uav_0`, `/uav_1` (PX4 1.17+) |
| `UXRCE_DDS_DOM_ID` | 0 | 0 | same domain → laptop sees both |
| `UXRCE_DDS_CFG` | TELEM2 | TELEM2 | serial port |
| `SER_TEL2_BAUD` | 921600 | 921600 | baud |

Check the real namespace after reboot with `ros2 topic list` (it's `/uav_<idx>`).

### Networking
- Laptop + all Pis on the **same Wi-Fi/subnet**, same `ROS_DOMAIN_ID` (default 0).
- `px4_msgs` on the **laptop** must match the drones' firmware (see §5).

### Bring-up (staged — do NOT skip to the full mission)

**1) Start an agent on each Pi:**
```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

**2) From the laptop, confirm both drones are visible:**
```bash
ros2 topic list | grep fmu        # expect BOTH /uav_0/fmu/... and /uav_1/fmu/...
```

**3) Arm each drone by namespace** (props off — note the `-p ns:=`):
```bash
ros2 run multi_sim arm_test --ros-args -p ns:=/uav_0 -p sysid:=1 -p hold_s:=5.0
ros2 run multi_sim arm_test --ros-args -p ns:=/uav_1 -p sysid:=2 -p hold_s:=5.0
```

**4) Simultaneous hover** (outdoors, GPS, RC + kill per drone):
```bash
ros2 run multi_sim hover_test --ros-args -p ns:=/uav_0 -p sysid:=1 -p alt:=3.0 -p hold_s:=5.0
ros2 run multi_sim hover_test --ros-args -p ns:=/uav_1 -p sysid:=2 -p alt:=3.0 -p hold_s:=5.0
```

**5) Full multi-drone mission** — one node, one config (`ns sysid wait tx ty alt`):
```bash
cat > /tmp/mission.txt <<'EOF'
/uav_0 1 0  8  5  4
/uav_1 2 0  8 -5  4
EOF
MISSION_CONFIG_FILE=/tmp/mission.txt ros2 run multi_sim mission_setpoints
```

> **Topic versioning.** The nodes read `_v1` out-topics by default (PX4 1.17). If a
> machine runs **older/unversioned** PX4 (e.g. an old SITL), set
> `PX4_OUT_SUFFIX=""` before running the node.

### Multi-drone caveats (different from Gazebo)
- **Wi-Fi is your control link.** Offboard needs setpoints > 2 Hz continuously; a
  Wi-Fi dropout triggers PX4's offboard failsafe. Fly where signal is strong, and
  keep **each drone's RC + kill switch** ready.
- **DDS discovery uses multicast** — some routers block client-to-client
  multicast, so the laptop may not see the Pis. If `ros2 topic list` is empty
  across machines, use a dedicated router / hotspot, or a FastDDS discovery server.
- Each drone needs its **own GPS lock**; start low and well separated.

---

## Troubleshooting

| symptom | fix |
|---------|-----|
| `apt update` fails with `Network is unreachable` on IPv6 addresses / `no longer has a Release file` | your network doesn't route IPv6 — force apt to IPv4: `echo 'Acquire::ForceIPv4 "true";' \| sudo tee /etc/apt/apt.conf.d/99force-ipv4`, then re-run `sudo apt update` |
| `apt: Unable to locate package ros-humble-*` | the ROS 2 apt repo wasn't added — redo §3c, ensure `sudo apt update` has no errors |
| `colcon build` fails: `Could not find ... gz-transport12` (package `gz_truth`) | you're building on the Pi (no Gazebo) — build only the flight packages: `colcon build --packages-select px4_msgs multi_sim` |
| `ros2: command not found` | you didn't source ROS: `source /opt/ros/humble/setup.bash` |
| `MicroXRCEAgent: open device error ... errno: 13 ... superuser privileges` | serial-port permission — add yourself to the `dialout` group: `sudo usermod -aG dialout $USER`, then `newgrp dialout` (or log out/in) |
| agent runs but 0 bytes; FC `uxrce_dds_client status` says `Running, disconnected`; `/dev/serial0` missing | the Pi's GPIO UART isn't enabled — `ttyAMA0` is still Bluetooth. Do the `enable_uart` + `dtoverlay=disable-bt` steps in §7 and reboot |
| `ros2 topic list` shows topics but `echo` shows nothing | PX4 is **best-effort**: add `--qos-reliability best_effort`. For low-rate topics (e.g. `vehicle_status`) also add `--qos-durability transient_local`. (`ros2 topic hz` can't set QoS in Humble — use `echo`.) |
| `RTPS_READER_HISTORY ... payload size 'N' larger than ... 'M'` (e.g. 220 vs 207) | **`px4_msgs` doesn't match your firmware** — clone the matching `release/1.x` branch and rebuild; see §5 |
| status topic missing | PX4 1.16+ renamed it to `/fmu/out/vehicle_status_v1` (the nodes here already use it) |
| agent build fails on a Fast-DDS branch | pin the branch to a tag — see §4 |
| `arm_safe` won't arm indoors | preflight needs GPS/EKF — expected; use `arm_test` (force) indoors, `arm_safe`/`hover_test` outdoors with GPS lock |
| arm/hover does nothing, `arming_state` stays 0, on a namespaced drone | you didn't pass the namespace — once a drone has `UXRCE_DDS_NS_IDX` it's **not** at root: add `-p ns:=/uav_0` (match `ros2 topic list`) and `-p sysid:=<MAV_SYS_ID>` |
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
