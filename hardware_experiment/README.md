# hardware_experiment

Files for real-drone flight tests (Holybro S500 + Raspberry Pi companion).

The flight nodes themselves live in the `multi_sim` package and run on the Pi:

```bash
# on the Pi: agent over serial to the flight controller (TELEM2)
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

# then, e.g.:
ros2 run multi_sim arm_safe                        # normal arm test (checks on)
ros2 run multi_sim hover_test --ros-args -p alt:=3.0 -p hold_s:=5.0
```

Add experiment-specific configs, logs, and launch helpers here.
