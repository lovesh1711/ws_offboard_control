#!/usr/bin/env python3
"""Generate a visually richer Gazebo world (worlds/demo.sdf) for the demo video.

Takes PX4's stock default.sdf (so every physics/sensor/GUI plugin is preserved
verbatim), renames the world to "demo", gives the ground a nicer colour, and
rings the flight area with a skyline of coloured "building" boxes.

The drones operate in x in [0,18], y in [-12,12] at 5-13 m altitude, so all
buildings are kept OUTSIDE a clear central box (|x|<=22 near the paths) to avoid
any collision — they are pure backdrop scenery.

Run:  python3 worlds/make_demo_world.py   (re-run any time to regenerate)
"""
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
PX4_DIR = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))
SRC = os.path.join(PX4_DIR, "Tools/simulation/gz/worlds/default.sdf")
DST = os.path.join(HERE, "demo.sdf")

# Deterministic layout so the video looks the same every run.
random.seed(7)

# (ambient/diffuse) colours for a mixed-material skyline.
PALETTE = [
    (0.26, 0.34, 0.46),   # glass blue
    (0.40, 0.44, 0.49),   # steel grey
    (0.56, 0.55, 0.52),   # concrete
    (0.60, 0.50, 0.36),   # warm tan
    (0.72, 0.72, 0.74),   # off-white
    (0.30, 0.42, 0.40),   # teal
]


def building(name, x, y, w, d, h, col):
    r, g, b = col
    wr, wg, wb = min(r + 0.15, 1), min(g + 0.15, 1), min(b + 0.15, 1)  # lit windows
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {h/2:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry>
          <material>
            <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
            <specular>{wr:.3f} {wg:.3f} {wb:.3f} 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def make_buildings():
    out, n = [], 0
    # Place buildings along four sides, ringing the (cleared) central flight box.
    # Each entry: (fixed-axis constant, axis to sweep, sweep range, which coord is fixed)
    rows = [
        ("x", -32, "y", range(-30, 31, 9)),   # back skyline (behind launch)
        ("x",  38, "y", range(-30, 31, 9)),   # front skyline (beyond targets)
        ("y",  30, "x", range(-24, 39, 9)),   # left side
        ("y", -30, "x", range(-24, 39, 9)),   # right side
    ]
    for fixed_axis, fixed_val, _sweep_axis, sweep in rows:
        for s in sweep:
            n += 1
            x = fixed_val if fixed_axis == "x" else s
            y = fixed_val if fixed_axis == "y" else s
            # jitter + variety
            x += random.uniform(-2.5, 2.5)
            y += random.uniform(-2.5, 2.5)
            w = random.uniform(4.0, 8.0)
            d = random.uniform(4.0, 8.0)
            h = random.uniform(10.0, 34.0)
            col = random.choice(PALETTE)
            out.append(building(f"building_{n}", x, y, w, d, h, col))
    return "".join(out)


def main():
    with open(SRC, "r", encoding="utf-8-sig") as f:
        world = f.read()

    world = world.replace('<world name="default">', '<world name="demo">', 1)

    # Sky blue background instead of the washed-out grey.
    world = world.replace(
        "<background_color>0.8 0.8 0.8</background_color>",
        "<background_color>0.55 0.72 0.90</background_color>",
    )

    # Give the ground a pleasant, less-washed-out colour.
    world = world.replace(
        "<ambient>0.8 0.8 0.8 1</ambient>\n            <diffuse>0.8 0.8 0.8 1</diffuse>\n            <specular>0.8 0.8 0.8 1</specular>",
        "<ambient>0.30 0.42 0.28 1</ambient>\n            <diffuse>0.34 0.46 0.30 1</diffuse>\n            <specular>0.10 0.12 0.10 1</specular>",
    )

    world = world.replace("</world>", make_buildings() + "  </world>")

    with open(DST, "w", encoding="utf-8") as f:
        f.write(world)
    print(f"wrote {DST}")


if __name__ == "__main__":
    main()
