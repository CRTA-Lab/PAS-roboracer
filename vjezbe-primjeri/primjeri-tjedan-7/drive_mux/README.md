# Drive mux

Lets you test the safety node in the simulator.

Without a mux the controller and the safety node both publish on `/drive`.
The simulator takes whichever message came last, so the brake command is
overwritten a few milliseconds later and the car never stops. The mux gives
the safety node priority, the same way `ackermann_mux` does on the real car.

```
/drive   (controller, low priority)  --+
                                       +--> drive_mux --> /ackermann_cmd --> simulator
/safety  (safety node, high priority) -+
```

A `/safety` command always goes through. After it, `/drive` is blocked for
`safety_timeout` seconds (default 0.5).

## Build

```bash
cd /sim_ws && colcon build --symlink-install --packages-select drive_mux && source install/setup.bash
```

## Run

Terminal 1, simulator and mux together (use this **instead of**
`ros2 launch f1tenth_gym_ros gym_bridge_launch.py`):

```bash
ros2 launch drive_mux sim_mux_launch.py
```

All simulator arguments still work (`map_path:=maps/levine`, `rviz:=false`, ...),
plus `safety_timeout:=1.0`.

Terminal 2, the safety node, with its output moved from `/drive` to `/safety`:

```bash
ros2 run safety_node safety_node --ros-args -r drive:=safety
```

Terminal 3, anything that drives the car on `/drive`:

```bash
ros2 run wall_follow wall_follow
```

```bash
ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 2.0}}"
```

The mux logs `safety took over, drive is blocked` and
`safety is silent, drive is back` when control changes hands.

## Good to know

- Keyboard teleop (`/cmd_vel`) goes straight into the simulator and is **not**
  blocked by the mux.
- Once the safety node brakes, it holds the brake (it keeps sending on
  `/safety`), so `/drive` stays blocked. To release it, drive the car away by
  hand with the keyboard teleop:

  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

  If a controller is still publishing on `/drive`, it gets the car back
  `safety_timeout` seconds after the release.
