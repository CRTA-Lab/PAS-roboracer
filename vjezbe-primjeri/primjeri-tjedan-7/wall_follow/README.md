# Wall follow, step by step

The car drives along a corridor and keeps the wall on its **left** side at a
fixed distance. We build the node in five small steps. Every step is a
complete node that you can run, and every step adds exactly one idea.

```
scan -> two beams -> wall angle + distance -> look ahead, error -> PID -> drive
        step 1       step 2                   step 3               step 4  step 5
```

To see what a step added, compare it with the previous one:

```bash
diff wall_follow/wall_follow3_error.py wall_follow/wall_follow4_pid.py
```

or search for `<-- NEW` in the file.

## Build and run

```bash
cd /sim_ws
colcon build --symlink-install --packages-select wall_follow
source install/setup.bash
ros2 run wall_follow wall1
```

| Step | Command | File | New output |
|---|---|---|---|
| 1 | `ros2 run wall_follow wall1` | `wall_follow1_beams.py` | log: `a`, `b` |
| 2 | `ros2 run wall_follow wall2` | `wall_follow2_wall.py` | RViz: `/wall_line` |
| 3 | `ros2 run wall_follow wall3` | `wall_follow3_error.py` | log: `distance`, `alpha`, `future`, `error` |
| 4 | `ros2 run wall_follow wall4` | `wall_follow4_pid.py` | RViz: `/steering` |
| 5 | `ros2 run wall_follow wall5` | `wall_follow5_drive.py` | the car drives (`/drive`) |
| final | `ros2 run wall_follow wall_follow` | `wall_follow_node.py` | same as step 5 |

The car does **not** move in steps 1 to 4. To move it by hand use the
"2D Pose Estimate" button in RViz: click where the car should be and drag
in the direction it should face.

Use a map without obstacles (`levine`). The car only looks to the left,
so on `levine_obs` it drives straight into the first box in its way.

### RViz displays (Add -> By topic)

| Topic | Type | From step | What it shows |
|---|---|---|---|
| `/scan` | LaserScan | - | the raw lidar scan |
| `/wall_line` | Marker | 2 | grey: beams `a` and `b`; orange: the wall; green: shortest line to the wall |
| `/steering` | Marker | 4 | blue arrow at the steering angle the controller asks for |

`viz.py` builds these markers. It is not part of the algorithm; you do not
need to read it.

---

## Step 1 - two beams

**Idea.** The lidar gives 1080 distances. We need only two of them, both on
the left side: `b` at 90 deg (straight left) and `a` at 45 deg (left and
forward). Angle 0 is straight ahead, positive angles are to the left.

**Key code**

```python
def beam_index(self, angle):
    return round((angle - self.lidar_angle_min) / self.lidar_resolution)

a = float(scan.ranges[self.beam_index(self.beam_a_angle)])   # 45 deg
b = float(scan.ranges[self.beam_index(self.beam_b_angle)])   # 90 deg
```

**What you should see**

```
a (45 deg) = 1.62 m    b (90 deg) = 1.06 m
```

**Problem.** Turn the car on the spot. `b` changes although the car is just
as far from the wall as before. `b` is the distance to the wall only when the
car is exactly parallel to it.

**Try.** Put the car parallel to a wall and compute `a / b` from the log.
Which number do you expect, and why?

---

## Step 2 - where is the wall?

**Idea.** Two points define a straight line. From `a`, `b` and the angle
`theta` between the two beams we get

- `alpha`: how much the car is turned relative to the wall
  (0 = parallel, positive = heading away from the wall),
- `distance`: the perpendicular distance to the wall.

**Key code**

```python
theta = math.radians(self.beam_b_angle - self.beam_a_angle)
alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
distance = b * np.cos(alpha)
```

**What you should see.** In RViz (`/wall_line`) the orange line lies on the
wall and the green line is the shortest way to it. Turn the car on the spot:
the green line keeps its length.

**Problem.** Place the car 0.7 m from the wall with its nose pointing at the
wall. The distance is perfect, but the car is about to crash. Knowing the
distance *now* is not enough.

**Try.** Drive the car (by hand) next to a corner or an open door. What
happens to the orange line? Is "the wall is a straight line" still true?

---

## Step 3 - look ahead, compute the error

**Idea.** The car needs time to react, so we control the distance it *will*
have after driving `look_ahead` metres straight on:

```
future_distance = distance + look_ahead * sin(alpha)
error           = wall_dist - future_distance
```

`error > 0`: the car will be too close. `error < 0`: it will be too far.

**Key code**

```python
future_distance = distance + look_ahead * np.sin(alpha)
error = wall_dist - future_distance
```

**What you should see**

```
distance = 1.06 m, alpha = +4.1 deg -> future 1.13 m, error = -0.43 m
```

Repeat the test from step 2: the distance is still 0.7 m, but the error is
clearly positive now.

**Problem.** We know the error, but the car does nothing with it.

**Try.** Start the node with `-p look_ahead:=0.0` and then with
`-p look_ahead:=3.0`, each time with the car turned about 20 deg towards the
wall. How does the error change? You can also change it while the node runs:
`ros2 param set /wall_follow_node look_ahead 3.0`.

---

## Step 4 - PID controller

**Idea.** A PID controller turns the error into a steering angle.

| Term | Looks at | Formula |
|---|---|---|
| P | the error now | `kp * error` |
| I | the error added up over time | `integral += ki * dt * error` |
| D | how fast the error changes | `kd * (error - prev_error) / dt` |

The wall is on the left, so a positive error (too close) must steer to the
right, which is a negative angle. That is why all three terms get a minus.

**Key code**

```python
proportional = self.kp * error
self.integral = self.integral + self.ki * dt * error
derivative = self.kd * (error - self.prev_error) / dt
steering_angle = -proportional - self.integral - derivative
```

**What you should see.** A blue arrow in RViz (`/steering`). Car too close to
the wall: the arrow points away from it. Car too far: the arrow points
towards it. The steering angle is only drawn; nothing is sent to the car.

**Problem.** The car stands still, so the error never goes away. The I term
keeps adding it up and the arrow slowly keeps turning (about 7 deg per minute
for an error of 0.4 m). The controller asks for more and more, but nobody
listens.

**Try.** Set `self.kp = 0.0` and `self.ki = 0.5`. Watch the arrow for 20
seconds. Then move the car so that the error changes sign. How long does the
arrow need to come back? This effect is called integral windup.

---

## Step 5 - drive

**Idea.** Send the steering angle and a speed to the car, and stop the car
when the node is closed.

**Key code**

```python
msg = AckermannDriveStamped()
msg.drive.steering_angle = float(steering_angle)
msg.drive.speed = float(self.vehicle_speed)
self.drive_pub.publish(msg)
```

```python
rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
...
except KeyboardInterrupt:
    node.stop()          # speed 0, sent three times
```

**What you should see.** The car drives and settles about 0.7 m from the left
wall. The log error goes to about 0. Ctrl+C stops the car.

**Try.** Change the speed while the car drives:

```bash
ros2 topic pub --once /vehicle_speed ackermann_msgs/msg/AckermannDrive "{speed: 2.0}"
```

At which speed does the car start to wobble or miss a corner? Which number
from the tuning table would you change first?

---

## Tuning table

| Name | Where | Default | Unit | Effect |
|---|---|---|---|---|
| `wall_dist` | ROS parameter | 0.7 | m | distance the car keeps from the left wall |
| `look_ahead` | ROS parameter | 1.0 | m | larger: reacts earlier and smoother, but cuts corners |
| `vehicle_speed` | topic `vehicle_speed` | 1.0 | m/s | driving speed |
| `kp` | SETUP | 1.0 | rad/m | larger: stronger reaction, more wobble |
| `ki` | SETUP | 0.005 | rad/(m s) | removes a small error that stays; too large: windup |
| `kd` | SETUP | 0.001 | rad s/m | damps fast changes; amplifies lidar noise |
| `integral_limit` | SETUP | 20.0 | rad | the I term restarts at 0 above this value |
| `beam_a_angle` | SETUP | 45 | deg | forward-left beam |
| `beam_b_angle` | SETUP | 90 | deg | left beam |
| `lidar_angle_min` | SETUP | -135 | deg | angle of beam 0 (property of the lidar) |
| `lidar_resolution` | SETUP | 0.25 | deg | angle between two beams (property of the lidar) |

Parameters can be set on the command line or while the node runs:

```bash
ros2 run wall_follow wall5 --ros-args -p wall_dist:=1.0 -p look_ahead:=0.5
ros2 param set /wall_follow_node wall_dist 1.0
```

## Discussion questions

1. The steering angle is never limited, but the car cannot steer more than
   about 0.4 rad. What does the car do when the controller asks for 1.9 rad?
   Where would you add a limit?
2. The I term is reset to 0 only when it grows above `+20`. What happens when
   it becomes very negative? Is 20 rad a sensible limit for a steering angle?
   How would you write a better anti-windup?
3. What do `alpha` and `error` become when beam `a` returns `inf` (no wall in
   range)? Follow the value through `pid_control`: what is the I term on the
   next scan, and on every scan after that?
4. `dt` is measured with the clock of the node, not with the time stamp of
   the scan. When does that make a difference?
5. The scan in the simulator has 1080 beams over 270 deg, so the real step
   is 270/1079 = 0.2502 deg, not 0.25. How far off is beam `b` from 90 deg?
   How would you compute the index from `scan.angle_min` and
   `scan.angle_increment` instead?
6. The speed is constant. How would you slow down before a corner, using
   only numbers the node already computes?
7. The car follows the left wall. What has to change to follow the right
   wall? (Beams, the sign of `alpha`, the sign of the steering angle.)
8. The D term divides a noisy difference by a very small `dt` (4 ms).
   Why is `kd` so small? What could you do about the noise?

## For teachers: is the final node the same as the original?

The node the lessons were made from is kept in
`test/reference/wall_follow_original.py`. The script

```bash
python3 test/compare_with_original.py [recorded_scans.npz]
```

runs the original and the finished node side by side on the same scans with
the same clock and compares every drive command bit for bit.
