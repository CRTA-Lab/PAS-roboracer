# Follow the Gap — lecture notes

Reference: [Lab 4 - Follow the Gap](https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab4.html).
The steps, the names (`reactive_node`, `preprocess_lidar`, `find_max_gap`,
`find_best_point`) and the two launch files are the ones the lab uses.

Eight small programs. Each one adds a single idea to the previous one, and
the last one drives the car around the track on its own.

## The idea in one slide

> Look at the lidar scan, cut out the closest obstacle, find the biggest
> piece of free space that is left (the "max gap"), steer towards the best
> point in it.

The car has no map and no memory. It decides again from scratch for every
laser scan.

The algorithm from the lab, step by step:

1. Obtain the laser scan and preprocess it.
2. Find the closest point.
3. Draw a safety bubble around the closest point and set every beam inside
   it to 0. All non-zero beams are now free space.
4. Find the max gap: the longest run of non-zero beams.
5. Find the best point in that gap.
6. Drive towards the best point: publish an `AckermannDriveStamped` on
   `/drive`.

```
read scan -> preprocess -> safety bubble -> max gap -> best point -> drive
```

## Setup

Build once inside the simulator container, then again after every edit of
`setup.py` or of a launch file:

```bash
cd /sim_ws && colcon build --symlink-install && source install/setup.bash
```

| Step | Run | What it adds | Add in RViz |
|---|---|---|---|
| 1 | `ros2 run gap_follow gap1` | read the scan | — |
| 2 | `ros2 run gap_follow gap2` | preprocess | `/scan_proc` (LaserScan) |
| 3 | `ros2 run gap_follow gap3` | closest point, safety bubble | `/scan_free` (LaserScan), `/closest` (Marker) |
| 4 | `ros2 run gap_follow gap4` | find the max gap | `/gaps` (Marker) |
| 5 | `ros2 run gap_follow gap5` | find the best point | `/target` (Marker) |
| 6 | `ros2 run gap_follow gap6` | drive | — |
| 7 | `ros2 run gap_follow gap7` | does the car fit? | `/scan_extended` (LaserScan) |
| 8 | `ros2 run gap_follow reactive_node` | slow down in turns, stop cleanly | — |

In steps 1–5 the car does not move by itself. Drive it with the keyboard
and watch the output change.

Every file has the same sections. The part that is new in a step is marked
with `<-- NEW`.

The lab tests on two maps, `levine_blocked` (empty loop) and `levine_obs`
(the loop with obstacles). Both are in this folder. Pick one when you start
the simulator:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py map_path:=maps/levine_obs
```

---

## Step 1 — Read the scan

**Idea.** The lidar sends a `LaserScan` message: a list of distances, one
per beam. The message does not store the angle of each beam, only where
the first beam points and how far apart the beams are. We compute the rest.

```python
ranges = np.array(scan.ranges)
angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
```

- Our lidar: 1080 beams over 270°, one beam every 0.25°.
- Angle 0 is straight ahead. Positive angles are to the left.
- From here on, `ranges[i]` and `angles[i]` describe the same beam.

**You should see.** One line per second in the terminal: number of beams,
angle range, distance straight ahead, closest obstacle.

**Try.** Drive towards a wall. Which number changes? Turn the car so the
wall is on the left. What happens to the angle of the closest obstacle?

---

## Step 2 — Preprocess

**Idea.** The raw scan is not ready to use. The car drives forward, so
beams that look sideways and backwards only confuse it: we keep the 180°
in front. A beam that hits nothing reports `inf`, which breaks arithmetic.
And whether a wall is 9 m or 12 m away makes no difference to the car, so
every distance is capped at `max_range` (8 m by default).

```python
front = np.abs(angles) <= self.fov / 2
ranges = ranges[front]
angles = angles[front]
ranges = np.nan_to_num(ranges, nan=self.max_range, posinf=self.max_range)
ranges = np.minimum(ranges, self.max_range)
```

- `front` is a mask: an array of `True`/`False`, one per beam.
- 720 of the 1080 beams remain, from -90° to +90°.

**You should see.** In RViz, `/scan_proc` covers only the half circle in
front of the car and stops `max_range` metres away.

**Try.** Run with `--ros-args -p max_range:=3.0`. How much of a long
hallway is now "equally far"?

---

## Step 3 — Closest point and safety bubble

**Idea.** The closest point is the most dangerous one. We draw a circle
around it, the *safety bubble*, and set every beam that points into the
bubble to 0. A beam with the value 0 means "do not go there". Every beam
that is still non-zero is *free space*.

```python
closest = int(np.argmin(ranges))
distance = max(ranges[closest], 0.05)
width = math.ceil(self.bubble_radius / (distance * angle_increment))

free = ranges.copy()
free[max(closest - width, 0):closest + width + 1] = 0.0
```

- `bubble_radius` = 0.3 m.
- `width` is a number of beams. It comes from the arc length formula
  `arc = radius * angle`, so `angle = bubble_radius / distance`.
- A near obstacle covers many beams, a far one only a few.

**You should see.** In RViz a red dot sits on the closest point, and
`/scan_free` has a hole around it.

**Try.** Drive towards an obstacle on `levine_obs`. When does the red dot
jump from the wall onto the obstacle? Run with
`--ros-args -p bubble_radius:=0.6` and watch the hole.

---

## Step 4 — Find the max gap

**Idea.** A *gap* is a group of non-zero beams that are next to each other.
The bubble can split the free space into two gaps, one on each side of the
obstacle. We take the one with the most beams.

```python
nonzero = np.where(free > 0.0)[0]                 # indices of free beams
breaks = np.where(np.diff(nonzero) != 1)[0] + 1   # where the run is broken
gaps = np.split(nonzero, breaks)
gap = max(gaps, key=len)
```

Example with beam indices:

```
nonzero = [3, 4, 5, 9, 10, 20]
diff    =   [1, 1, 4, 1, 10]      -> breaks after 5 and after 10
gaps    = [3, 4, 5]  [9, 10]  [20]
max gap = [3, 4, 5]
```

**You should see.** In RViz the max gap is green and the other gaps are
grey. The terminal prints how many beams are in each gap.

**Try.** Next to a side wall there is only one gap. Drive towards an
obstacle in the middle of the hallway. On which side is the green gap, and
when does it switch sides?

---

## Step 5 — Find the best point

**Idea.** Where in the max gap do we aim? The naive answer is the furthest
beam. But many beams are equally far (all the capped ones), and `argmax`
simply returns the first of them, at the edge of the open area. So we look
at all *far* beams, the ones that reach at least `far_ratio` (95 %) of the
deepest beam, take the widest group of them and aim at its middle.

```python
depth = free[gap]
far = gap[depth >= self.far_ratio * depth.max()]
breaks = np.where(np.diff(far) != 1)[0] + 1       # same trick as in step 4
widest = max(np.split(far, breaks), key=len)
best = widest[len(widest) // 2]
```

**Why not the middle of the whole gap?** A wide gap can contain a dead end,
such as a pocket in the wall or a sealed doorway. The far beams point down
the track.

**You should see.** In RViz an arrow points from the car to the best point.
The terminal prints its angle.

**Try.** Replace the body of `find_best_point` with the naive
`return gap[np.argmax(free[gap])]`. Where does the arrow point in a long
hallway?

---

## Step 6 — Drive

**Idea.** The angle of the best point becomes the steering angle. The
wheels cannot turn further than 0.34 rad (about 20°), so we limit it. The
speed is constant.

```python
steer = float(np.clip(target_angle, -self.max_steer, self.max_steer))

msg = AckermannDriveStamped()
msg.drive.steering_angle = steer
msg.drive.speed = self.speed
self.drive_pub.publish(msg)
```

**You should see.** The car drives on its own and follows the arrow. The
terminal prints the angle of the best point, the steering angle after the
limit, and the speed.

**Three problems to notice.**

1. Next to a corner or an obstacle the arrow can point right past its
   edge. The lidar is a point, but the car is about 30 cm wide.
2. The car takes corners at full speed. Try `--ros-args -p speed:=3.0`.
3. Press Ctrl+C. The car keeps rolling, because the last command it
   received still says "drive".

---

## Step 7 — Does the car fit?

**Idea.** A gap is only worth aiming at if the car fits through it. The
bubble around the closest point does not tell us that. Where the distance
jumps between two neighbouring beams, there is an edge: a corner or the
side of an obstacle. We make every edge wider by half a car width (plus a
margin), before the bubble and the gap search.

```python
jump = ranges[i + 1] - ranges[i]
if abs(jump) > self.disparity:                   # an edge (jump > 0.1 m)
    close = min(ranges[i], ranges[i + 1])        # distance to the near side
    width = math.ceil(self.clearance / (close * angle_increment))
    # overwrite `width` beams on the far side with the distance `close`
```

- `clearance` = 0.18 m: half the car width (about 0.155 m) plus a margin.
- The same arc length formula as for the bubble.

**You should see.** In RViz, `/scan_extended` has "shadows" growing
sideways from every edge. The arrow keeps away from corners, and openings
too narrow for the car are no longer far. The terminal prints how many
edges were found and how many beams were pulled in.

**Try.** Set `clearance` to 0.05 and then to 0.4. What happens in a narrow
passage with the large value?

---

## Step 8 — Drive smoothly and stop cleanly

This is `reactive_node`, the node the lab asks for.

**Idea.** A large steering angle means a sharp turn, so we slow down.

| Steering angle | Speed |
|---|---|
| below 0.1 rad (about 6°) | 100 % |
| 0.1 to 0.2 rad | 75 % |
| above 0.2 rad (about 11°) | 50 % |

```python
if abs(steer) > 0.2:
    speed = self.speed * 0.5
elif abs(steer) > 0.1:
    speed = self.speed * 0.75
else:
    speed = self.speed
```

On Ctrl+C the node sends one last command with speed 0 before it exits:

```python
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    node.publish_drive(0.0, 0.0)
```

**You should see.** The car brakes before corners, drives full laps and
stops when you press Ctrl+C. The terminal prints the steering angle and
the speed it leads to.

### One launch file per map

A fast tuning suits the empty loop, a careful one suits the obstacles. The
lab starts the node with one launch file per map, each with its own
parameter values:

```bash
ros2 launch gap_follow levine_blocked_launch.py
```

```bash
ros2 launch gap_follow levine_obs_launch.py
```

The launch files start only our node. The simulator runs separately.

| | `levine_blocked` | `levine_obs` |
|---|---|---|
| `speed` | 3.0 m/s | 1.0 m/s |

With these values the car laps `levine_blocked` without touching a wall.
On `levine_obs`, started as in the lab (`sx: 9.96`, `sy: 2.8`,
`stheta: 1.5708`), it gets through three corners and past the dead-end
pocket on the north hallway, then crashes at the fourth corner, where a
box leaves only a narrow passage. Getting through that one is the lab's
bonus.

---

## Rules from the lab

- The node reads only `/scan` and publishes on `/drive`. The other topics
  it publishes (`/scan_proc`, `/gaps`, ...) are its own, for RViz.
- It must not read anything else from the simulator: no odometry, no
  `/tf`, no lap counter.
- Package `gap_follow`, executable `reactive_node`, launch files
  `levine_blocked_launch.py` and `levine_obs_launch.py`.

## Numbers you can tune

| Name | Value | Meaning | Too small | Too large |
|---|---|---|---|---|
| `max_range` | 8.0 m | further than this counts as equally far | a dead end looks as good as the track | only a few beams count as far, the target jumps |
| `bubble_radius` | 0.3 m | bubble around the closest point | drives close past the closest obstacle | the bubble hides the way through |
| `clearance` | 0.18 m | how much every edge is widened | clips corners | narrow passages look closed |
| `far_ratio` | 0.95 | share of the deepest beam that counts as far | aims at the middle of everything | aims at a single beam |
| `disparity` | 0.1 m | jump that counts as an edge | noise is treated as edges | real edges are missed |
| `fov` | 180° | part of the scan we look at | misses sharp turns | reacts to things beside the car |
| `speed` | 1.0 m/s | top speed | — | no time to react |

## Questions for discussion

1. The car decides from one scan at a time. What kind of situation can that
   not handle?
2. In a plain hallway the closest point is on a side wall. How much does
   the safety bubble change there? Where does it matter?
3. The speed depends only on the steering angle. What other information in
   the scan could set the speed?
4. At the fourth corner of `levine_obs` the car crashes. Look at the scan
   in RViz just before: which step of the pipeline makes the wrong choice?
5. What happens if another car drives in front of ours?
