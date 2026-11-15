"""
Check that the finished node computes exactly what the original node did.

Both nodes are created in one process and fed the same LaserScan messages
with the same (fake) clock. Every drive command they publish is captured
and compared bit for bit.

    python3 test/compare_with_original.py                 # synthetic scans
    python3 test/compare_with_original.py recorded.npz    # + recorded scans

`recorded.npz` holds `ranges` (N x beams) and `stamps` (N, seconds), as
written by a scan recorder. The original node is kept in
test/reference/wall_follow_original.py.
"""
import importlib.util
import os
import pathlib
import struct
import sys
import warnings

os.environ.setdefault('ROS_DOMAIN_ID', '99')   # keep test traffic off the sim

from ackermann_msgs.msg import AckermannDrive   # noqa: E402
import numpy as np   # noqa: E402
import rclpy   # noqa: E402
from rclpy.parameter import Parameter   # noqa: E402
from rclpy.time import Time   # noqa: E402
from sensor_msgs.msg import LaserScan   # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
BEAMS = 1080
STEP_FILES = ['wall_follow5_drive', 'wall_follow_node']


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeClock:
    def __init__(self):
        self.ns = 0

    def now(self):
        return Time(nanoseconds=self.ns)


def instrument(node, publisher_name, clock):
    """Capture the drive commands and replace the clock of `node`."""
    sent = []
    getattr(node, publisher_name).publish = sent.append
    node.get_clock = lambda: clock
    for name in ('wall_pub', 'steering_pub'):     # RViz only, not compared
        if hasattr(node, name):
            getattr(node, name).publish = lambda msg: None
    return sent


def bits(x):
    return struct.pack('<d', float(x))


def scan_msg(ranges):
    msg = LaserScan()
    msg.header.frame_id = 'ego_racecar/laser'
    msg.angle_min = -2.356194496154785
    msg.angle_increment = 0.004367366898804903
    msg.ranges = [float(r) for r in ranges]
    return msg


def synthetic_cases(rng):
    """Yield (name, list of (stamp_ns, ranges, action))."""
    t = 1_000_000_000

    def advance(dt_s):
        nonlocal t
        t += int(dt_s * 1e9)
        return t

    # 1) random walls, random scan period
    seq = []
    for _ in range(3000):
        seq.append((advance(rng.uniform(0.001, 0.05)),
                    rng.uniform(0.05, 10.0, BEAMS), None))
    yield 'random scans', seq

    # 2) smooth "driving along a wall" signal with lidar noise
    seq = []
    for k in range(3000):
        d = 0.7 + 0.4 * np.sin(k / 150.0)
        psi = 0.3 * np.sin(k / 90.0)
        r = np.full(BEAMS, 5.0)
        r[900] = d / np.cos(psi) + rng.normal(0, 0.01)
        r[720] = d / np.cos(psi - np.pi / 4) + rng.normal(0, 0.01)
        seq.append((advance(0.004), r, None))
    yield 'smooth wall + noise', seq

    # 3) nasty values: inf, nan, zero, repeated time stamp (dt = 0)
    seq = []
    specials = [np.inf, np.nan, 0.0, 25.0, 0.05, 1.0]
    for a in specials:
        for b in specials:
            r = rng.uniform(0.5, 3.0, BEAMS)
            r[720], r[900] = a, b
            seq.append((advance(0.01), r, None))
            seq.append((advance(0.0), r, None))          # dt = 0
            for _ in range(3):                           # recover (or not)
                seq.append((advance(0.01), rng.uniform(0.5, 3.0, BEAMS), None))
    yield 'inf / nan / zero / dt=0', seq

    # 4) speed command, parameter changes, integral above its limit
    seq = [(advance(0.01), rng.uniform(0.5, 3.0, BEAMS), ('speed', 2.3))]
    for _ in range(50):
        seq.append((advance(0.01), rng.uniform(0.5, 3.0, BEAMS), None))
    seq.append((advance(0.01), rng.uniform(0.5, 3.0, BEAMS),
                ('params', (500.0, 0.3))))
    for _ in range(400):                                 # error ~ +500 m
        seq.append((advance(1.0), rng.uniform(0.5, 3.0, BEAMS), None))
    seq.append((advance(0.01), rng.uniform(0.5, 3.0, BEAMS),
                ('params', (0.7, 1.0))))
    for _ in range(200):
        seq.append((advance(0.01), rng.uniform(0.5, 3.0, BEAMS), None))
    yield 'speed topic, parameters, integral reset', seq


def recorded_case(path):
    data = np.load(path)
    stamps = (data['stamps'] * 1e9).astype(np.int64)
    return 'recorded live scans', [
        (int(t), r, None) for t, r in zip(stamps, data['ranges'])]


def run_case(name, seq, original_module, new_module):
    clock = FakeClock()
    old = original_module.WallFollow()
    new = new_module.WallFollow()
    old_sent = instrument(old, 'publisher_', clock)
    new_sent = instrument(new, 'drive_pub', clock)
    resets = 0

    for stamp, ranges, action in seq:
        clock.ns = stamp
        if action and action[0] == 'speed':
            msg = AckermannDrive(speed=action[1])
            old.vehicle_speed_callback(msg)
            new.speed_callback(msg)
        if action and action[0] == 'params':
            for node in (old, new):
                node.set_parameters([
                    Parameter('wall_dist', value=action[1][0]),
                    Parameter('look_ahead', value=action[1][1])])
        scan = scan_msg(ranges)
        before = new.integral
        old.callback_scan(scan)
        new.scan_callback(scan)
        if before != 0.0 and new.integral == 0.0:
            resets += 1

    same_count = len(old_sent) == len(new_sent)
    mismatches = 0
    for o, n in zip(old_sent, new_sent):
        if (bits(o.drive.steering_angle) != bits(n.drive.steering_angle)
                or bits(o.drive.speed) != bits(n.drive.speed)
                or o.header.frame_id != n.header.frame_id
                or o.header.stamp != n.header.stamp):
            mismatches += 1
    state_same = (bits(old.integral) == bits(new.integral)
                  and bits(old.prev_error) == bits(new.prev_error))
    nans = sum(1 for n in new_sent if not np.isfinite(n.drive.steering_angle))
    ok = same_count and mismatches == 0 and state_same
    print(f'{"PASS" if ok else "FAIL"}  {name:42s} scans {len(seq):6d}  '
          f'commands {len(old_sent):6d}/{len(new_sent):6d}  '
          f'different {mismatches}  non-finite {nans}  '
          f'integral resets {resets}')
    old.destroy_node()
    new.destroy_node()
    return ok


def main():
    warnings.simplefilter('ignore')
    np.seterr(all='ignore')
    rclpy.init()
    original = load(HERE / 'reference' / 'wall_follow_original.py', 'original')
    all_ok = True
    for step in STEP_FILES:
        print(f'--- original  vs  {step}.py')
        new = load(HERE.parent / 'wall_follow' / f'{step}.py', step)
        cases = list(synthetic_cases(np.random.default_rng(1)))
        if len(sys.argv) > 1:
            cases.append(recorded_case(sys.argv[1]))
        for name, seq in cases:
            all_ok &= run_case(name, seq, original, new)
    rclpy.try_shutdown()
    print('ALL IDENTICAL' if all_ok else 'DIFFERENCES FOUND')
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
