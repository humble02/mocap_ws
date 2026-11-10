#!/usr/bin/env python3
"""Exercise the installed Jazzy bridge with local ZMQ frames; no hardware needed.

Run after sourcing source_jazzy.sh. The check uses a separate ROS domain and
localhost discovery, and starts/stops only its own processes.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=91)
    args = parser.parse_args()
    if not 0 <= args.domain <= 101 and not 215 <= args.domain <= 232:
        parser.error('Use domain 0..101 or 215..232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ.pop('CYCLONEDDS_URI', None)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ.pop('ROS_STATIC_PEERS', None)

    import rclpy
    import zmq
    from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
    from tf2_msgs.msg import TFMessage
    from fzmotion_ros import LusterFrameStruct_pb2 as pb

    root = Path(__file__).resolve().parents[1]
    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    port = pub.bind_to_random_port('tcp://127.0.0.1')
    frame = pb.Frame()
    frame.FrameId = 1
    body = frame.rigidBodys.add()
    body.Name = 'box'
    body.IsTrack = True
    body.X, body.Y, body.Z = 1000., 2000., 3000.
    s = math.sqrt(0.5)
    body.qx, body.qy, body.qz, body.qw = 0., s, 0., s
    body.speeds.XfSpeed, body.speeds.YfSpeed, body.speeds.ZfSpeed = .5, 1., 1.5
    body.palstance.fZPalstance = .3
    body.acceleratedSpeeds.XfAcceleratedSpeed = 10.
    body.acceleratedSpeeds.YfAcceleratedSpeed = 20.
    body.acceleratedSpeeds.ZfAcceleratedSpeed = 30.
    body.accpalstance.AccfZPalstance = 2.

    for index, name in enumerate(('robot', 'target'), start=1):
        other = frame.rigidBodys.add()
        other.CopyFrom(body)
        other.Name = name
        other.X += index * 1000.

    rclpy.init()
    observer = rclpy.create_node('mocap_jazzy_check')
    received = {f'{role}/{kind}': [] for role in ('box', 'robot', 'target')
                for kind in ('pose', 'velocity', 'acceleration')}
    received['tf'] = []
    subscriptions = []
    for role in ('box', 'robot', 'target'):
        for kind, cls in (('pose', PoseStamped), ('velocity', TwistStamped), ('acceleration', AccelStamped)):
            key = f'{role}/{kind}'
            subscriptions.append(observer.create_subscription(cls, '/fzmotion/' + key,
                lambda msg, key=key: received[key].append(msg), 100))
    subscriptions.append(observer.create_subscription(TFMessage, '/tf', lambda msg: received['tf'].append(msg), 100))

    def pump(seconds, payload=None):
        deadline = time.monotonic() + seconds
        next_send = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if payload is not None and now >= next_send:
                pub.send(payload)
                next_send += .02
            rclpy.spin_once(observer, timeout_sec=.005)

    def vector(actual, expected):
        values = [actual.x, actual.y, actual.z]
        assert all(math.isclose(x, y, abs_tol=1e-5) for x, y in zip(values, expected)), (values, expected)

    results = []
    try:
        with tempfile.TemporaryDirectory(prefix='mocap_jazzy_') as directory:
            config = Path(directory) / 'mock.yaml'
            config.write_text(f'''fzmotion_node:
  ros__parameters:
    ip: "127.0.0.1"
    port: {port}
    frame_id: "mock_world"
    yaw_offset_deg: 0.0
''')
            cases = (
                ('run.sh', [str(root / 'run.sh'), '--ros-args', '-p', 'yaw_offset_deg:=90.0'], True),
                ('ros2 run', ['ros2', 'run', 'fzmotion_ros', 'fzmotion_node', '--ros-args', '--params-file', str(config)], False),
                ('ros2 launch', ['ros2', 'launch', 'fzmotion_ros', 'fzmotion.launch.py', f'params_file:={config}'], False),
            )
            for index, (name, command, turned) in enumerate(cases):
                pump(.15)
                for messages in received.values():
                    messages.clear()
                env = dict(os.environ, FZMOTION_CONFIG=str(config))
                logfile = Path(directory) / f'node_{index}.log'
                with logfile.open('w') as log:
                    proc = subprocess.Popen(command, cwd='/tmp', env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    try:
                        deadline = time.monotonic() + 12
                        while min(map(len, received.values())) < 10 and time.monotonic() < deadline:
                            assert proc.poll() is None, logfile.read_text()
                            pump(.1, frame.SerializeToString())
                        assert min(map(len, received.values())) >= 10, (name, {k: len(v) for k, v in received.items()}, logfile.read_text())
                        # DDS delivers the topics through separate callbacks. Drain
                        # outstanding callbacks before comparing the last full frame.
                        pump(.15)
                        pose = received['box/pose'][-1]
                        velocity = received['box/velocity'][-1]
                        accel = received['box/acceleration'][-1]
                        tf = next(m.transforms[0] for m in reversed(received['tf']) if m.transforms[0].child_frame_id == 'box')
                        vector(pose.pose.position, (3., 1., 2.) if turned else (1., -3., 2.))
                        vector(velocity.twist.linear, (1.5, .5, 1.) if turned else (.5, -1.5, 1.))
                        vector(velocity.twist.angular, (0., .3, 0.) if turned else (.3, 0., 0.))
                        vector(accel.accel.linear, (30., 10., 20.) if turned else (10., -30., 20.))
                        vector(accel.accel.angular, (0., 2., 0.) if turned else (2., 0., 0.))
                        q = pose.pose.orientation
                        assert all(math.isclose(x, y, abs_tol=1e-5) for x, y in zip((q.x, q.y, q.z, q.w), (0., 0., s, s)))
                        assert tf.child_frame_id == 'box'
                        for msg in (pose, velocity, accel, tf):
                            assert msg.header.frame_id == 'mock_world'
                        stamp = pose.header.stamp
                        key = (stamp.sec, stamp.nanosec)
                        for topic in ('box/velocity', 'box/acceleration', 'robot/pose', 'target/pose'):
                            assert any((m.header.stamp.sec, m.header.stamp.nanosec) == key for m in received[topic])
                        assert any((m.transforms[0].header.stamp.sec, m.transforms[0].header.stamp.nanosec) == key for m in received['tf'])
                        for index, role in enumerate(('box', 'robot', 'target')):
                            point = received[f'{role}/pose'][-1].pose.position
                            vector(point, (3., 1.+index, 2.) if turned else (1.+index, -3., 2.))
                            for kind in ('velocity', 'acceleration'):
                                assert received[f'{role}/{kind}'][-1].header.frame_id == 'mock_world'
                        age = (observer.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) / 1e9
                        assert 0 <= age < 1., age
                        count = len(received['box/pose'])
                        pump(1., frame.SerializeToString())
                        rate = len(received['box/pose']) - count
                        assert 30 <= rate <= 65, rate

                        # Bad packets must not terminate the node, and loss of tracking
                        # or data must not keep publishing old poses with fresh stamps.
                        for payload in (b'\xff\xff', None):
                            pump(.2, payload)
                            count = len(received['box/pose'])
                            pump(.3, payload)
                            assert len(received['box/pose']) == count
                            assert proc.poll() is None, logfile.read_text()
                        robot_count = len(received['robot/pose'])
                        body.IsTrack = False
                        pump(.2, frame.SerializeToString())
                        count = len(received['box/pose'])
                        pump(.3, frame.SerializeToString())
                        assert len(received['box/pose']) == count
                        assert len(received['robot/pose']) > robot_count, 'One lost body must not suppress another'
                        body.IsTrack = True
                        pump(.3, frame.SerializeToString())
                        assert len(received['box/pose']) > count
                        # A position-only target is sufficient even without a quaternion.
                        target = frame.rigidBodys[2]
                        saved_q = (target.qx, target.qy, target.qz, target.qw)
                        target.qx = target.qy = target.qz = target.qw = 0.
                        count = len(received['target/pose'])
                        pump(.3, frame.SerializeToString())
                        assert len(received['target/pose']) > count
                        target.qx, target.qy, target.qz, target.qw = saved_q
                        # Non-finite box state is isolated to box topics.
                        body.X = float('nan')
                        pump(.2, frame.SerializeToString())
                        count = len(received['box/pose'])
                        robot_count = len(received['robot/pose'])
                        pump(.3, frame.SerializeToString())
                        assert len(received['box/pose']) == count
                        assert len(received['robot/pose']) > robot_count
                        body.X = 1000.
                        if name == 'ros2 run':
                            # Terminal Ctrl+C targets the foreground process group;
                            # ros2 run itself does not forward a parent-only SIGINT.
                            os.killpg(proc.pid, signal.SIGINT)
                        else:
                            proc.send_signal(signal.SIGINT)
                        assert proc.wait(timeout=8) == 0, logfile.read_text()
                        assert 'Traceback' not in logfile.read_text(), logfile.read_text()
                        result = {'entrypoint': name, 'status': 'passed', 'received_hz': rate, 'clean_shutdown': True, 'bodies': ['box', 'robot', 'target'], 'position_only_target': True, 'independent_tracking_loss': True}
                        results.append(result)
                        print(json.dumps(result), flush=True)
                    except BaseException:
                        print(logfile.read_text(), flush=True)
                        raise
                    finally:
                        if proc.poll() is None:
                            os.killpg(proc.pid, signal.SIGTERM)
                            try:
                                proc.wait(timeout=3)
                            except subprocess.TimeoutExpired:
                                os.killpg(proc.pid, signal.SIGKILL)
                                proc.wait(timeout=3)
    finally:
        observer.destroy_node()
        rclpy.try_shutdown()
        pub.close(0)
        context.term()
    print(json.dumps({'jazzy_checks': results, 'hardware_tested': False}, indent=2))


if __name__ == '__main__':
    main()
