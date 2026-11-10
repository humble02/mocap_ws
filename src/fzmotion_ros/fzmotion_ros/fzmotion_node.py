"""FZMotion -> ROS 2 bridge.

Receives rigid body pose + velocity/acceleration from FZMotion (LuMoSDK ZMQ +
protobuf) and publishes all tracked box/robot/target bodies on their own prefix:
  - geometry_msgs/PoseStamped     on <prefix>/pose     (position + orientation)
  - geometry_msgs/TwistStamped    on <prefix>/velocity      (linear + angular velocity)
  - geometry_msgs/AccelStamped    on <prefix>/acceleration    (linear + angular acceleration)
  - optional tf2 transform world -> <rigid name>

FZMotion is a Y-up, right-handed frame; ROS is Z-up (REP-103). With
apply_axis_map=true (default) everything is mapped through
R_map = Rz(yaw_offset) * Rx(+90deg) (see axis_map.py), for both vectors
(position/vel/accel/angular) and the orientation quaternion, so pose and
velocity share one frame.

Note: run with the environment that provides rclpy (sourced ROS 2) plus
pyzmq + protobuf==3.20.3. The provided run.sh takes care of that.
"""

import math

import zmq

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from google.protobuf.message import DecodeError
from geometry_msgs.msg import (
    PoseStamped, TransformStamped, TwistStamped, AccelStamped)
from tf2_ros import TransformBroadcaster

from . import LusterFrameStruct_pb2 as pb
from .axis_map import fz_vec_to_ros, fz_quat_to_ros, quat_rotate


def _sanitize_frame(name):
    """Make a valid-ish ROS frame id out of a rigid body name."""
    s = ''.join(c if (c.isalnum() or c in '_-/.') else '_' for c in name.strip())
    if not s:
        s = 'rigid'
    if s[0].isdigit():
        s = 'rb_' + s
    return s


class FZMotionNode(Node):
    def __init__(self):
        super().__init__('fzmotion_node')

        self.declare_parameter('ip', '169.254.209.149')
        self.declare_parameter('port', 6868)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('pos_scale', 0.001)    # position mm -> m
        self.declare_parameter('vel_scale', 1.0)      # linear velocity already m/s
        self.declare_parameter('acc_scale', 1.0)      # linear accel already m/s^2
        self.declare_parameter('ang_scale', 1.0)      # angular already rad/s, rad/s^2
        for role in ('box', 'robot', 'target'):
            self.declare_parameter(f'{role}.rigid_name', role)
            self.declare_parameter(f'{role}.topic_prefix', f'/fzmotion/{role}')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('apply_axis_map', True)   # FZ Y-up -> ROS Z-up
        self.declare_parameter('yaw_offset_deg', 0.0)    # theta: 0 / +90 / -90 ...
        self.declare_parameter('poll_rate', 200.0)

        self.ip = self.get_parameter('ip').value
        self.port = int(self.get_parameter('port').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.pos_scale = float(self.get_parameter('pos_scale').value)
        self.vel_scale = float(self.get_parameter('vel_scale').value)
        self.acc_scale = float(self.get_parameter('acc_scale').value)
        self.ang_scale = float(self.get_parameter('ang_scale').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.apply_axis_map = bool(self.get_parameter('apply_axis_map').value)
        self.theta = math.radians(float(self.get_parameter('yaw_offset_deg').value))
        poll_rate = float(self.get_parameter('poll_rate').value)

        self.bodies = {}
        self.roles = {}
        for role in ('box', 'robot', 'target'):
            name = self.get_parameter(f'{role}.rigid_name').value
            prefix = self.get_parameter(f'{role}.topic_prefix').value.rstrip('/')
            if not name or name in self.bodies:
                raise ValueError('box/robot/target require distinct nonempty rigid names')
            self.roles[name] = role
            self.bodies[name] = (
                self.create_publisher(PoseStamped, prefix + '/pose', 10),
                self.create_publisher(TwistStamped, prefix + '/velocity', 10),
                self.create_publisher(AccelStamped, prefix + '/acceleration', 10))
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.ctx = zmq.Context()
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.CONFLATE, 1)
        self.sub.setsockopt(zmq.RCVTIMEO, 0)
        self.sub.connect('tcp://%s:%d' % (self.ip, self.port))
        self.sub.setsockopt_string(zmq.SUBSCRIBE, '')

        self._last_rx = self.get_clock().now().nanoseconds / 1e9
        self._last_warn = 0.0
        self.get_logger().info(f'FZMotion tcp://{self.ip}:{self.port}, bodies={list(self.bodies)}, frame={self.frame_id}')

        self.create_timer(1.0 / max(poll_rate, 1.0), self.tick)

    def tick(self):
        try:
            msg = self.sub.recv(zmq.DONTWAIT)
        except zmq.Again:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_rx > 3.0 and now - self._last_warn > 3.0:
                self.get_logger().warn('No FZMotion data on %s:%d for %.0fs ...'
                                       % (self.ip, self.port, now - self._last_rx))
                self._last_warn = now
            return

        frame = pb.Frame()
        try:
            frame.ParseFromString(msg)
        except DecodeError:
            self.get_logger().warning('Ignoring malformed FZMotion protobuf frame', throttle_duration_sec=3.0)
            return
        self._last_rx = self.get_clock().now().nanoseconds / 1e9
        stamp = self.get_clock().now().to_msg()
        seen = set()
        for rigid in frame.rigidBodys:
            if rigid.IsTrack and rigid.Name in self.bodies and rigid.Name not in seen:
                seen.add(rigid.Name)
                self.publish_rigid(rigid, stamp)

    def publish_rigid(self, r, stamp):
        # The placement target needs only a valid XYZ. A missing orientation or
        # velocity must not suppress its position (the gateway never uses them).
        target_pose = None
        if self.roles[r.Name] == 'target' and all(math.isfinite(v) for v in (r.X, r.Y, r.Z)):
            target_pose = PoseStamped()
            target_pose.header.stamp = stamp
            target_pose.header.frame_id = self.frame_id
            xyz = fz_vec_to_ros((r.X*self.pos_scale, r.Y*self.pos_scale, r.Z*self.pos_scale), self.theta, self.apply_axis_map)
            target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z = xyz
        def publish_position_only():
            if target_pose is not None:
                # Zero quaternion denotes unavailable orientation; position stays usable.
                self.bodies[r.Name][0].publish(target_pose)
        values = (r.X, r.Y, r.Z, r.qw, r.qx, r.qy, r.qz,
                  r.speeds.XfSpeed, r.speeds.YfSpeed, r.speeds.ZfSpeed,
                  r.palstance.fXPalstance, r.palstance.fYPalstance, r.palstance.fZPalstance,
                  r.acceleratedSpeeds.XfAcceleratedSpeed, r.acceleratedSpeeds.YfAcceleratedSpeed,
                  r.acceleratedSpeeds.ZfAcceleratedSpeed, r.accpalstance.AccfXPalstance,
                  r.accpalstance.AccfYPalstance, r.accpalstance.AccfZPalstance)
        if not all(math.isfinite(v) for v in values):
            publish_position_only()
            return
        qnorm = math.sqrt(sum(x*x for x in (r.qx, r.qy, r.qz, r.qw)))
        if not 0.5 < qnorm < 1.5:
            publish_position_only()
            return
        q_fz = tuple(v/qnorm for v in (r.qx, r.qy, r.qz, r.qw))
        pub_pose, pub_vel, pub_accel = self.bodies[r.Name]
        th = self.theta
        am = self.apply_axis_map

        px, py, pz = fz_vec_to_ros(
            (r.X * self.pos_scale, r.Y * self.pos_scale, r.Z * self.pos_scale), th, am)
        qx, qy, qz, qw = fz_quat_to_ros(q_fz, th, am)

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.position.z = pz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        pub_pose.publish(ps)

        sp = r.speeds
        vx, vy, vz = fz_vec_to_ros(
            (sp.XfSpeed * self.vel_scale, sp.YfSpeed * self.vel_scale,
             sp.ZfSpeed * self.vel_scale), th, am)
        # angular velocity/acceleration are in the rigid body's OWN frame -> rotate
        # to the (FZ) world frame with the body orientation before axis-mapping.
        pl = r.palstance
        w_body = (pl.fXPalstance * self.ang_scale,
                  pl.fYPalstance * self.ang_scale,
                  pl.fZPalstance * self.ang_scale)
        wx, wy, wz = fz_vec_to_ros(quat_rotate(q_fz, w_body), th, am)
        tw = TwistStamped()
        tw.header.stamp = stamp
        tw.header.frame_id = self.frame_id
        tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = vx, vy, vz
        tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = wx, wy, wz
        pub_vel.publish(tw)

        ac = r.acceleratedSpeeds
        ax, ay, az = fz_vec_to_ros(
            (ac.XfAcceleratedSpeed * self.acc_scale,
             ac.YfAcceleratedSpeed * self.acc_scale,
             ac.ZfAcceleratedSpeed * self.acc_scale), th, am)
        aa = r.accpalstance
        a_body = (aa.AccfXPalstance * self.ang_scale,
                  aa.AccfYPalstance * self.ang_scale,
                  aa.AccfZPalstance * self.ang_scale)
        alx, aly, alz = fz_vec_to_ros(quat_rotate(q_fz, a_body), th, am)
        a = AccelStamped()
        a.header.stamp = stamp
        a.header.frame_id = self.frame_id
        a.accel.linear.x, a.accel.linear.y, a.accel.linear.z = ax, ay, az
        a.accel.angular.x, a.accel.angular.y, a.accel.angular.z = alx, aly, alz
        pub_accel.publish(a)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.frame_id
            t.child_frame_id = _sanitize_frame(r.Name)
            t.transform.translation.x = px
            t.transform.translation.y = py
            t.transform.translation.z = pz
            t.transform.rotation = ps.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        try:
            self.sub.close(0)
            self.ctx.term()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FZMotionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
