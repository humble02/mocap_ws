"""Mock FZMotion ZMQ publisher with KNOWN values, for verifying the node.

Units per SDK: position mm, linear velocity/accel in WORLD frame (m/s, m/s^2),
angular velocity/accel in BODY frame (rad/s, rad/s^2).

This mock uses a NON-identity body orientation to exercise the body->world
rotation of the angular quantities:
  q_fz = +90 deg about FZ-Y (up):  (0, sin45, 0, cos45) = (0, .7071, 0, .7071)

Input (constant frame, rigids 'box', 'robot', 'target'):
  pos(mm, world)          = (1000, 2000, 3000)
  q_fz                    = (0, 0.70710678, 0, 0.70710678)
  speed(m/s, world)       = (0.5, 1.0, 1.5)
  palstance(rad/s, BODY)  = (0.0, 0.0, 0.3)     -> world = R_y90 * (0,0,0.3) = (0.3,0,0)
  accel(m/s^2, world)     = (10, 20, 30)
  accpalstance(rad/s^2,BODY) = (0,0,2)          -> world = (2,0,0)

Expected published (pos_scale=.001, vel/acc/ang scale=1, theta=0 => (x,-z,y)):
  pose pos(m)        = (1.0, -3.0, 2.0)
  vel linear(m/s)    = (0.5, -1.5, 1.0)   [world, unchanged by body rot]
  vel angular(rad/s) = (0.3, 0.0, 0.0)    [body->world THEN map]  (buggy would be (0,-0.3,0))
  acc linear(m/s^2)  = (10.0, -30.0, 20.0)
  acc angular(rad/s^2) = (2.0, 0.0, 0.0)
"""
import sys
import time
import math
import zmq
import LusterFrameStruct_pb2 as pb

port = int(sys.argv[1]) if len(sys.argv) > 1 else 6899
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:%d" % port)

f = pb.Frame()
f.FrameId = 1
r = f.rigidBodys.add()
r.Id = 1
r.Name = "box"
r.IsTrack = True
r.X, r.Y, r.Z = 1000.0, 2000.0, 3000.0
s = math.sin(math.radians(45.0))
r.qx, r.qy, r.qz, r.qw = 0.0, s, 0.0, s
r.speeds.fSpeed = 0.0
r.speeds.XfSpeed, r.speeds.YfSpeed, r.speeds.ZfSpeed = 0.5, 1.0, 1.5
r.palstance.fXPalstance, r.palstance.fYPalstance, r.palstance.fZPalstance = 0.0, 0.0, 0.3
r.acceleratedSpeeds.fAcceleratedSpeed = 0.0
r.acceleratedSpeeds.XfAcceleratedSpeed = 10.0
r.acceleratedSpeeds.YfAcceleratedSpeed = 20.0
r.acceleratedSpeeds.ZfAcceleratedSpeed = 30.0
r.accpalstance.AccfXPalstance = 0.0
r.accpalstance.AccfYPalstance = 0.0
r.accpalstance.AccfZPalstance = 2.0

for index, name in enumerate(('robot', 'target'), start=2):
    other = f.rigidBodys.add()
    other.CopyFrom(r)
    other.Id = index
    other.Name = name
    other.X += 1000 * (index - 1)

data = f.SerializeToString()
print("mock PUB on :%d (non-identity body orientation)" % port)
while True:
    pub.send(data)
    time.sleep(0.02)
