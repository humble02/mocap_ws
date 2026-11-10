"""Diagnose angular-velocity frame during a spin.

Reads raw FZMotion frames for ~8 s and, for rigid 'main', prints:
  body(w)  = raw palstance from the SDK (assumed BODY frame, rad/s)
  world(w) = body rotated by the rigid orientation q  (body->world, via quat_rotate)

Run it, and ROTATE 'main' about your intended axis during the 8 s.
  - If world(w) comes out as a clean single axis  -> quaternion convention (body->world) is correct.
  - If body(w) is clean but world(w) is messy      -> quaternion convention is reversed (need conjugate).
  - If both messy                                  -> the physical rotation isn't a clean single axis.

Usage:
    cd ~/mocap_ws/py && ../env_jazzy/bin/python diag_spin.py [ip]
"""
import sys
import time
import zmq
import LusterFrameStruct_pb2 as pb


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


ip = sys.argv[1] if len(sys.argv) > 1 else "169.254.209.149"
ctx = zmq.Context()
s = ctx.socket(zmq.SUB)
s.setsockopt(zmq.CONFLATE, 1)
s.setsockopt(zmq.RCVTIMEO, 1000)
s.connect("tcp://%s:6868" % ip)
s.setsockopt_string(zmq.SUBSCRIBE, "")

print("capturing 8s - rotate 'main' about your intended axis NOW ...")
t0 = time.time()
n = 0
while time.time() - t0 < 8.0:
    try:
        m = s.recv()
    except zmq.Again:
        continue
    f = pb.Frame()
    f.ParseFromString(m)
    for r in f.rigidBodys:
        if r.IsTrack and r.Name == "main":
            pl = r.palstance
            body = (pl.fXPalstance, pl.fYPalstance, pl.fZPalstance)
            if abs(body[0]) + abs(body[1]) + abs(body[2]) > 0.05:   # only while moving
                world = quat_rotate((r.qx, r.qy, r.qz, r.qw), body)
                print("body= %7.3f %7.3f %7.3f  |  world= %7.3f %7.3f %7.3f"
                      % (body[0], body[1], body[2], world[0], world[1], world[2]))
                n += 1
print("done (%d moving samples)" % n)
s.close()
ctx.term()
