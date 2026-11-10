"""Quick viewer: dump raw FZMotion frames (mm, raw protobuf) over ZMQ.

Usage:
    python3 raw_dump.py [ip]        # default 169.254.209.149
Run from this py/ directory (needs LusterFrameStruct_pb2.py alongside).
"""
import sys
import zmq
import LusterFrameStruct_pb2 as pb

ip = sys.argv[1] if len(sys.argv) > 1 else "169.254.209.149"
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.setsockopt(zmq.CONFLATE, 1)
sub.setsockopt(zmq.RCVTIMEO, 5000)
sub.connect("tcp://%s:6868" % ip)
sub.setsockopt_string(zmq.SUBSCRIBE, "")

for _ in range(3):
    try:
        msg = sub.recv()
    except zmq.Again:
        print("timeout: no data")
        break
    f = pb.Frame()
    f.ParseFromString(msg)
    tracked = [r for r in f.rigidBodys if r.IsTrack]
    print("frame %d | rigid=%d tracked=%d" % (f.FrameId, len(f.rigidBodys), len(tracked)))
    for r in tracked:
        print("  [%s] X=%.2f Y=%.2f Z=%.2f q=(%.4f,%.4f,%.4f,%.4f) v=(%.2f,%.2f,%.2f)"
              % (r.Name, r.X, r.Y, r.Z, r.qx, r.qy, r.qz, r.qw,
                 r.speeds.XfSpeed, r.speeds.YfSpeed, r.speeds.ZfSpeed))

sub.close()
ctx.term()
