"""Compatibility import for the standalone vendor SDK examples.

The unchanged vendor protocol now ships inside the ROS package. This fallback
also lets the helpers run from a source checkout without sourcing ROS.
"""
import sys
from pathlib import Path

try:
    from fzmotion_ros.LusterFrameStruct_pb2 import *  # noqa: F401,F403
except ModuleNotFoundError as exc:
    if exc.name not in ('fzmotion_ros', 'fzmotion_ros.LusterFrameStruct_pb2'):
        raise
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'fzmotion_ros'))
    from fzmotion_ros.LusterFrameStruct_pb2 import *  # noqa: F401,F403
