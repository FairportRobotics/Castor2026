"""
LimelightHelpers.py – stub for the community limelightlib-python library.

Replace this file with the real LimelightHelpers.py from:
  https://github.com/LimelightVision/limelightlib-python

The following methods must be verified to exist in the real library before
any Limelight-dependent code is run on the robot (see PLAN.txt section 3):
  - getBotPoseEstimate_wpiBlue_MegaTag2()
  - SetIMUMode()
  - getTargetPose_CameraSpace()
  - SetRobotOrientation()
  - SetFiducialIDFiltersOverride()
  - setPipelineIndex()
  - getTV()
"""

from __future__ import annotations

from dataclasses import dataclass, field

from wpimath.geometry import Pose2d


@dataclass
class PoseEstimate:
    pose: Pose2d = field(default_factory=Pose2d)
    timestampSeconds: float = 0.0
    tagCount: int = 0
    isMegaTag2: bool = False


# ── Pose estimation ────────────────────────────────────────────────────────────

def getBotPoseEstimate_wpiBlue_MegaTag2(limelightName: str) -> PoseEstimate | None:
    """Return None (stub — no camera connected in sim)."""
    return None


def getBotPoseEstimate_wpiBlue(limelightName: str) -> PoseEstimate | None:
    return None


# ── Robot orientation / IMU ────────────────────────────────────────────────────

def SetRobotOrientation(
    limelightName: str,
    yaw: float,
    yawRate: float,
    pitch: float,
    pitchRate: float,
    roll: float,
    rollRate: float,
) -> None:
    pass


def SetIMUMode(limelightName: str, mode: int) -> None:
    pass


# ── Target information ─────────────────────────────────────────────────────────

def getTV(limelightName: str) -> bool:
    """Return False — no targets visible in stub."""
    return False


def getTargetPose_CameraSpace(limelightName: str) -> list[float]:
    """Return empty list — no target in stub."""
    return []


# ── Pipeline / filter control ──────────────────────────────────────────────────

def setPipelineIndex(limelightName: str, pipelineIndex: int) -> None:
    pass


def SetFiducialIDFiltersOverride(limelightName: str, validIDs: list[int]) -> None:
    pass
