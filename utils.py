"""
utils.py – pure math utilities for Castor 2026 (Python/MagicBot port).

Direct translation of Utils.java. No framework dependencies.
"""

import math

from wpilib import DriverStation
from wpimath.geometry import Pose3d

from constants import FieldPoses, ShootingRegion, ShootingRegionDimensions


def calculate_turret_angle_from_camera_tag_distance(distance_to_tag: float) -> float:
    """
    Calculates the turret rotation (in rotations, robot-relative) needed
    to aim at the HUB given the camera's measured distance to the tag.

    :param distance_to_tag: camera distance to the AprilTag (metres)
    :return: turret angle in rotations (robot-relative)
    """
    camera_distance_to_target = distance_to_tag + 0.6
    camera_to_shoot_distance = 0.26035

    # Pythagorean theorem
    shooter_distance_to_target = math.sqrt(
        camera_distance_to_target**2 + camera_to_shoot_distance**2
    )
    angle_at_shooter = math.acos(camera_to_shoot_distance / shooter_distance_to_target)
    robot_turret_angle = (math.pi / 2) - angle_at_shooter
    robot_turret_rotations = robot_turret_angle / math.pi

    return robot_turret_rotations


def find_robot_shooting_region(
    robot_pose: Pose3d,
    alliance: DriverStation.Alliance,
) -> ShootingRegion:
    """
    Determines the shooting region for the robot based on its pose and alliance.

    :param robot_pose: current robot pose
    :param alliance: robot's alliance (Blue or Red)
    :return: the matching ShootingRegion enum value
    """
    region = ShootingRegion.NON_SHOOTING_REGION
    robot_x = robot_pose.X()
    robot_y = robot_pose.Y()

    # Local aliases to keep condition lines under 120 chars
    D = ShootingRegionDimensions
    in_neutral_x = D.NEUTRAL_ZONE_REGION_X_MIN < robot_x < D.NEUTRAL_ZONE_REGION_X_MAX

    if alliance == DriverStation.Alliance.kBlue:
        if robot_x < ShootingRegionDimensions.BLUE_ALLIANCE_ZONE_REGION_X_MAX:
            region = ShootingRegion.BLUE_OWN_ALLIANCE_ZONE
        elif (
            in_neutral_x
            and robot_y > ShootingRegionDimensions.BLUE_LEFT_REGION_Y_MIN
        ):
            region = ShootingRegion.BLUE_LEFT_NEUTRAL_ZONE
        elif (
            in_neutral_x
            and robot_y < ShootingRegionDimensions.BLUE_RIGHT_REGION_Y_MAX
        ):
            region = ShootingRegion.BLUE_RIGHT_NEUTRAL_ZONE
        elif (
            robot_x > ShootingRegionDimensions.RED_ALLIANCE_ZONE_REGION_X_MIN
            and robot_y > ShootingRegionDimensions.BLUE_LEFT_REGION_Y_MIN
        ):
            region = ShootingRegion.BLUE_LEFT_OPPONENT_ALLIANCE_ZONE
        elif (
            robot_x > ShootingRegionDimensions.RED_ALLIANCE_ZONE_REGION_X_MIN
            and robot_y < ShootingRegionDimensions.BLUE_RIGHT_REGION_Y_MAX
        ):
            region = ShootingRegion.BLUE_RIGHT_OPPONENT_ALLIANCE_ZONE

    elif alliance == DriverStation.Alliance.kRed:
        if robot_x > ShootingRegionDimensions.RED_ALLIANCE_ZONE_REGION_X_MIN:
            region = ShootingRegion.RED_OWN_ALLIANCE_ZONE
        elif (
            in_neutral_x
            and robot_y < ShootingRegionDimensions.RED_LEFT_REGION_Y_MAX
        ):
            region = ShootingRegion.RED_LEFT_NEUTRAL_ZONE
        elif (
            in_neutral_x
            and robot_y > ShootingRegionDimensions.RED_RIGHT_REGION_Y_MIN
        ):
            region = ShootingRegion.RED_RIGHT_NEUTRAL_ZONE
        elif (
            robot_x < ShootingRegionDimensions.BLUE_ALLIANCE_ZONE_REGION_X_MAX
            and robot_y < ShootingRegionDimensions.RED_LEFT_REGION_Y_MAX
        ):
            region = ShootingRegion.RED_LEFT_OPPONENT_ALLIANCE_ZONE
        elif (
            robot_x > ShootingRegionDimensions.RED_ALLIANCE_ZONE_REGION_X_MIN
            and robot_y > ShootingRegionDimensions.RED_RIGHT_REGION_Y_MIN
        ):
            region = ShootingRegion.RED_RIGHT_OPPONENT_ALLIANCE_ZONE

    return region


def get_hood_angle_for_distance(distance_meters: float) -> float:
    """
    Returns the hood angle for a given distance.

    :param distance_meters: distance to target in metres
    :return: hood servo angle
    """
    if distance_meters >= 4:
        return 200.0
    return 0.0


def get_launcher_rpm_for_distance(distance_meters: float) -> float:
    """
    Returns the launcher RPM for a given distance, capped at 5000 RPM.

    :param distance_meters: distance to target in metres
    :return: launcher RPM
    """
    return min((667.557 * distance_meters) + 1500.699, 5000.0)


def find_shooting_target(
    region: ShootingRegion,
    alliance: DriverStation.Alliance,
) -> Pose3d:
    """
    Determines the appropriate shooting target for the robot's current region
    and alliance.

    :param region: the robot's current ShootingRegion
    :param alliance: the robot's alliance (Blue or Red)
    :return: target Pose3d, or all-zero Pose3d if region/alliance is invalid
    """
    target = Pose3d()

    if alliance == DriverStation.Alliance.kBlue:
        if region == ShootingRegion.BLUE_OWN_ALLIANCE_ZONE:
            target = FieldPoses.BLUE_HUB_POSE
        elif region == ShootingRegion.BLUE_LEFT_NEUTRAL_ZONE:
            target = FieldPoses.BLUE_LEFT_ALLIANCE_ZONE_PASS_TARGET_POSE
        elif region == ShootingRegion.BLUE_RIGHT_NEUTRAL_ZONE:
            target = FieldPoses.BLUE_RIGHT_ALLIANCE_ZONE_PASS_TARGET_POSE
        elif region == ShootingRegion.BLUE_LEFT_OPPONENT_ALLIANCE_ZONE:
            target = FieldPoses.BLUE_LEFT_NEUTRAL_ZONE_PASS_TARGET_POSE
        elif region == ShootingRegion.BLUE_RIGHT_OPPONENT_ALLIANCE_ZONE:
            target = FieldPoses.BLUE_RIGHT_NEUTRAL_ZONE_PASS_TARGET_POSE

    elif alliance == DriverStation.Alliance.kRed:
        if region == ShootingRegion.RED_OWN_ALLIANCE_ZONE:
            target = FieldPoses.RED_HUB_POSE
        elif region == ShootingRegion.RED_LEFT_NEUTRAL_ZONE:
            target = FieldPoses.RED_LEFT_ALLIANCE_ZONE_PASS_TARGET_POSE
        elif region == ShootingRegion.RED_RIGHT_NEUTRAL_ZONE:
            target = FieldPoses.RED_RIGHT_ALLIANCE_ZONE_PASS_TARGET_POSE
        elif region == ShootingRegion.RED_LEFT_OPPONENT_ALLIANCE_ZONE:
            target = FieldPoses.RED_LEFT_NEUTRAL_ZONE_PASS_TARGET_POSE
        elif region == ShootingRegion.RED_RIGHT_OPPONENT_ALLIANCE_ZONE:
            target = FieldPoses.RED_RIGHT_NEUTRAL_ZONE_PASS_TARGET_POSE

    return target
