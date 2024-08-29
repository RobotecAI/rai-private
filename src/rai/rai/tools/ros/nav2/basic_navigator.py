import time

from geometry_msgs.msg import Point, PoseStamped
from langchain.tools import tool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rai.tools.ros.nav2.navigator import RaiNavigator


def loop_and_report(navigator: BasicNavigator, timeout: int = 30) -> str:
    ts = time.perf_counter()
    while not navigator.isTaskComplete():
        if time.perf_counter() - ts > timeout:
            navigator.cancelTask()
            # navigator.lifecycleShutdown()
            return "Timed out!"

    # Do something depending on the return code
    result = navigator.getResult()
    # navigator.lifecycleShutdown()
    if result == TaskResult.SUCCEEDED:
        return "Goal succeeded!"
    elif result == TaskResult.CANCELED:
        return "Goal was canceled!"
    elif result == TaskResult.FAILED:
        return "Goal failed!"
    else:
        return "Goal has an invalid return status!"


@tool
def go_to_pose(x: float, y: float) -> str:
    """Use this tool to go to a pose."""
    navigator = BasicNavigator()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.orientation.z = 0.0
    navigator.goToPose(goal_pose)
    return loop_and_report(navigator)


@tool
def spin(degrees_rad: float) -> str:
    """Use this tool to spin the robot."""
    navigator = RaiNavigator()
    navigator.spin(spin_dist=degrees_rad)
    return loop_and_report(navigator)


@tool
def backup(distance_m: float, speed_mps: float) -> str:
    """Use this tool to back up the robot."""
    navigator = BasicNavigator()
    navigator.backup(backup_dist=distance_m, backup_speed=speed_mps)
    return loop_and_report(navigator)


@tool
def drive_on_heading(distance_m: float, speed_mps, time_allowance) -> str:
    """Use this tool to drive on a heading."""
    navigator = RaiNavigator()
    p = Point()
    p.x = distance_m

    navigator.drive_on_heading(p, speed_mps, time_allowance)
    return loop_and_report(navigator)
