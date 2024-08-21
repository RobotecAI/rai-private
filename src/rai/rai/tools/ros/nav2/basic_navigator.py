from geometry_msgs.msg import Point
from langchain.tools import tool

from rai.tools.ros.nav2.navigator import RaiNavigator


@tool
def spin_robot(degrees_rad: float) -> str:
    """Use this tool to spin the robot."""
    navigator = RaiNavigator()
    navigator.spin(spin_dist=degrees_rad)
    return "Robot spinning."


@tool
def drive_forward(distance_m: float) -> str:
    """Use this tool to drive the robot forward."""
    navigator = RaiNavigator()
    p = Point()
    p.x = distance_m

    navigator.drive_on_heading(p, 0.5, 10)
    return "Robot driving forward."
