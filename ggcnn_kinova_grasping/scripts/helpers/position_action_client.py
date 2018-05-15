# Borrowed and modified from the kinova-ros examples.

import rospy
import actionlib
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg


def move_to_position(position, orientation):
    """Send a cartesian goal to the action server."""
    global position_client
    if position_client is None:
        init()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=('m1n6s200_link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    position_client.send_goal(goal)

    if position_client.wait_for_result(rospy.Duration(10.0)):
        return position_client.get_result()
    else:
        position_client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

action_address = '/m1n6s200_driver/pose_action/tool_pose'
position_client = None

def init():
    global position_client
    position_client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    position_client.wait_for_server()
