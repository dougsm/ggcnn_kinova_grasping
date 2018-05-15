# Borrowed and modified from the kinova-ros examples.

import rospy
import actionlib
import kinova_msgs.msg

finger_maxTurn = 7400

def set_finger_positions(finger_positions):
    """Send a gripper goal to the action server."""
    global gripper_client
    global finger_maxTurn

    finger_positions[0] = min(finger_maxTurn, finger_positions[0])
    finger_positions[1] = min(finger_maxTurn, finger_positions[1])

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    gripper_client.send_goal(goal)
    if gripper_client.wait_for_result(rospy.Duration(5.0)):
        return gripper_client.get_result()
    else:
        gripper_client.cancel_all_goals()
        rospy.WARN('        the gripper action timed-out')
        return None

action_address = '/m1n6s200_driver/fingers_action/finger_positions'
gripper_client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction)
