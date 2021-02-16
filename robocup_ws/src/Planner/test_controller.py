import rospy
from robocup_control.srv import *
def request_actions():
    print("Starting client...")
    rospy.wait_for_service("actions_return")
    actions_req = rospy.ServiceProxy("actions_return",ActionServices)
    actions = actions_req(0,0,0,1)
    return actions.left_wheel, actions.right_wheel
if __name__ == "__main__":
    print(request_actions())