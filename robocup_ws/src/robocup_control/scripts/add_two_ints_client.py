from __future__ import print_function

#Import necessary libraries
import sys
import rospy
from robocup_control.srv import *
#Request service add_two_ints
def add_two_ints_client(x,y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints',AddTwoInts)
        resp1 = add_two_ints(x,y)
        return resp1.Sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__=="__main__":
    if(len(sys.argv) == 3):
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        print(x)
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))