#Server script
from __future__ import print_function
#Import service files (request, response)
from robocup_control.srv import AddTwoInts,AddTwoIntsResponse
#Import python ros module
import rospy

#Implement request server side
def handle_add_two_ints(req):
    print(req)
    print("Returning [%s + %s = %s]"%(req.A, req.B, (req.A + req.B)))
    return AddTwoIntsResponse(req.A + req.B)

#Initiate server node to accept service requests
def add_two_ints_server():
    #Initialize python node
    rospy.init_node('add_two_ints_server')
    #Initialize service add_two_ints for server
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    #Make sure code does not exit until service shuts down
    rospy.spin()

if __name__=="__main__":
    add_two_ints_server()