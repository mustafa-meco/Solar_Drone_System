
# import mission_test from lib directory

import lib.ros as ros_man
import lib.settings as set_man
from lib.mission_test import MavrosMissionTest
from lib.myDrone import MyDrone
import rostest
import rospy
import os
import sys
from geometry_msgs.msg import PoseStamped



# module config
_NODE_NAME = "Drone_Node_0202.test"
_PKG = 'px4'

# module state
_settings_obj: dict = None
drone = None


def ros_node_setup():
    global drone

    print("MAVROS Launching")
    # os.system("roslaunch mavros px4.launch fcu_url:=\"udp://:14540@192.168.1.36:14557\"")
    print("MAVROS Launched")
    print("-" for i in range(50))
    
    rospy.init_node("offb_node_with_class", anonymous=True)
    drone = MyDrone()
    
    #name = "mavros_mission_tests-MC_mission_box.plan"

    #name = "Drone_Node"
    #if len(sys.argv) > 1:
    #    name += "-%s" % sys.argv[1]
    
    #rostest.rosrun(_PKG, name, MavrosMissionTest)
    
    
def ros_node_loop():
    global drone

    try:
        # Arm the drone
        #drone.arming(True)

        # Take off to an altitude of 5 meters
        drone.takeoff(5.0)
        rospy.sleep(10.0)
        drone.set_mode('OFFBOARD')
        # Waypoints to test
        waypoint_1 = PoseStamped()
        waypoint_1.pose.position.x = -10.0
        waypoint_1.pose.position.y = 0.0
        waypoint_1.pose.position.z = 5.0

        waypoint_2 = PoseStamped()
        waypoint_2.pose.position.x = -10.0
        waypoint_2.pose.position.y = 20.0
        waypoint_2.pose.position.z = 5.0

        waypoints = [waypoint_1, waypoint_2]

        # Handle waypoints with default threshold and yaw
        # log "sending waypoint...."
        rospy.loginfo("sending waypoint....")
        drone.handle_waypoints(waypoints,1)

        # Perform landing
        drone.landing()

    except rospy.ROSInterruptException:
        pass