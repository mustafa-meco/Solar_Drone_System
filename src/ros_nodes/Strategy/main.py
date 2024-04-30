
# import mission_test from lib directory

from lib import mission_test
import lib.ros as ros_man
import lib.settings as set_man
from lib.mission_test import MavrosMissionTest
import rostest
import rospy
import os
import sys



# module config
_NODE_NAME = "mavros_mission_0202.test"
_PKG = 'px4'

# module state
_settings_obj: dict = None


def ros_node_setup():
    print("MAVROS Launching")
    # os.system("roslaunch mavros px4.launch fcu_url:=\"udp://:14540@192.168.1.36:14557\"")
    print("MAVROS Launched")
    print("-" for i in range(50))
    rospy.init_node('test_node', anonymous=True)

    name = "mavros_mission_tests-MC_mission_box.plan"

    name = "mavros_mission_test"
    if len(sys.argv) > 1:
        name += "-%s" % sys.argv[1]
    
    rostest.rosrun(_PKG, name, MavrosMissionTest)
    
    
def ros_node_loop():
    pass