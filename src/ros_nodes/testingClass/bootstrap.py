# autopep8: off
import time
import rospy
import sys
import os

sys.path.append(os.getcwd())

# change this
import main as Drone_node
# autopep8: on


# change this
_NODE_DELAY = 0.01  # 50ms delay / operation frequency 20Hz


if __name__ == '__main__':
    # change this
    Drone_node.ros_node_setup()

    while True:
        if rospy.is_shutdown():
            break

        try:
            # change this
            Drone_node.ros_node_loop()

        except rospy.ROSInterruptException:
            break

        time.sleep(_NODE_DELAY)