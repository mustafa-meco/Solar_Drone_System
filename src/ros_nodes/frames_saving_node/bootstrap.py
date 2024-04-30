# autopep8: off
import time
import rospy
import sys
import os

sys.path.append(os.getcwd())

# change this
import ros_nodes.frames_saving_node.main as frames_saving_node
# autopep8: on


# change this
_NODE_DELAY = 0.01  # 10ms delay / operation frequency 100Hz


if __name__ == '__main__':
    # change this
    frames_saving_node.ros_node_setup()

    while True:
        if rospy.is_shutdown():
            break

        try:
            # change this
            frames_saving_node.ros_node_loop()

        except rospy.ROSInterruptException:
            break

        time.sleep(_NODE_DELAY)