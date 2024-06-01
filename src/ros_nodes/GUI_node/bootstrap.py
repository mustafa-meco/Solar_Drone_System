# autopep8: off
import time
import rospy
import sys
import os

sys.path.append(os.getcwd())

# change this
import ros_nodes.GUI_node.general_main as main
# autopep8: on




def bootstrap():
    main.main()
    


if __name__ == '__main__':
    bootstrap()
    