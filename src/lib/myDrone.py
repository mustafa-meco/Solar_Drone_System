import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import NavSatFix
from pygeodesy.geoids import GeoidPGM
from mavros_msgs.srv import CommandTOL, CommandTOLRequest

class MyDrone:
    def __init__(self, initial_pose=None):
        self.current_state = State()
        
        self.cmd_landing_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        rospy.wait_for_service("/mavros/cmd/land", timeout=10.0)
        rospy.loginfo("mavros land service is available")
        self.egm = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)
        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.Sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pos_callback)
        self.global_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_pos_callback)
        self.cmd_takeoff_client = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

        self.rate = rospy.Rate(10)
        self.pose=PoseStamped()
        self.global_position = NavSatFix()
        
        if initial_pose:
            self.pose = initial_pose
        else:
            # Default initial pose (0, 0, 2)
            self.pose.pose.position.x = 0
            self.pose.pose.position.y = 0
            self.pose.pose.position.z = 0
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True
    def local_pos_callback(self, pose):
        # This callback function will be called whenever a new message is received on /mavros/local_position/pose
        # Update the local pose in the class
        self.pose = pose
        
    def global_pos_callback(self, global_fix):
        # Convert AMSL altitude to ellipsoid height
        #"""Calculates AMSL to ellipsoid conversion offset."""
        ellipsoid_height = global_fix.altitude - self.egm.height(global_fix.latitude, global_fix.longitude)

        # Update the global position in the class
        self.global_position = global_fix
        self.global_position.altitude = ellipsoid_height



    def state_cb(self, msg):
        self.current_state = msg

    def arming(self, value):
        self.set_mode('MANUAL')
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = value
        last_req = rospy.Time.now()
        while not rospy.is_shutdown():
            if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                    rospy.sleep(1.0)  # Add a small delay after arming
                    break
                last_req = rospy.Time.now()
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

   
    def takeoff(self, altitude):
        self.arming(True)
        self.set_mode("AUTO.TAKEOFF")


        self.rate.sleep()
        last_req_mode = rospy.Time.now()
        last_req = rospy.Time.now()
        while not rospy.is_shutdown():

            # Use the takeoff service instead of publishing to the local position topic
            takeoff_req = CommandTOLRequest()
            takeoff_req.altitude = altitude+self.global_position.altitude  # Set the desired takeoff altitude
            takeoff_req.min_pitch = 0.0  # Set the minimum pitch during takeoff
            takeoff_req.yaw=0
            takeoff_req.latitude=self.global_position.latitude
            takeoff_req.longitude=self.global_position.longitude

            #min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 10.0}" 
            
            # Send the takeoff command
            try:
                response = self.cmd_takeoff_client(takeoff_req)
                if response.success:
                    rospy.loginfo("Vehicle is taking off")
                    break
                else:
                    rospy.logerr("Failed to send takeoff command")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                
        while(abs(self.pose.pose.position.z-altitude)<0.1):
            self.rate.sleep()
        self.set_mode("OFFBOARD")   
        rospy.loginfo("Takeoff Successful")
        self.rate.sleep()
    
    
    def landing(self):
        # Implement the landing function here
        landing_req = CommandTOLRequest()
        landing_req.yaw = 0.0  # Set the desired yaw angle during landing

        # Send the land command
        try:
            response = self.cmd_landing_client(landing_req)
            if response.success:
                rospy.loginfo("Vehicle is landing")
            else:
                rospy.logerr("Failed to send land command")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_mode(self, mode):
        set_mode_req = SetModeRequest()
        set_mode_req.custom_mode = mode
        last_req = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_state.mode != mode and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(set_mode_req).mode_sent:
                    rospy.loginfo(f"Mode set to {mode}")
                    rospy.sleep(1.0)  # Add a small delay after setting mode
                    break
                last_req = rospy.Time.now()
            self.rate.sleep()
            
            
    def handle_waypoints(self, waypoints, threshold=0.1, default_yaw=0.0):
        for waypoint_number, waypoint in enumerate(waypoints, start=1):
        # Ensure that each waypoint is of type PoseStamped
            if not isinstance(waypoint, PoseStamped):
                rospy.logerr("Waypoint is not of type PoseStamped. Skipping...")
                continue
        # If yaw is specified in the waypoint, use it; otherwise, use the default_yaw
            yaw = waypoint.pose.orientation.z if waypoint.pose.orientation.z else self.pose.pose.orientation.z
        # Set the yaw in the current pose
            waypoint.pose.orientation.z = yaw

            while not rospy.is_shutdown():
                # Check if the current pose is close to the target waypoint
                if (
                    abs(self.pose.pose.position.x - waypoint.pose.position.x) < threshold
                    and abs(self.pose.pose.position.y - waypoint.pose.position.y) < threshold
                    and abs(self.pose.pose.position.z - waypoint.pose.position.z) < threshold
                ):
                    rospy.loginfo(f"Waypoint #{waypoint_number} reached at (x={waypoint.pose.position.x}, y={waypoint.pose.position.y}, z={waypoint.pose.position.z})")
                    break

                # Publish the target waypoint
                self.local_pos_pub.publish(waypoint)
                self.rate.sleep()