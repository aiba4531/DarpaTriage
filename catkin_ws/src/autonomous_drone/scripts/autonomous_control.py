#!/usr/bin/env python

# Import the ROS libraries
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped

# Import system and os so that launch script knows to add the scripts directory to the Python path
import sys
import os

# Add the scripts directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import TSP solver 
import tsp_solver

class AutonomousController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('autonomous_controller', anonymous=True)

        ## MAVROS topics and services ##
        
        # Shows the connection state of aircraft
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)
       
        # Publishes the target position of the aircraft       
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # Subscribes to the current position of the aircraft
        self.local_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        
        # Clients for arming, setting mode, takeoff, and landing
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.takeoff_client = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self.land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)

        # State variables
        self.current_state = None
        self.current_pose = None
        self.target_pose = PoseStamped()

    def state_callback(self, msg):
        """Callback for /mavros/state"""
        self.current_state = msg
        
    def pose_callback(self, msg):
        """Callback for /mavros/local_position/pose"""
        self.current_pose = msg
                
    def reached_waypoint(self, current_pose, target_pose, tolerance=0.5):
        """Checks if the drone has reached a waypoint."""
        return abs(current_pose.pose.position.x - target_pose.pose.position.x) < tolerance and \
               abs(current_pose.pose.position.y - target_pose.pose.position.y) < tolerance and \
               abs(current_pose.pose.position.z - target_pose.pose.position.z) < tolerance
                       
    def arm_and_takeoff(self, target_altitude=5.0):
        """Arms the drone and takes off to a specified altitude."""
        rate = rospy.Rate(20)  # 20 Hz
        
        # Wait for the /mavros/state callback to initialize self.current_state
        while not rospy.is_shutdown() and self.current_state is None:
            rospy.loginfo("Waiting for state information...")
            rate.sleep()


        # Wait for FCU connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            rate.sleep()


        # Send a few setpoints before starting
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.pose.position.x = 0
        self.target_pose.pose.position.y = 0
        self.target_pose.pose.position.z = target_altitude
        
        # Send a few setpoints before starting
        for _ in range(100):
            self.target_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.target_pose)
            rate.sleep()
        
        
        # Set the mode to GUIDED
        rospy.loginfo("Setting mode to GUIDED...")
        response = self.set_mode_client(custom_mode="GUIDED")
        if response.mode_sent:
            rospy.loginfo("GUIDED mode set successfully!")
        else:
            rospy.logerr("Failed to set GUIDED mode!")
            
            
        # Wait for Prearm check: NOTE This doesnt work for some reason            
        # rospy.loginfo("Waiting for prearm check...")
        # while not rospy.is_shutdown() and not self.current_state.system_status == "PREARMED":
        #     rospy.logwarn("Drone is not ready to arm.")
        #     rate.sleep()
        # rospy.loginfo("Prearm check passed!")
            
            
        # Arm the drone NOTE: WAIT FOR PREARM check and set home position
        rospy.loginfo("Arming the drone...")
        while not rospy.is_shutdown() and not self.current_state.armed:
            response = self.arming_client(value=True)
            if response.success:
                rospy.loginfo("Drone armed successfully!")
            else:
                rospy.logerr("Failed to arm the drone!")
            rate.sleep()
        
        
        # Takeoff to the target altitude   
        rospy.loginfo("Taking off to an altitude of {} meters...".format(target_altitude))
        while not rospy.is_shutdown():
            response = self.takeoff_client(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=target_altitude)
            if response.success:
                rospy.loginfo("Takeoff successful!")
                break
            else:
                rospy.logerr("Failed to takeoff!")
            rate.sleep()
            
            
        # Wait for the drone to reach the target altitude
        while self.current_pose is not None and not self.reached_waypoint(self.current_pose, self.target_pose):
            rate.sleep()
        rospy.loginfo("Drone reached target altitude!")
        
        # Generate random targets
        num_targets = 10
        targets = tsp_solver.generate_random_targets(num_targets)

        # Solve TSP
        tour, targets, cost = tsp_solver.solve_tsp_milp(targets)
        
        # Show TSP Plot
        tsp_solver.plot_tsp(targets, tour, show_plot=True)
        
        # Print the TSP solution
        if tour is not None:
            rospy.loginfo("TSP solution:")
            for i, j in tour:
                rospy.loginfo(f"Target {i} at {targets[i]}.2f -> Target {j} at {targets[j]}.2f")
        else:
            rospy.logerr("No solution found!")
            
            
        # Fly to each target in the TSP solution
        for i, j in tour:
            rospy.loginfo(f"Flying to Target {j}...")
            self.target_pose.header.stamp = rospy.Time.now()
            self.target_pose.pose.position.x = targets[j][0]
            self.target_pose.pose.position.y = targets[j][1]
            self.target_pose.pose.position.z = target_altitude
            while not rospy.is_shutdown() and not self.reached_waypoint(self.current_pose, self.target_pose):
                self.target_pose.header.stamp = rospy.Time.now()
                self.local_pos_pub.publish(self.target_pose)
                rate.sleep()
        
        
        # Land the drone
        rospy.loginfo("Landing the drone...")
        while not rospy.is_shutdown():
            response = self.land_client(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=0)
            if response.success:
                rospy.loginfo("Landing accepted!")
                break
            else:
                rospy.logerr("Failed to land!")
            rate.sleep()
        
            
        # Check if drone has landed
        while not rospy.is_shutdown() and self.current_state.armed:
            rate.sleep()    
        rospy.loginfo("Drone has landed!")
        

if __name__ == "__main__":
    try:
        controller = AutonomousController()
        controller.arm_and_takeoff()
    except rospy.ROSInterruptException:
        pass
