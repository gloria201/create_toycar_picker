#!/usr/bin/env python  
  
import roslib; roslib.load_manifest('create_navigation')  
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  
  
class NavTest():  
    def __init__(self):  
        rospy.init_node('nav_test', anonymous=True)          
        rospy.on_shutdown(self.shutdown)  
          
        # time stay at each location (sec)  
        self.rest_time = rospy.get_param("~rest_time", 10)  
          
        # whether to run in the fake simulator  
        self.fake_test = rospy.get_param("~fake_test", False)  
          
        # reture goal state values 
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  

        # Publish topic to control the robot
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)  
          
        # Subscribe move_base action server  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        # Wait 60 seconds for the action server            
        rospy.loginfo("Waiting for move_base action server...")            
        self.move_base.wait_for_server(rospy.Duration(60))           
        rospy.loginfo("Connected to move base server")  
          
        # set the initial pose in RViz   
        initial_pose = PoseWithCovarianceStamped()  
          
        # Get the initial pose from Rviz 
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")  
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
        self.last_location = Pose()  
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)  
          
        # Check the existance of initial pose  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1) 

        # set the goal in RViz   
        goal = PoseWStamped()   

        # Get the goal from Rviz   
        rospy.loginfo("*** Click the 2D Goal button in RViz to set the goal...")  
        goal = MoveBaseGoal()
        rospy.wait_for_message('move_base_simple/goal', PoseStamped)  
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.update_goal)  

        # Check the existance of goal  
        while goal.header.stamp == "":  
            rospy.sleep(1) 
        
        # Save variables to keep track of success rate, running time and moving distance of the robot   
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  

        # Begin the main loop and run through a sequence of locations               
        rospy.loginfo("Starting navigation test")  
        while not rospy.is_shutdown():  
            # If all the goals reached, resequence
            if i == n_locations:  
                i = 0  
                sequence = sample(locations, n_locations)  
                # Skip the first location in new sequence if it is the same as the former last location   
                if sequence[0] == last_location:  
                    i = 1  
              
            # Get the next location in the current sequence    
            location = sequence[i]  
                          
            # Keep track of the moving distance
            # Use updated initial pose if available 
            if initial_pose.header.stamp == "":  
                distance = sqrt(pow(locations[location].position.x -   
                                    locations[last_location].position.x, 2) +  
                                pow(locations[location].position.y -   
                                    locations[last_location].position.y, 2))  
            else:  
                rospy.loginfo("Updating current pose.")  
                distance = sqrt(pow(locations[location].position.x -   
                                    initial_pose.pose.pose.position.x, 2) +  
                                pow(locations[location].position.y -   
                                    initial_pose.pose.pose.position.y, 2))  
                initial_pose.header.stamp = ""  
              
            # Store the last location for distance calculations   
            last_location = location  
              
            # Increment the counters  
            i += 1  
            n_goals += 1  
          
            # Set up the next goal location   
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = locations[location]  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  
              
            # Let the user know where the robot is going next  
            rospy.loginfo("Going to: " + str(location))  
              
            # Start the robot toward the next location   
            self.move_base.send_goal(self.goal)  
              
            # 5 minutes to get the goal  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   
              
            # Check for success or failure  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    n_successes += 1  
                    distance_traveled += distance  
                    rospy.loginfo("State:" + str(state))  
                else:  
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
              
            # count running time   
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0  
              
            # Print a summary of this navigation  
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +   
                          str(n_goals) + " = " +   
                          str(100 * n_successes/n_goals) + "%")  
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +   
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  
            rospy.sleep(self.rest_time)  
              
    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  

    def update_goal_pose(self, goal):  
        self.goal = goal 
  
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  
        
def trunc(f, n):  
    # Truncates/pads a float f to n decimal places without rounding  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  
  
if __name__ == '__main__':  
    try:  
        NavTest()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        pass
