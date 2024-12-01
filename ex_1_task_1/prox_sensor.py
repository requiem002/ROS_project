#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from demo_programs.msg import prox_sensor  # Custom message for proximity sensors

class RobotController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('robot_controller', anonymous=True)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("Starting Node")
        
        # Subscribers
        rospy.Subscriber('/hokuyo', LaserScan, self.lidar_callback)
        rospy.Subscriber('/cop/prox_sensors', prox_sensor, self.proximity_callback)
        
        # Control rate (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Initialize sensor data storage
        self.lidar_data = None
        self.prox_data = None
        
        # Initialize Twist message
        self.vel_msg = Twist()

        self.rotation_counter = 0

    def lidar_callback(self, data):
        self.lidar_data = data
        
    def proximity_callback(self, data):
        self.prox_data = data

    def stop(self):
        """Stop the robot by setting all velocities to 0"""
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(self.vel_msg)
        rospy.loginfo("Robot stopped")

    def move_forward(self, speed=0.2):
        """Move the robot forward at the specified speed"""
        self.vel_msg.linear.x = speed
        self.vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel_msg)
        #rospy.loginfo(f"Moving forward at speed: {speed}")

    def rotate(self, angular_speed=0.5):
        """Rotate the robot at the specified angular speed"""
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.vel_msg)

        self.rotation_counter += 1 if angular_speed > 0 else -1
        #rospy.loginfo(f"Rotating at speed: {angular_speed}")

        

    def run(self):
        """Main control loop"""
        
        obstacle_count = 0
        DETECT_THRESHOLD = 0.2  # Distance at which an obstacle is considered "detected"
        CLEAR_THRESHOLD = 0.3   # Distance at which an obstacle is considered "cleared"
        
    
        while not rospy.is_shutdown():
            
            if self.prox_data:
            # Log all front sensor values for debugging
                # Check if any front sensors detect an obstacle
                front_sensors_active = (
                (0 < self.prox_data.prox_front <= DETECT_THRESHOLD) or 
                (0 < self.prox_data.prox_front_left <= DETECT_THRESHOLD) or 
                (0 < self.prox_data.prox_front_right <= DETECT_THRESHOLD) or    
                (0 < self.prox_data.prox_front_right_right <= DETECT_THRESHOLD) or
                (0 < self.prox_data.prox_front_left_left <= DETECT_THRESHOLD)
                )



                front_45_sensors_check = (
                (0 < self.prox_data.prox_front_right_right <= DETECT_THRESHOLD) or
                (0 < self.prox_data.prox_front_left_left <= DETECT_THRESHOLD)
                )
                rospy.loginfo(f"""
                    Proximity Readings 1:
                    Front: {self.prox_data.prox_front}
                    Front Left: {self.prox_data.prox_front_left}
                    Front Right: {self.prox_data.prox_front_right}
                """)
                
                
                
                turn_direction = 0
    
                if front_sensors_active:
                    
                    obstacle_count  += 1
                    
                    """
                    if obstacle_count > 5:
                        
                        rospy.loginfo("Obstacle persistently detected! Perhaps I am stuck!")
                        self.stop()
                        self.rotate(0.8)
                        obstacle_count = 0
                        rospy.sleep(1)
                        
                        """
                # If obstacle detected, rotate away from it
                    if self.prox_data.prox_front_right > self.prox_data.prox_front_left:
                        
                        turn_direction = -0.1
                        #self.rotation_counter -= 1
                        rospy.loginfo("Obstacle on right - rotating left")
                        
                    elif self.prox_data.prox_front_left > self.prox_data.prox_front_right:
                        
                        turn_direction = 0.1
                        #self.rotation_counter += 1
                        rospy.loginfo("Obstacle on left - rotating right")
                    else:

                        turn_direction = 0
                        rospy.loginfo("Not sure what to do!")
                    
                     #Continue rotating in the chosen direction until the obstacle is cleared
                    while front_sensors_active and not rospy.is_shutdown():
                        
                        if turn_direction == 0:
                            break
                        
                        self.rotate(angular_speed = turn_direction)
                        rospy.sleep(0.1)  # Short delay to avoid overwhelming the system
                        
                        # Update front sensor status in real-time
                        front_sensors_active = (
                        (self.prox_data.prox_front > 0 and self.prox_data.prox_front <= CLEAR_THRESHOLD) or
                        (self.prox_data.prox_front_left > 0 and self.prox_data.prox_front_left <= CLEAR_THRESHOLD) or
                        (self.prox_data.prox_front_right > 0 and self.prox_data.prox_front_right <= CLEAR_THRESHOLD) or
                        (0 < self.prox_data.prox_front_right_right <= CLEAR_THRESHOLD)or
                        (0 < self.prox_data.prox_front_left_left <= CLEAR_THRESHOLD) 
                    )
                        
                    #    rospy.loginfo(f"""
                    #    Proximity Readings 2:
                    #    Front: {self.prox_data.prox_front}
                    #    Front Left: {self.prox_data.prox_front_left}
                    #    Front Right: {self.prox_data.prox_front_right}
                    #""")
                        

                            
                        if not front_sensors_active:
                            
                            rospy.sleep(0.5)
                            rospy.loginfo("Obstacle cleared - resuming forward movement")
                            break
                    
                        
                else:

                    if self.rotation_counter != 0:
                        rospy.loginfo(f"Rotation counter: {self.rotation_counter}")
                        turn_direction = 0.1 if self.rotation_counter < 0 else -0.1

                        self.rotate(angular_speed = turn_direction)
                        #self.rotation_counter += 1 if turn_direction > 0 else -1

                    else:
                        # If no obstacles detected, move forward
                        obstacle_count = 0
                        self.move_forward()
                        rospy.loginfo("No obstacles detected - moving forward")
                        
                        rospy.sleep(0.5)
            
            self.rate.sleep()


if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
