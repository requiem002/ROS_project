#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
#from sensor_msgs.msg import LaserScan
from demo_programs.msg import prox_sensor  # Custom message for proximity sensors
from demo_programs.msg import line_sensor
from sensor_msgs.msg import Image
import logging
from std_msgs.msg import String

class RobotController:
    def __init__(self):
        # Initialize ROS
        rospy.init_node('robot_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.log_pub = rospy.Publisher('/log', String, queue_size =10 )
        rospy.Subscriber('/cop/prox_sensors', prox_sensor, self.proximity_callback)
        rospy.Subscriber('/cop/line_sensors', line_sensor, self.line_callback)
        rospy.Subscriber('cop/rgb_image', Image, self.image_callback)
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()
        self.prox_data = None
        self.line_data = None
        self.image_data = None
        self.state = "MOVE_FORWARD"  # Initial state
        self.prev_state = None
        self.obstacle_timer = 0
        self.direction = 'RIGHT'
        self.turn_direction = 0.5
        self.turn_counter = 0
        self.reorient_counter = 0
        self.REORIENT_THRESHOLD = 15  # Adjust this value as needed
        self.prev_turn_direction = 0.0
    def image_callback(self,data):
        self.image_data = data
    def line_callback(self,data):
        self.line_data = data

    def proximity_callback(self, data):
        self.prox_data = data

    def stop(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel_msg)

    def move_forward(self, speed=0.2):
        self.vel_msg.linear.x = speed
        self.vel_msg.angular.z = 0.0  # Ensure no rotation
        self.cmd_vel_pub.publish(self.vel_msg)
        #rospy.loginfo("Moving forward now...")
        
    def rotate_and_move(self, angular_speed, linear_speed=0.15):
        self.vel_msg.linear.x = linear_speed
        self.vel_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.vel_msg)
        
        if angular_speed > 0:
            self.turn_counter += 0.5
        else:
            self.turn_counter -= 0.5

    def rotate(self, angular_speed):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.vel_msg)
        
        if angular_speed > 0:
            self.turn_counter += 1
        else:
            self.turn_counter -= 1
            
    def front_ll_clear(self):
        
        if self.prox_data.prox_front_left_left == 0 or self.prox_data.prox_front_left_left > 0.4:
            return 1
        else:        

            return 0
        
    def front_rr_clear(self):
        
        if self.prox_data.prox_front_right_right == 0 or self.prox_data.prox_front_right_right > 0.4:
            return 1
        else:
            return 0
            
    def front_right_clear(self):
        
        if self.prox_data.prox_front_right == 0 or self.prox_data.prox_front_right > 0.5:
            return 1
        else:
            return 0
        
        
    def front_left_clear(self):
        
        if self.prox_data.prox_front_left == 0 or self.prox_data.prox_front_left > 0.5:
            return 1
        else:
            return 0
        

    def run(self):
        while not rospy.is_shutdown():
            
            if self.state != self.prev_state:
                self.log_pub.publish(f"State: {self.state}")
                self.prev_state = self.state  # Update the previous state

            if not self.prox_data:
                continue  # Wait for sensor data
                
            # Validate individual sensor readings
            if self.prox_data.prox_front is None:
                continue  # or set to default value
            
            # State machine
            if self.state == "MOVE_FORWARD":

                if self.is_obstacle_detected() or self.is_side_obstacle_detected():
                    self.state = "AVOID_OBSTACLE"
                else:
                    self.move_forward()
                
            elif self.state == "AVOID_OBSTACLE":                
                
                if self.is_obstacle_cleared():
                    self.state = "MOVE_FORWARD"
                    self.log_pub.publish(f"Obstacle cleared, moving forward")
                else:
                    self.get_turn_direction()
                    self.state = "ROTATE_AWAY"

                    
            elif self.state == "ROTATE_AWAY":

                self.obstacle_timer +=1
                self.rotate(angular_speed = self.turn_direction)
                
                if self.obstacle_timer > 50:
                    self.state = "STUCK"
                    
                elif self.direction == "RIGHT" and self.front_ll_clear():
                    #rospy.loginfo("Clear on the left, move forward")
                    self.state = "POST_ROTATION"
                    
                elif self.direction == "LEFT" and self.front_rr_clear():
                    self.state = "POST_ROTATION"
                    
            elif self.state == "POST_ROTATION":
                if not self.is_side_obstacle_detected():
                    self.move_forward()
                    
                    if self.direction == "RIGHT" and self.prox_data.prox_front_left_left  == 0:
                        self.reorient_counter += 1
                        
                        if self.reorient_counter >= self.REORIENT_THRESHOLD:
                            self.state = "REORIENT"
                            self.reorient_counter = 0  # Reset the counter
                            
                    elif self.direction == "LEFT" and self.prox_data.prox_front_right_right == 0:
                        self.reorient_counter += 1
                        if self.reorient_counter >= self.REORIENT_THRESHOLD:
                            self.state = "REORIENT"
                            self.reorient_counter = 0  # Reset the counter
                    else:
                        self.reorient_counter = 0  # Reset if sensor detects obstacle again
                else:
                    self.stop()
                    self.state = "AVOID_OBSTACLE"
                    self.reorient_counter = 0  # Reset the counter
                    
            elif self.state == "REORIENT":
                
                if self.is_obstacle_detected():
                    self.log_pub.publish(f"Obstacle detected during reorientation.")
                    
                    self.state = "AVOID_OBSTACLE"

                elif self.turn_counter != 0:
                    local_direction = -0.4 if self.turn_counter > 0 else 0.4
                    self.rotate(angular_speed = local_direction)
                    
                else:
                    self.log_pub.publish(f"Reoriented north, proceeding onwards.")
                    self.state = "MOVE_FORWARD"
                    self.obstacle_timer = 0
        
                
            elif self.state == "STUCK":
                self.stop()
                self.log_pub.publish(f"Robot stuck. Reversing from obstacle.")
                self.move_forward(speed=-0.1)  # Reverse
                rospy.sleep(1)
                self.state = "AVOID_OBSTACLE"
                self.obstacle_timer = 0
                                                                
            self.rate.sleep()
            
    #Perhaps could benefit from a buffer time for forward sensor detection!

    def is_obstacle_detected(self):
        threshold = 0.3
        return any(0 < s <= threshold for s in [
            self.prox_data.prox_front,
            self.prox_data.prox_front_left,
            self.prox_data.prox_front_right
        ])
    
    def is_side_obstacle_detected(self):
        threshold = 0.25
        return any(0 < s <= threshold for s in [
                self.prox_data.prox_front_right_right,
                self.prox_data.prox_front_left_left
                ])

    def is_obstacle_cleared(self):
        clear_threshold = 0.6
        return all(s > clear_threshold or s == 0 for s in [
            self.prox_data.prox_front,
            self.prox_data.prox_front_left,
            self.prox_data.prox_front_right,
            self.prox_data.prox_front_right_right,
            self.prox_data.prox_front_left_left
        ])
    
    def is_all_clear(self):
        all_clear_threshold = 0.5
        return all(s == 0 for s in [
            self.prox_data.prox_front,
            self.prox_data.prox_front_left,
            self.prox_data.prox_front_right,
            self.prox_data.prox_front_right_right,
            self.prox_data.prox_front_left_left
        ])
    
    def is_rear_clear(self):
        all_clear_threshold = 0.6
        return all(s == 0 for s in [
            self.prox_data.prox_back,
            self.prox_data.prox_back_left,
            self.prox_data.prox_back_right,
            self.prox_data.prox_back_right_right,
            self.prox_data.prox_back_left_left
        ])
        

    def get_turn_direction(self):
        
        front_diff = self.prox_data.prox_front_left - self.prox_data.prox_front_right
        if abs(front_diff) < 0.05 and self.turn_counter == 0:
            #If difference is small enough and turn counter is zero, head on collision.
            
            self.direction  = "RIGHT"
            self.turn_direction = 0.4
            
        elif self.prox_data.prox_front_left_left < self.prox_data.prox_front_right_right:
            #rhs sensor is closer, so rotate left!
            self.direction = "LEFT"
            self.turn_direction  = -0.4
            
        else:
            
            self.direction = "RIGHT"
            self.turn_direction  = 0.4
        


if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass