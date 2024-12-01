#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
#from sensor_msgs.msg import LaserScan
from demo_programs.msg import prox_sensor  # Custom message for proximity sensors

class RobotController:
    def __init__(self):
        # Initialize ROS
        rospy.init_node('robot_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/cop/prox_sensors', prox_sensor, self.proximity_callback)
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()
        self.prox_data = None
        self.state = "MOVE_FORWARD"  # Initial state
        self.obstacle_timer = 0
        self.direction = 'RIGHT'
        self.turn_direction = 0.5
        self.turn_counter = 0

    def proximity_callback(self, data):
        self.prox_data = data

    def stop(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel_msg)

    def move_forward(self, speed=0.2):
        self.vel_msg.linear.x = speed
        self.vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel_msg)
        #rospy.loginfo("Moving forward now...")
        
    def rotate_and_move(self, angular_speed=0.5, linear_speed=0.1):
        self.vel_msg.linear.x = linear_speed
        self.vel_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.vel_msg)
        
        if angular_speed > 0:
            self.turn_counter += 1
        else:
            self.turn_counter -= 1

    def rotate(self, angular_speed=0.5):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.vel_msg)
        
        if angular_speed > 0:
            self.turn_counter += 1
        else:
            self.turn_counter -= 1

    def run(self):
        while not rospy.is_shutdown():
            if not self.prox_data:
                continue  # Wait for sensor data
                
            rospy.loginfo(f"""
                    Proximity Readings 2:
                    Front: {self.prox_data.prox_front}
                    Front Left: {self.prox_data.prox_front_left}
                    Front Right: {self.prox_data.prox_front_right}
                """)
            
            # State machine
            if self.state == "MOVE_FORWARD":
                
                
                #rospy.loginfo("State: Forward")
                if self.is_obstacle_detected():
                    self.state = "AVOID_OBSTACLE"
                else:
                    self.move_forward()

            elif self.state == "AVOID_OBSTACLE":
                rospy.loginfo("State: Avoid Obstacle")
                if self.is_obstacle_cleared():
                    self.state = "MOVE_FORWARD"
                else:
                    self.turn_direction = self.get_turn_direction()
                    rospy.loginfo(f"Moving {self.direction}")
                    self.rotate(self.turn_direction)
                    self.obstacle_timer += 1
                    self.state = "ROTATE_AWAY"
                    if self.obstacle_timer > 100:  # Timeout
                        self.state = "STUCK"

            elif self.state == "STUCK":
                rospy.loginfo("State: Stuck")
                self.stop()
                self.move_forward(speed=-0.1)  # Reverse
                rospy.sleep(1)
                self.state = "MOVE_FORWARD"
                self.obstacle_timer = 0
                
            elif self.state == "HEAD_ON_WALL":
                rospy.loginfo("State: Wall head-on impact")
                
                
            elif self.state == "ROTATE_AWAY":
                # Determine difference to prioritize direction
                difference = self.prox_data.prox_front_right - self.prox_data.prox_front_left
                rospy.loginfo(f"Difference: {difference}, Turn Counter: {self.turn_counter}")
            
                if abs(difference) < 0.01 and self.turn_counter < 99999999:  # Sensors agree
                    self.state = "MOVE_FORWARD"
                else:
                    if self.direction == "RIGHT" and self.prox_data.prox_front_left_left == 0:
                        rospy.loginfo("Right wall passed!")
                        self.state = "REORIENT"
                    elif self.direction == "LEFT" and self.prox_data.prox_front_right_right == 0:
                        rospy.loginfo("Left wall passed!")
                        self.state = "REORIENT"
                    else:
                        self.rotate_and_move(angular_speed=self.turn_direction, linear_speed=0.05)
                    
            elif self.state == "REORIENT":
                rospy.loginfo("State: Reorienting")
                if self.turn_counter == 0:
                    self.state = "MOVE_FORWARD"
                else:
                    direction = -0.5 if self.turn_counter > 0 else 0.5
                    self.rotate(direction)
                    
        self.rate.sleep()

    def is_obstacle_detected(self):
        threshold = 0.4
        return any(0 < s <= threshold for s in [
            self.prox_data.prox_front,
            self.prox_data.prox_front_left,
            self.prox_data.prox_front_right
        ])

    def is_obstacle_cleared(self):
        clear_threshold = 0.5
        return all(s > clear_threshold or s == 0 for s in [
            self.prox_data.prox_front,
            self.prox_data.prox_front_left,
            self.prox_data.prox_front_right
        ])
    
    def is_all_clear(self):
        all_clear_threshold = 0.4
        return all(s > all_clear_threshold or s == 0 for s in [
            self.prox_data.prox_front,
            self.prox_data.prox_front_left,
            self.prox_data.prox_front_right,
            self.prox_data.prox_front_right_right,
            self.prox_data.prox_front_left_left
        ])
        

    def get_turn_direction(self):
        if self.prox_data.prox_front_right > self.prox_data.prox_front_left:
            self.direction = 'LEFT'
            return -0.5  # Turn left
        else:
            self.direction = 'RIGHT'
            return 0.5  # Turn right
        
if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass