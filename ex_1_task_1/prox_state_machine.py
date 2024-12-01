#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
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

    def rotate(self, angular_speed=0.5):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.vel_msg)

    def run(self):
        while not rospy.is_shutdown():
            if not self.prox_data:
                continue  # Wait for sensor data
            
            # State machine
            if self.state == "MOVE_FORWARD":
                rospy.loginfo("State: Forward")
                if self.is_obstacle_detected():
                    self.state = "AVOID_OBSTACLE"
                else:
                    self.move_forward()

            elif self.state == "AVOID_OBSTACLE":
                rospy.loginfo("State: Avoid Obstacle")
                if self.is_obstacle_detected():
                    self.state = "MOVE_FORWARD"
                else:
                    turn_direction = self.get_turn_direction()
                    rospy.loginfo(f"State: Moving {self.direction}")
                    self.rotate(turn_direction)
                    self.obstacle_timer += 1
                    if self.obstacle_timer > 50:  # Timeout
                        self.state = "STUCK"

            elif self.state == "STUCK":
                rospy.loginfo("State: Stuck")
                self.stop()
                self.move_forward(speed=-0.1)  # Reverse
                rospy.sleep(1)
                self.state = "MOVE_FORWARD"
                self.obstacle_timer = 0

            self.rate.sleep()

    def is_obstacle_detected(self):
        threshold = 0.3
        return any(0 < s <= threshold for s in [
            self.prox_data.prox_front,
            self.prox_data.prox_front_left,
            self.prox_data.prox_front_right
        ])

    def is_obstacle_cleared(self):
        clear_threshold = 0.4
        return all(s > clear_threshold or s == 0 for s in [
            self.prox_data.prox_front,
            self.prox_data.prox_front_left,
            self.prox_data.prox_front_right
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