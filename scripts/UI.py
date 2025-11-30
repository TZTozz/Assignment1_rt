#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.duration import Duration

class UI(Node):

    def __init__(self):
        super().__init__('UI')
        self.publisher1_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.publisher2_ = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)

        self.subscription = self.create_subscription(Bool, 'stop', self.listener_callback, 10)
        
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #Time
        self.start_time = self.get_clock().now()
        self.lastTime = self.start_time - Duration(seconds = 2.0) 
        self.duration = 1.0  #(s)

        self.message = Twist()
        self.needStop = False
    
    def listener_callback(self, msg):
        self.needStop = msg.data
        

    def timer_callback(self):
        #Computed the elapsed time
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.lastTime).nanoseconds / 1e9
        
        if not self.needStop:
            if elapsed_time > self.duration:        #More than a second
                self.numTarta = int(input("Which turtle do you want to move? (1 or 2) "))
                self.vel = float(input("Linear velocity? "))
                self.angular = float(input("Angular velocity? "))
                self.message.linear.x = self.vel
                self.message.angular.z = self.angular
                self.get_logger().info(f'The velocity of turtle_{self.numTarta} is {self.message.linear}')
                
                if self.numTarta == 1:
                    self.publisher1_.publish(self.message)
                    self.lastTime = self.get_clock().now()
                elif self.numTarta == 2:
                    self.publisher2_.publish(self.message)
                    self.lastTime = self.get_clock().now()
                else: self.get_logger().info(f'Error in the selection of the turtle: "{self.numTarta}"')

            else:       #Less than a second
                self.get_logger().info(f'Still sending to turtle_{self.numTarta} the linear velocity of {self.vel} and angular velocity of {self.angular}')
                if self.numTarta == 1:
                    self.publisher1_.publish(self.message)
                elif self.numTarta == 2:
                    self.publisher2_.publish(self.message)
        else:                           #Stops the turtles
            self.message.linear.x = 0.0
            self.message.angular.z = 0.0
            if self.numTarta == 1:
                    self.publisher1_.publish(self.message)
            elif self.numTarta == 2:
                    self.publisher2_.publish(self.message)
            self.get_logger().info(f'Stopped turtle_{self.numTarta}')

        

            

def main(args=None):
    rclpy.init(args=args)
    node = UI()

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Main").info("Nodo terminato.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()