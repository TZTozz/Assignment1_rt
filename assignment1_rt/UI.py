import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class UI(Node):

    def __init__(self):
        super().__init__('UI')
        self.publisher1_ = self.create_publisher(Twist, 'turtle1', 10)
        self.publisher2_ = self.create_publisher(Twist, 'turtle2', 10)
        
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #Time
        self.start_time = self.get_clock().now()
        self.lastTime = self.start_time
        self.duration = 1.0  #(s)

        self.message = Twist()
        

    def timer_callback(self):
        #Computed the elapsed time
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.lastTime)
        

        if elapsed_time > self.duration:        #More than a second
            self.numTarta = input("Which turtle do you want to move? (1 or 2)")
            self.vel = input("Velocity? ")
            self.message.linear = self.vel
            self.get_logger().info(f'The velocity of turtle_"{self.numTarta}" is "{self.message.linear}"')
            
            if self.numTarta == 1:
                self.publisher1_.publish(self.message)
                self.lastTime = self.get_clock().now()
            elif self.numTarta == 2:
                self.publisher2_.publish(self.message)
                self.lastTime = self.get_clock().now()
            else: self.get_logger().info('Error in the selection of the turtle')

        else:       #Less than a second
            self.get_logger().info(f'Still sending to turtle_"{self.numTarta}" the velocity of "{self.vel}"')
            if self.numTarta == 1:
                self.publisher1_.publish(self.message)
            elif self.numTarta == 2:
                self.publisher2_.publish(self.message)
            

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