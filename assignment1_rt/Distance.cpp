#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class Distance: public rclcpp::Node
{
    public:
        Distance(): Node("distance_node")
        {
            //bind == connection. _1 is a place older 
            //topic_callback is a function executed every time a massage is recived in the topic
            subscription_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&Distance::topic_callback, this, _1));
            
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Distance::timer_callback, this));
        }
    
    private:
        void topic_callback(const turtlesim::msg::Pose::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "The positon is: %f, %f '", msg->x, msg->y);
            x_ = msg->x;
            y_ = msg->y;
        }

        void timer_callback()
        {

            RCLCPP_INFO(this->get_logger(), "Publishing: Linear: %f, Angular: %f", message.linear.x, message.angular.z);
            publisher_->publish(message);
        }
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::Twist message;
        float x_, y_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Distance>());
    rclcpp::shutdown();
    return 0;
}