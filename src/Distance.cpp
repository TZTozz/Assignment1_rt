#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using std::placeholders::_1;

class Distance: public rclcpp::Node
{
    public:
        Distance(): Node("distance_node")
        {
            //bind == connection. _1 is a place older 
            //topic_callback is a function executed every time a massage is recived in the topic
            subscription1_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&Distance::topic_callback1, this, _1));
            subscription2_ = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10, std::bind(&Distance::topic_callback2, this, _1));
            
            publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Distance::timer_callback, this));
        }
    
    private:
        void topic_callback1(const turtlesim::msg::Pose::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "The positon is: %f, %f '", msg->x, msg->y);
            x1_ = msg->x;
            y1_ = msg->y;
        }

        void topic_callback2(const turtlesim::msg::Pose::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "The positon is: %f, %f '", msg->x, msg->y);
            x2_ = msg->x;
            y2_ = msg->y;
        }

        void timer_callback()
        {
            //Compute the distance
            float xDistance_ = fabs(x1_ - x2_);
            float yDistance_ = fabs(y1_ - y2_);

            float distance = sqrt(pow(xDistance_, 2) + pow(yDistance_, 2));

            if (distance < 2)
            {
                message.linear.x = 0.0;
                message.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Stopping the turtles");
                publisher1_->publish(message);
                publisher2_->publish(message);
            }
        }
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription1_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription2_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher2_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::Twist message;
        float x1_, y1_, x2_, y2_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Distance>());
    rclcpp::shutdown();
    return 0;
}