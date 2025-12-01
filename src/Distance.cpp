#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>

#define threshold 2

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
            
            publisherDistance_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);
            publisherStop_ = this->create_publisher<std_msgs::msg::Bool>("stop", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Distance::timer_callback, this));
            msg_stop.data = false;
        }
    
    private:
        void topic_callback1(const turtlesim::msg::Pose::SharedPtr msg){
            //RCLCPP_INFO(this->get_logger(), "The positon is: %f, %f '", msg->x, msg->y);
            x1_ = msg->x;
            y1_ = msg->y;
        }

        void topic_callback2(const turtlesim::msg::Pose::SharedPtr msg){
            //RCLCPP_INFO(this->get_logger(), "The positon is: %f, %f '", msg->x, msg->y);
            x2_ = msg->x;
            y2_ = msg->y;
        }

        void timer_callback()
        {
            //Compute the distance
            float xDistance_ = fabs(x1_ - x2_);
            float yDistance_ = fabs(y1_ - y2_);

            
            msg_distance.data = sqrt(pow(xDistance_, 2) + pow(yDistance_, 2));

            RCLCPP_INFO(this->get_logger(), "The distance is: %f", msg_distance.data);
            publisherDistance_->publish(msg_distance);

            if (msg_distance.data < threshold || x1_ > 10.0 || x1_ < 1.0 || y1_ > 10.0 || y1_ < 1.0 || x2_ > 10.0 || x2_ < 1.0 || y2_ > 10.0 || y2_ < 1.0)
            {
                msg_stop.data = true;
                publisherStop_->publish(msg_stop);
                RCLCPP_INFO(this->get_logger(), "Stopping the turtles");
                alreadyStopped = true;
            }
            else
            {
                if (alreadyStopped) 
                {
                    msg_stop.data = false;
                    publisherStop_->publish(msg_stop);
                    RCLCPP_INFO(this->get_logger(), "The turtles can move");
                    alreadyStopped = false;
                }
            }

        }
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription1_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription2_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisherDistance_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisherStop_;
        rclcpp::TimerBase::SharedPtr timer_;
        std_msgs::msg::Float32 msg_distance;
        std_msgs::msg::Bool msg_stop;
        float x1_, y1_, x2_, y2_;
        bool alreadyStopped = false;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Distance>());
    rclcpp::shutdown();
    return 0;
}