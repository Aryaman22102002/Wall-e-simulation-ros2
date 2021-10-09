#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>


using namespace std::chrono_literals;
using std::placeholders::_1;

// Declaring all variables used
  double Kp = 3.0,Ki = 0.1,Kd = 0.8;
  double pitch_x, pitch_rate = 0, pitch_error = 0, prev_pitch_error = 0, pitch_error_difference = 0;
  double pitch_desired = 0.0;
  double pitch_area = 0, correction = 0, P_term, D_term, I_term;
  double speed = 0, speed_max = 0.4, speed_min = 0.2;
  
// Class for subscribing and publishing
class PublishingSubscriber : public rclcpp::Node
{
  // Subscriber and publisher node
  public:
    PublishingSubscriber()
    : Node("publishing_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/demo/imu", 10, std::bind(&PublishingSubscriber::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel",10);   
        timer_ = this->create_wall_timer(
        100ms, std::bind(&PublishingSubscriber::timer_callback, this));   
    }
    
  protected:
    
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
     // Assigning value of orientation.y taken from imu
     pitch_x = msg->orientation.y;
     
     RCLCPP_INFO(this->get_logger(), "Publishing pitch: '%lf;", pitch_x);
     
     // Pitch error, pitch error difference, previous pitch error, pitch rate, pitch area 
     pitch_error = pitch_desired - pitch_x;
     pitch_error_difference = pitch_error - prev_pitch_error;
     prev_pitch_error = pitch_error;
     pitch_rate = (pitch_error_difference);
     pitch_area += pitch_error;
     
     // Max and min value of pitch error
     if(pitch_area > 4.0)
     {
       pitch_area = 4.0;
     }
     else if(pitch_area < - 4.0)
     {
       pitch_area = -4.0;
     }
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf','%lf','%lf','%lf' and '%lf'", pitch_error,pitch_error_difference,prev_pitch_error,pitch_rate,pitch_area);
     // Calculating PID terms
     P_term = Kp*pitch_error;
     D_term = Kd*pitch_rate;
     I_term = Ki*pitch_area;
     correction = P_term + D_term + I_term;
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf','%lf','%f' and '%f'", P_term,D_term,I_term,correction);
     
     speed = fabs(correction);
     if(speed > speed_max)
     {
       speed = speed_max;
     } 
     else if(speed < speed_min)
     {
       speed = speed_min;
     }
   }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;   
      
   void timer_callback() {
      //Conditions for forward and backward velocity
      
      if(pitch_error > 0.142072)
      {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = -speed ;
      RCLCPP_INFO(this->get_logger(), "Publishing speed: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
      else if(pitch_error < 0.142072)
      {
       auto message = geometry_msgs::msg::Twist();
      message.linear.x = 1.8*speed ;
      RCLCPP_INFO(this->get_logger(), "Publishing speed: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
      else if(pitch_error > -1.629693)
       {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = -speed ;
       RCLCPP_INFO(this->get_logger(), "Publishing speed: '%lf'", message.linear.x);
       publisher_->publish(message);
      }
      else
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = 0.0 ;
       RCLCPP_INFO(this->get_logger(), "Publishing speed: '%lf'", message.linear.x);
       publisher_->publish(message);
      }
     }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishingSubscriber>());
  rclcpp::shutdown();
  return 0;
}
