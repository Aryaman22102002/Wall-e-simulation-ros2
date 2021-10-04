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
 double a ,b ,c ,Kp = 1,Ki = 0.01,Kd = 0.0001;
  double pitch_x, pitch_rate = 0, pitch_error = 0, prev_pitch_error = 0, pitch_error_difference = 0;
  double pitch_desired = 0.65;
  double pitch_area = 0, correction = 0, P_term, D_term, I_term;
  
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
        500ms, std::bind(&PublishingSubscriber::timer_callback, this));   
    }
    
  protected:
    
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
     // Assigning values of linear accleration taken from imu
     a = msg->linear_acceleration.x;
     b = msg->linear_acceleration.y;
     c = msg->linear_acceleration.z;
     
     // Calculation of pitch
     pitch_x = atan(a/sqrt(b*b + c*c));
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf;", pitch_x);
     
     // Pitch error, pitch error difference, previous pitch error, pitch rate, pitch area 
     pitch_error = pitch_desired - pitch_x;
     pitch_error_difference = pitch_error - prev_pitch_error;
     prev_pitch_error = pitch_error;
     pitch_rate = (pitch_error_difference/0.5);
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
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;   
      
   void timer_callback() {
      //Conditions for forward and backward velocity
      
      if(pitch_error < -0.15)
      {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = -fabs(correction) ;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
      else if(pitch_error > 0.15)
      {
       auto message = geometry_msgs::msg::Twist();
      message.linear.x = fabs(correction) ;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
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
