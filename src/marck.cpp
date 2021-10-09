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
  double Kp = -10,Ki = 1.0,Kd = -1.0;
  double pitch_x, pitch_rate = 0, pitch_error = 0, prev_pitch_error = 0, pitch_error_difference = 0;
  double pitch_desired = 0.0;
  double pitch_area = 0,pitch_area_new, correction = 0, P_term, D_term, I_term;
  double speed = 0, speed_max = 0.50;//, speed_min = 1.0;
  
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
        1ms, std::bind(&PublishingSubscriber::timer_callback, this));   
    }
    
  protected:
    
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
     // Assigning value of orientation.y taken from imu
     pitch_x = msg->orientation.y;
     
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf;", pitch_x);
     
     // Pitch error, pitch error difference, previous pitch error, pitch rate, pitch area 
     pitch_error = pitch_x;
     pitch_error_difference = pitch_error - prev_pitch_error;
     prev_pitch_error = pitch_error;
     pitch_rate = (pitch_error_difference);
     pitch_area += pitch_error;
     P_term = Kp*pitch_error;
     // Max and min value of pitch error
     if(pitch_area > 2.0)
     {
       pitch_area_new = -(P_term+1);
     }
     else if(pitch_area < - 2.0)
     {
       pitch_area_new = -(P_term-2);
     }
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf','%lf','%lf','%lf' and '%lf'", pitch_error,pitch_error_difference,prev_pitch_error,pitch_rate,pitch_area_new);
     // Calculating PID terms
     //P_term = Kp*pitch_error;
     D_term = Kd*pitch_rate;
     I_term = Ki*pitch_area_new;
     correction = P_term + D_term + I_term;
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf','%lf','%f' and '%f'", P_term,D_term,I_term,correction);
     
    // speed = correction;
    // if(speed > 0)
    // {
    //   speed = speed_max;
    // } 
    // else if(speed < 0)
   //  {
    //   speed = speed_max;
    // }
   }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;   
      
   void timer_callback() {
      //Conditions for forward and backward velocity
      
      if(P_term < -3.50000 && P_term > -4.500000)
      {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = speed_max ;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
      else if(P_term < -2.50000 && P_term > -3.500000)
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = speed_max + 0.750000;
       RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
       publisher_->publish(message);
      }
      else if(P_term < -1.500000 && P_term > -2.500000)
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = speed_max+1.25;
       RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
       publisher_->publish(message);
      }
      else if(P_term < -0.500000 && P_term > -1.500000)
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = speed_max+1.5;
       RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
      else if(P_term < 0.000000  && P_term > -0.500000 )
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = speed_max+1.75 ;
       RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
        else if(P_term < 4.500000 && P_term > 3.500000)
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = -(speed_max-0.2);
       RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
        else if(P_term < 3.500000 && P_term > 2.500000)
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = -(speed_max-0.30) ;
       RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
        else if(P_term < 2.500000 && P_term > 1.500000)
      {
       auto message = geometry_msgs::msg::Twist();
      message.linear.x = -(speed_max -0.35) ;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
        else if(P_term < 1.500000 && P_term > 0.500000)
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = -(speed_max-1.4);
       RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
       else if (P_term < 0.500000 && P_term > 0.000000)
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = -(speed_max-0.45);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
      else if(P_term < 5.500000 && P_term > 4.500000)
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = -speed_max ;
       RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
      else
      {
       auto message = geometry_msgs::msg::Twist();
       message.linear.x = 0.0;
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