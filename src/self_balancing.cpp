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

// Converting pitch from radians to degrees
double Convert(double radian)
{
    double pi = 3.14159;
    return(radian * (180 / pi));
}

// Declaring all variables used
double Kp = 0.03*1.2*1.1/10.0*0.1*5*1.5*1.3, Ki = 0.001, Kd = 0.1*1.2*10*3*2/5*2.5*0.5;
double pitch, pitch_rate = 0, pitch_error = 0, prev_pitch_error = 0, pitch_error_difference = 0, pitch_desired = -17.00, e;
double pitch_area = 0, correction = 0, P_term, D_term, I_term;
  
class self_balancing : public rclcpp::Node
{
  // Subscriber and publisher node
  public:
    self_balancing()
    : Node("selfbalancing")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/demo/imu", 10, std::bind(&self_balancing::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel",10);   
        timer_ = this->create_wall_timer(
        500ms, std::bind(&self_balancing::timer_callback, this));   
    }
    
  protected:
    
    // Callback function for subscriber
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
     // Assigning value of orientation.y taken from imu
     pitch = msg->orientation.y;
     e = Convert(pitch);
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf;", e);
     
     // Pitch error, pitch error difference, previous pitch error, pitch rate, pitch area 
     pitch_error = 5*(e-pitch_desired);
     pitch_rate = pitch_error - prev_pitch_error;
     pitch_area += pitch_error ;
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf;", pitch_area);
     
     // Making pitch_area zero everytime it crosses maximum or minimum value
     if(pitch_area > 5.0/1.4)
     {
       pitch_area = 0.0;
     }
     else if(pitch_area < -5.0/1.4)
     {
      pitch_area = 0.0;
     }
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf','%lf','%lf','%lf' and '%lf'", pitch_error,pitch_error_difference,prev_pitch_error,pitch_rate,pitch_area);
     
     // Calculating PID terms
     P_term = Kp*pitch_error;
     D_term = Kd*pitch_rate;
     I_term = pitch_area*Ki;
     correction = P_term + D_term + I_term;
     prev_pitch_error = pitch_error;
     RCLCPP_INFO(this->get_logger(), "Publishing: '%lf','%lf','%f' and '%f'", P_term,D_term,I_term,correction);    
   }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;  
    
    // Callback function for publisher 
    void timer_callback() {
      {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = correction;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
      publisher_->publish(message);
      }
     }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; 
  
};

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<self_balancing>());
  rclcpp::shutdown();
  return 0;
}
