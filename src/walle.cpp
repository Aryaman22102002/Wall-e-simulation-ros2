#include <functional>  // Arithmetic, comparisons, and logical operations
#include <memory>  // Dynamic memory management
#include <string>  // String functions
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
double Kp = 0.03*1.2*1.1/10.0*0.1*5*1.5*1.3*1.5*0.5*0.5, Ki = 0.001, Kd = 0.1*1.2*10*3*2/5*2.5*0.5;
double pitch, pitch_rate = 0, pitch_error = 0, prev_pitch_error = 0, pitch_error_difference = 0, pitch_desired = -3.00, e;
double pitch_area = 0, correction = 0, P_term, D_term, I_term;
double r = 0.0, error = 0.0, prev_error = 0.0, error_diff = 0.0, error_area = 0.0;
double kp = 0.4, kd = 0.0, ki = 0.0, corrections = 0.0, ang_speed = 0.0,speed = 0.2;
int direction = 0;
  
class walle : public rclcpp::Node
{
  // Subscriber and publisher node
  public:
    walle()
    : Node("WallE")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/demo/imu", 10, std::bind(&walle::topic_callback, this, _1));
      
      subscription1_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/image_raw", 10, std::bind(&walle::topic_callbacks, this, _1));
      
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel",10);   
        timer_ = this->create_wall_timer(
        10ms, std::bind(&walle::timer_callback, this));   
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
    
   void topic_callbacks(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      // Callback function for subscriber
      int count = 0, index = 0, flag = 1;
      
      // Reading values of 'r' from rbg values 
      for(int i = 0; i < 96; i+=3)
      {
       r = msg->data[i];
       RCLCPP_INFO(this->get_logger(), "Publishing: '%lf;", r);
       
       // Checking for white pixels
       if( r > 125.00)
       {
         if(flag == 1)
         {
           index = i+1;      // Assigning position of first white pixel to variable index
           flag = 0;
         }
          count ++;         // Counting total number of white pixels
       }
      }
   
       if(count == 7)                 // If we have all white pixels in range of camera
       {
         if(index <= 12)               // Checking position of first white pixel with respect to ideal position of first white pixel
         {
           error = 16 - index - 4;     // Calculating error by checking for deviation between position of current centre white pixel and ideal centre white pixel if deviation is on the left side
           direction = 1;
         }
         else
         {
           error = index + 4 - 17;     // Calculating error by checking for deviation between position of current centre white pixel and ideal centre white pixel if deviation is on the right side
           direction = 2;
         }
       }
       else if(count >= 2 && count < 15)   // If we don't have all white pixels in range of camera
       {
         if(index < 12)
         {
           error = 12 - index;          // Calculating error by checking for deviation between position of current first white pixel and ideal first white pixel if deviation is on the left side
           direction = 1;
         }
         else if(index > 21)
         {
           error = index - 21;         // Calculating error by checking for deviation between position of current first white pixel and ideal first white pixel if deviation is on the right side
           direction = 2;
         }
         if(count > 15)
         {
          //error = 12 - index;          // Calculating error by checking for deviation between position of current first white pixel and ideal first white pixel if deviation is on the left side
           direction = 0;
         }
       }
        
        
      // Calculating PID terms
        error_area += error;
        error_diff = error - prev_error;
        prev_error = error;
         ang_speed = kp*error + kd*error_diff + ki*error_area;
   
     }   
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription1_; 
    
    // Callback function for publisher 
    void timer_callback() {
     RCLCPP_INFO(this->get_logger(), "Publishing: 'Hi''%lf','%lf'",correction,ang_speed);
    if(e>-10.000000 && e<5.000000){//if balanced state,then line follow
      if(direction == 1)
    {  
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = speed;
      message.angular.z = -ang_speed ;
      publisher_->publish(message);
    }
    else if(direction == 2)
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = speed;
      message.angular.z = ang_speed ;
      publisher_->publish(message);
    }}else
    {
     auto message = geometry_msgs::msg::Twist();
     message.linear.x = correction;
     message.angular.z = 0.0 ;
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
  rclcpp::spin(std::make_shared<walle>());
  rclcpp::shutdown();
  return 0;
}
