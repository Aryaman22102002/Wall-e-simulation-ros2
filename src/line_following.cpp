#include <rclcpp/rclcpp.hpp>
#include <functional> 
#include <chrono>
#include <memory>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Declaring all variables used
double r = 0.0, error = 0.0, prev_error = 0.0, error_diff = 0.0, error_area = 0.0;
double kp = 0.4, kd = 0.0, ki = 0.0, correction = 0.0, ang_speed = 0.0;
int direction = 0;


class line_following : public rclcpp::Node
{
  // Subscriber and publisher node
  public:
    line_following()
    : Node("linefollowing")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/image_raw", 10, std::bind(&line_following::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel",10);   
        timer_ = this->create_wall_timer(
        10ms, std::bind(&line_following::timer_callback, this));   
    }

  protected:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
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
           index = i+1;                      // Assigning position of first white pixel to variable index
           flag = 0;
         }
          count ++;                          // Counting total number of white pixels
       }
      }
   
       if(count == 10)                       // If we have all white pixels in range of camera
       {
         if(index <= 12)                     // Checking position of first white pixel with respect to ideal position of first white pixel
         {
           error = 16 - index - 4;          // Calculating error by checking for deviation between position of current centre white pixel and ideal centre white pixel if deviation is on the left side
           direction = 1;
         }
         else
         {
           error = index + 4 - 17;         // Calculating error by checking for deviation between position of current centre white pixel and ideal centre white pixel if deviation is on the right side
           direction = 2;
         }
       }
       else if(count >= 5 && count < 10)     // If we don't have all white pixels in range of camera
       {
         if(index < 12)
         {
           error = 12 - index;               // Calculating error by checking for deviation between position of current first white pixel and ideal first white pixel if deviation is on the left side
           direction = 1;
         }
         else if(index > 21)
         {
           error = index - 21;               // Calculating error by checking for deviation between position of current first white pixel and ideal first white pixel if deviation is on the right side
           direction = 2;
         }
       }
        
        
      // Calculating PID terms
        error_area += error;
        error_diff = error - prev_error;
        prev_error = error;
        correction = kp*error + kd*error_diff + ki*error_area;
        ang_speed = correction;
   
     }   
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    
   // Callback function for publisher 
   void timer_callback() {
    
    // Giving angular speed either clockwise or anti-clockwise
    if(direction == 1)
    {  
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 0.2;
      message.angular.z = -ang_speed ;
      publisher_->publish(message);
    }
    else if(direction == 2)
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 0.2;
      message.angular.z = ang_speed ;
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
  rclcpp::spin(std::make_shared<line_following>());
  rclcpp::shutdown();
  return 0;
}
