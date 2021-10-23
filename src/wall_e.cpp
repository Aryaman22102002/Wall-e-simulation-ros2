#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory>     // Dynamic memory management
#include <string>     // String functions
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Function to convert radians to degrees
double Convert(double radian)
{
  double pi = 3.14159;
  return (radian * (180 / pi));
}

// Declaring all variables used
double Kp = 0.03 * 1.2 * 1.1 / 10.0 * 0.1 * 5 * 1.5 * 1.3 * 1.5 * 0.5 * 0.5 * 100, Ki = 0.001, Kd = 0.1 * 1.2 * 10 * 3 * 2 / 5 * 2.5 * 0.5;
double kp = 0.18, ki = 0.0, kd = 0.0, speed = 0.1;
double pitch_x, pitch_rate = 0, pitch_error = 0, prev_pitch_error = 0, pitch_error_difference = 0, pitch_desired = -5.0, e;
double pitch_area = 0, correction = 0, P_term1, D_term1, I_term1;

double ang_area = 0, ang_speed = 0, P_term2, D_term2, I_term2;
double r = 0.0, error = 0.0, prev_error = 0.0, error_diff = 0.0, error_area = 0.0;
int direction = 0;
bool balanced_state = false;

class walle : public rclcpp::Node
{
  // Subscriber and publisher node
public:
  walle()
      : Node("wall_e")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/demo/imu", 10, std::bind(&walle::topic_callback, this, _1));

    subscription1_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10, std::bind(&walle::topic_callbacks, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&walle::timer_callback, this));
  }

protected:
  void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    // Assigning value of orientation.y taken from imu
    pitch_x = msg->orientation.y;
    e = Convert(pitch_x);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%lf;", e);

    // Declaring the range of balanced state
    if (e < 6.000000 && e > -6.000000)
    {
      balanced_state = true;
    }

    // Pitch error, pitch error difference, previous pitch error, pitch rate, pitch area
    pitch_error = 5 * (e - pitch_desired);
    pitch_rate = pitch_error - prev_pitch_error;
    pitch_area += pitch_error;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%lf;", pitch_area);

    // Setting up the limits of pitch area
    if (pitch_area > 5.0 / 1.4)
    {
      pitch_area = 0.0;
    }
    else if (pitch_area < -5.0 / 2.0)
    {
      pitch_area = 0.0;
    }
    RCLCPP_INFO(this->get_logger(), "Publishing: '%lf','%lf','%lf','%lf' and '%lf'", pitch_error, pitch_error_difference, prev_pitch_error, pitch_rate, pitch_area);
    // Calculating PID terms
    P_term1 = Kp * pitch_error;
    D_term1 = Kd * pitch_rate;
    I_term1 = pitch_area * Ki;
    correction = P_term1 + D_term1 + I_term1;
    prev_pitch_error = pitch_error;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%lf','%lf','%f' and '%f'", P_term1, D_term1, I_term1, correction);
  }
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;


  // Callback function for subscriber
  void topic_callbacks(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    balanced_state = false;
    int count = 0, index = 0, flag = 1;

    // Reading values of 'r' from rbg values
    for (int i = 0; i < 96; i += 3)
    {
      r = msg->data[i];
      RCLCPP_INFO(this->get_logger(), "Publishing: '%lf;", r);

      // Checking for red pixels
      if (r >= 135.00)
      {
        if (flag == 1)
        {
          index = i + 1;                   // Assigning position of first red pixel to variable index
          flag = 0;
        }
        count++;                           // Counting total number of red pixels
      }
    }

    if (count < 9)                         // If number of red pixels is less than nine
    {
      if (index <= 5) 
      {
        error = 5 - index;                 // Checking devaition on left side
        direction = 1;
      }
      else if(index >= 18)
      {
        error = index - 18 ;               // Checking devaition on right side
        direction = -1;
      }
    }
  
  RCLCPP_INFO(this->get_logger(), "Count: %d", count);
  // Calculating PID terms
  error_area += error;
  error_diff = error - prev_error;
  prev_error = error;
  ang_speed = kp * error + kd * error_diff + ki * error_area;

} 
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription1_;


void timer_callback()

// If bot is not balanced, provide only linear velocity for balancing else provide both linear and angular velocity
{
  if (balanced_state == false)
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = correction;
    message.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.linear.x);
    publisher_->publish(message);
  }
  else
  {
    if (direction == 1)
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = speed;
      message.angular.z = -ang_speed;
      publisher_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Publishing ang: '%lf;", message.angular.z);

    }
    else if (direction == -1)
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = speed;
      message.angular.z = ang_speed;
      publisher_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Publishing ang: '%lf;", message.angular.z);
    }
  }
}
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<walle>());
  rclcpp::shutdown();
  return 0;
}
