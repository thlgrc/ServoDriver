
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"


using std::placeholders::_1;

class CentralController : public rclcpp::Node
{
public:
  CentralController()
  : Node("central_controller"), steer_(90), brake_(85)
  {
    	ax_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"aX", 10, std::bind(&CentralController::ax_callback, this, _1));
	ay_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"aY", 10, std::bind(&CentralController::ay_callback, this, _1));
	az_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"aZ", 10, std::bind(&CentralController::az_callback, this, _1));
	gx_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"gX", 10, std::bind(&CentralController::gx_callback, this, _1));
	gy_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"gY", 10, std::bind(&CentralController::gy_callback, this, _1));
	gz_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"gZ", 10, std::bind(&CentralController::gz_callback, this, _1));
	  
	temp_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"temp", 10, std::bind(&CentralController::temp_callback, this, _1));
	  
	front_right_speed_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"frontRightSpeed", 10, std::bind(&CentralController::front_right_speed_callback, this, _1));

	pub1_ = create_publisher<std_msgs::msg::UInt8>("frontRightSteerPosition", 10);
   	pub2_ = create_publisher<std_msgs::msg::UInt8>("frontRightBrakePosition", 10);
	pub3_ = create_publisher<std_msgs::msg::Float64>("frontRightTorque", 10);

	timer_ = create_wall_timer(std::chrono::seconds(3), std::bind(&CentralController::timer_callback, this));

  }

private:
	size_t steer_;
	size_t brake_;

	void timer_callback()
  {

    // Create a message to publish
    	auto msg = std_msgs::msg::UInt8();
	auto msg1 = std_msgs::msg::UInt8();
	auto msg2 = std_msgs::msg::Float64();

    	msg.data = steer_;  // Static steering angle data
	msg1.data = brake_; // static brake angle data
	msg2.data = rand() % 3000; // Random torque data

    // Publish to each topic
    	pub1_->publish(msg); //front right steer position
	pub2_->publish(msg1); // front right brake position
	pub3_->publish(msg2); // front right torque
    
  }

  void ax_callback(const std_msgs::msg::Float64 & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "aX: '%f'", msg.data);  
		    
  }
 void ay_callback(const std_msgs::msg::Float64 & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "aY: '%f'", msg.data);  
		    
  }
 void az_callback(const std_msgs::msg::Float64 & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "aZ: '%f'", msg.data);  
		    
  }
 void gx_callback(const std_msgs::msg::Float64 & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "gX: '%f'", msg.data);  
		    
  }
 void gy_callback(const std_msgs::msg::Float64 & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "gY: '%f'", msg.data);  
		    
  }
 void gz_callback(const std_msgs::msg::Float64 & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "gZ: '%f'", msg.data);  
		    
  }

	void temp_callback(const std_msgs::msg::Float64 & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "temp: '%f'", msg.data);  
		    
  }
  void front_right_speed_callback(const std_msgs::msg::Float64 & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "Front right BLDC speed reading: '%f'", msg.data);  
		    
  }
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ax_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ay_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr az_subscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gx_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gy_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gz_subscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr temp_subscription;
	
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_right_speed_subscription;

	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub1_;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub2_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub3_;
  	rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralController>());
  rclcpp::shutdown();
  return 0;
}
