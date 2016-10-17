#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

//Core msgs
#include <core/triskar_msgs/Velocity.hpp>

//ROS msgs
//#include <sensor_msgs/Imu.h>
//#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#define USE_USB_SERIAL 1
#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <ros.h>

namespace rosserial {
class RosSerialPublisher: public core::mw::CoreNode {
public:
	RosSerialPublisher(const char* name,
	         core::os::Thread::Priority priority = core::os::Thread::PriorityEnum::NORMAL);

public:
	/*static bool
	imuCallback(
	   const core::sensor_msgs::Imu& msg,
	   core::mw::Node*               node);*/


	static void
	setpointCallback(const geometry_msgs::Twist& setpoint_msg);

	/*static bool
	currentLeftCallback(const core::actuator_msgs::Setpoint_f32& msg,
				   core::mw::Node* node);

	static bool
	currentRightCallback(const core::actuator_msgs::Setpoint_f32& msg,
					   core::mw::Node* node);

	static bool
	torqueLeftCallback(const core::actuator_msgs::Setpoint_f32& msg,
				   core::mw::Node* node);

	static bool
	torqueRightCallback(const core::actuator_msgs::Setpoint_f32& msg,
					   core::mw::Node* node);



	static bool
	encoderLeftCallback(const core::sensor_msgs::Delta_f32& msg,
				   core::mw::Node* node);

	static bool
	encoderRightCallback(const core::sensor_msgs::Delta_f32& msg,
					   core::mw::Node* node);*/

private:
	void setpointCallbackPrivate(const geometry_msgs::Twist& setpoint_msg);

private:
      bool
      onPrepareMW();

      bool
	  onStart();

      bool
      onLoop();

private:

      static std::function<void(const geometry_msgs::Twist&)> rosCallback;

private:
    //Nova Core
	/*core::mw::Subscriber<core::sensor_msgs::Imu, 5> _subscriberImu;
	core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 5> _subscriberCurrentLeft;
	core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 5> _subscriberCurrentRight;
	core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 5> _subscriberTorqueLeft;
	core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 5> _subscriberTorqueRight;
	core::mw::Subscriber<core::sensor_msgs::Delta_f32, 5> _subscriberLeft;
	core::mw::Subscriber<core::sensor_msgs::Delta_f32, 5> _subscriberRight;*/
	core::mw::Publisher<core::triskar_msgs::Velocity> _publisher;

	bool currentLeft;
	bool currentRight;
	bool torqueLeft;
	bool torqueRight;
	bool imuNew;
	bool encoderLeft;
	bool encoderRight;

	//ROS
	/*std_msgs::Float32 ros_current_left_msg;
	std_msgs::Float32 ros_current_right_msg;
	std_msgs::Float32 ros_torque_left_msg;
	std_msgs::Float32 ros_torque_right_msg;
	std_msgs::Float32 ros_left_msg;
	std_msgs::Float32 ros_right_msg;*/

	/*ros::Publisher imu_pub;
	ros::Publisher current_left_pub;
	ros::Publisher current_right_pub;
	ros::Publisher torque_left_pub;
	ros::Publisher torque_right_pub;
	ros::Publisher encoder_left_pub;
	ros::Publisher encoder_right_pub;*/
	ros::Subscriber<geometry_msgs::Twist> setpoint_sub;

};

}
