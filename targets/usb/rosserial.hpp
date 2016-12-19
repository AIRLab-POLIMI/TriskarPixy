#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

//Core msgs
#include <core/sensor_msgs/Proximity.hpp>
#include <core/sensor_msgs/Delta_f32.hpp>
#include <core/pixy_msgs/Pixy.hpp>
#include <core/pixy_msgs/Led.hpp>
#include <core/pixy_msgs/Servo.hpp>
#include <core/triskar_msgs/Velocity.hpp>

//ROS msgs
#include <geometry_msgs/Twist.h>
#include <triskar_msgs/Pixy.h>
#include <triskar_msgs/PixyServo.h>
#include <std_msgs/ColorRGBA.h>
#include <triskar_msgs/Proximity.h>

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
	static void
	setpointCallback(const geometry_msgs::Twist& setpoint_msg);

	static void
	pixyColorCallback(const std_msgs::ColorRGBA& color_msg);

	static void
	pixyServoCallback(const triskar_msgs::PixyServo& servo_msg);

	static bool
	twistCallback(const core::triskar_msgs::Velocity& msg,
				   void* node);

	static bool
	pixyCallback(const core::pixy_msgs::Pixy& msg,
					   void* node);

	static bool
	irCallback(const core::sensor_msgs::Proximity& msg,
					   void* node);

	static bool
	encoderCallback_0(const core::sensor_msgs::Delta_f32& msg,
					   void* node);

	static bool
	encoderCallback_1(const core::sensor_msgs::Delta_f32& msg,
						   void* node);

	static bool
	encoderCallback_2(const core::sensor_msgs::Delta_f32& msg,
						   void* node);

private:
	void setpointCallbackPrivate(const geometry_msgs::Twist& setpoint_msg);
	void pixyColorCallbackPrivate(const std_msgs::ColorRGBA& setpoint_msg);
	void pixyServoCallbackPrivate(const triskar_msgs::PixyServo& servo_msg);

private:
      bool
      onPrepareMW();

      bool
	  onStart();

      bool
      onLoop();

private:
      static std::function<void(const geometry_msgs::Twist&)> rosCallbackTwist;
      static std::function<void(const std_msgs::ColorRGBA&)> rosCallbackPixyColor;
      static std::function<void(const triskar_msgs::PixyServo&)> rosCallbackPixyServo;

private:
    //Nova Core
    core::mw::Subscriber<core::triskar_msgs::Velocity, 5> _subscriberTwist;
    core::mw::Subscriber<core::sensor_msgs::Proximity, 5> _subscriberProximity;
    core::mw::Subscriber<core::pixy_msgs::Pixy, 5> _subscriberPixy;

    core::mw::Subscriber<core::sensor_msgs::Delta_f32, 5> _subscriberEncoder[3];

	core::mw::Publisher<core::triskar_msgs::Velocity> _cmd_publisher;
	core::mw::Publisher<core::pixy_msgs::Led> _led_publisher;
	core::mw::Publisher<core::pixy_msgs::Servo> _servo_publisher;

	bool twist;
	bool pixy;
	bool ir;
	bool encoder[3];

	//ROS
	geometry_msgs::Twist ros_twist_msg;
	triskar_msgs::Pixy ros_pixy_msg;
	triskar_msgs::Proximity ros_ir_msg;
	geometry_msgs::Vector3 ros_enc_msg; //TODO change

	ros::Publisher twist_pub;
	ros::Publisher ir_pub;
	ros::Publisher pixy_pub;
	ros::Publisher enc_pub;
	ros::Subscriber<geometry_msgs::Twist> setpoint_sub;
	ros::Subscriber<std_msgs::ColorRGBA> pixy_led_sub;
	ros::Subscriber<triskar_msgs::PixyServo> pixy_servo_sub;

	//Loop
	core::os::Time _stamp;

};

}
