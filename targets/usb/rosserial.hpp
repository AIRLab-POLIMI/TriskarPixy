#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

//Core msgs
#include <core/sensor_msgs/Proximity.hpp>
#include <core/pixy_msgs/Pixy.hpp>
#include <core/triskar_msgs/Velocity.hpp>

//ROS msgs
#include <geometry_msgs/Twist.h>
#include <triskar_msgs/Pixy.h>
#include <triskar_msgs/IR.h>

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

	static bool
	twistCallback(const core::triskar_msgs::Velocity& msg,
				   void* node);

	static bool
	pixyCallback(const core::pixy_msgs::Pixy& msg,
					   void* node);

	static bool
	irCallback(const core::sensor_msgs::Proximity& msg,
					   void* node);

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
    core::mw::Subscriber<core::triskar_msgs::Velocity, 5> _subscriberTwist;
    core::mw::Subscriber<core::sensor_msgs::Proximity, 5> _subscriberProximity;
    core::mw::Subscriber<core::pixy_msgs::Pixy, 5> _subscriberPixy;
	core::mw::Publisher<core::triskar_msgs::Velocity> _publisher;

	bool twist;
	bool pixy;
	bool ir;

	//ROS
	geometry_msgs::Twist ros_twist_msg;
	triskar_msgs::Pixy ros_pixy_msg;
	triskar_msgs::IR ros_ir_msg;

	ros::Publisher twist_pub;
	ros::Publisher ir_pub;
	ros::Publisher pixy_pub;
	ros::Subscriber<geometry_msgs::Twist> setpoint_sub;

};

}
