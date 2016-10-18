#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

//Core msgs
#include <core/triskar_msgs/Velocity.hpp>

//ROS msgs
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
	static void
	setpointCallback(const geometry_msgs::Twist& setpoint_msg);

	static bool
	twistCallback(const core::triskar_msgs::Velocity& msg,
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
	core::mw::Publisher<core::triskar_msgs::Velocity> _publisher;

	bool twist;

	//ROS
	geometry_msgs::Twist ros_twist_msg;

	ros::Publisher twist_pub;
	ros::Subscriber<geometry_msgs::Twist> setpoint_sub;

};

}
