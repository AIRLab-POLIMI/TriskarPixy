#include "rosserial.hpp"

ros::NodeHandle nh;

const char* setpointName = "cmd_vel";
const char* twist_name = "vel";

const float encoderFrequency = 100;

namespace rosserial {

std::function<void(const geometry_msgs::Twist&)> RosSerialPublisher::rosCallback;

RosSerialPublisher::RosSerialPublisher(const char* name,
		core::os::Thread::Priority priority) :
		CoreNode::CoreNode(name, priority),
		twist_pub(twist_name, &ros_twist_msg),
		setpoint_sub(setpointName, RosSerialPublisher::setpointCallback)
{
	_workingAreaSize = 512;
	twist = false;
}

bool RosSerialPublisher::onPrepareMW() {
	rosCallback	= std::bind(&RosSerialPublisher::setpointCallbackPrivate, this, std::placeholders::_1);

	subscribe(_subscriberTwist, twist_name);
	_subscriberTwist.set_callback(twistCallback);


	advertise(_publisher, setpointName);

	return true;
}

bool RosSerialPublisher::twistCallback(const core::triskar_msgs::Velocity& msg,
			   void* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);


	tmp->ros_twist_msg.angular.z = msg.angular;
	tmp->ros_twist_msg.angular.x = 0;
	tmp->ros_twist_msg.angular.y = 0;

	tmp->ros_twist_msg.linear.x = msg.linear[0];
	tmp->ros_twist_msg.linear.y = msg.linear[1];
	tmp->ros_twist_msg.linear.z = 0;

	tmp->twist = true;

	return true;
}

bool RosSerialPublisher::onLoop() {

	if(this->spin(core::os::Time::ms(1)))
	{
		if(twist)
		{
			twist_pub.publish(&ros_twist_msg);
			twist = false;
		}
	}

	nh.spinOnce();

	core::os::Thread::sleep(core::os::Time::ms(10));


	return true;
}

bool RosSerialPublisher::onStart() {
	nh.initNode();
	nh.advertise(twist_pub);

	nh.subscribe(setpoint_sub);


	nh.spinOnce();
	core::os::Thread::sleep(core::os::Time::ms(100));

	return true;
}


void RosSerialPublisher::setpointCallback(const geometry_msgs::Twist& setpoint_msg)
{
	rosCallback(setpoint_msg);
}

void RosSerialPublisher::setpointCallbackPrivate(const geometry_msgs::Twist& setpoint_msg)
{
	 core::triskar_msgs::Velocity* msgp;

	 if (_publisher.alloc(msgp)) {
		 msgp->linear[0] = setpoint_msg.linear.x;
		 msgp->linear[1] = setpoint_msg.linear.y;
		 msgp->angular = setpoint_msg.angular.x;

		 _publisher.publish(*msgp);
	 }
}

}
