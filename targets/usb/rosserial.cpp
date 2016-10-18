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
		ir_pub(twist_name, &ros_ir_msg),
		pixy_pub(twist_name, &ros_pixy_msg),
		setpoint_sub(setpointName, RosSerialPublisher::setpointCallback)
{
	_workingAreaSize = 512;
	twist = false;
	pixy = false;
	ir = false;
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


bool RosSerialPublisher::pixyCallback(const core::pixy_msgs::Pixy& msg,
				   void* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_pixy_msg.checksum = msg.checksum;
	tmp->ros_pixy_msg.signature = msg.signature;
	tmp->ros_pixy_msg.x = msg.x;
	tmp->ros_pixy_msg.y = msg.y;
	tmp->ros_pixy_msg.width = msg.width;
	tmp->ros_pixy_msg.height = msg.height;

	tmp->pixy = true;

	return true;
}

bool RosSerialPublisher::irCallback(const core::sensor_msgs::Proximity& msg,
				   void* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	for( unsigned int i = 0; i < 8; i++)
		tmp->ros_ir_msg.range[i] = msg.value[i];


	tmp->ir = true;

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

		if(ir)
		{
			ir_pub.publish(&ros_ir_msg);
			ir = false;
		}

		if(pixy)
		{
			pixy_pub.publish(&ros_pixy_msg);
			pixy = false;
		}
	}

	nh.spinOnce();

	core::os::Thread::sleep(core::os::Time::ms(10));


	return true;
}

bool RosSerialPublisher::onStart()
{
	nh.initNode();
	nh.advertise(twist_pub);
	nh.advertise(ir_pub);
	nh.advertise(pixy_pub);

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
