#include "rosserial.hpp"

ros::NodeHandle nh;

const char* setpointName = "cmd_vel";
const char* twist_name = "vel";
const char* enc_name = "enc";
const char* ir_name = "proximity";
const char* pixy_name = "pixy";

const float encoderFrequency = 100;

namespace rosserial {

std::function<void(const geometry_msgs::Twist&)> RosSerialPublisher::rosCallback;

RosSerialPublisher::RosSerialPublisher(const char* name,
		core::os::Thread::Priority priority) :
		CoreNode::CoreNode(name, priority),
		twist_pub(twist_name, &ros_twist_msg),
		ir_pub(ir_name, &ros_ir_msg),
		pixy_pub(pixy_name, &ros_pixy_msg),
		enc_pub(enc_name, &ros_enc_msg),
		setpoint_sub(setpointName, RosSerialPublisher::setpointCallback)
{
	_workingAreaSize = 1024;
	twist = false;
	pixy = false;
	ir = false;

	for(unsigned int i = 0; i < 3; i++)
		encoder[i] = false;
}

bool RosSerialPublisher::onPrepareMW() {
	rosCallback	= std::bind(&RosSerialPublisher::setpointCallbackPrivate, this, std::placeholders::_1);

	_subscriberTwist.set_callback(twistCallback);
	subscribe(_subscriberTwist, twist_name);

	_subscriberProximity.set_callback(irCallback);
	subscribe(_subscriberProximity, ir_name);

	_subscriberPixy.set_callback(pixyCallback);
	subscribe(_subscriberPixy, pixy_name);


	_subscriberEncoder[0].set_callback(encoderCallback_0);
	subscribe(_subscriberEncoder[0], "encoder_0");

	_subscriberEncoder[1].set_callback(encoderCallback_1);
	subscribe(_subscriberEncoder[1], "encoder_1");

	_subscriberEncoder[2].set_callback(encoderCallback_2);
	subscribe(_subscriberEncoder[2], "encoder_2");

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

bool RosSerialPublisher::encoderCallback_0(const core::sensor_msgs::Delta_f32& msg,
				   void* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_enc_msg.x = msg.value;


	tmp->encoder[0] = true;

	return true;
}

bool RosSerialPublisher::encoderCallback_1(const core::sensor_msgs::Delta_f32& msg,
					   void* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_enc_msg.y = msg.value;


	tmp->encoder[1] = true;

	return true;
}

bool RosSerialPublisher::encoderCallback_2(const core::sensor_msgs::Delta_f32& msg,
					   void* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_enc_msg.z = msg.value;


	tmp->encoder[2] = true;

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

bool RosSerialPublisher::onStart()
{
	nh.initNode();
	nh.advertise(twist_pub);
	nh.advertise(ir_pub);
	nh.advertise(pixy_pub);
	nh.advertise(enc_pub);

	nh.subscribe(setpoint_sub);


	nh.spinOnce();
	core::os::Thread::sleep(core::os::Time::ms(100));

	_stamp = core::os::Time::now();

	return true;
}


bool RosSerialPublisher::onLoop() {

	core::os::Thread::sleep_until(_stamp + core::os::Time::hz(100));

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

		if(encoder[0] && encoder[1] && encoder[2])
		{
			enc_pub.publish(&ros_enc_msg);

			for(unsigned int i = 0; i < 3; i++)
				encoder[i] = false;
		}
	}

	nh.spinOnce();

	_stamp = core::os::Time::now();


	return true;
}

}
