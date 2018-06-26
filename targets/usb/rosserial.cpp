#include "rosserial.hpp"

ros::NodeHandle nh;

const char* setpointName = "cmd_vel";
const char* twist_name = "vel";
const char* enc_name = "enc";
const char* ir_name = "proximity";
const char* pixy_name = "pixy";
const char* pixy_led_name = "pixy_led";
const char* pixy_servo_name = "pixy_servo";

const float encoderFrequency = 100;

namespace rosserial {

std::function<void(const geometry_msgs::Twist&)> RosSerialPublisher::rosCallbackTwist;
std::function<void(const std_msgs::ColorRGBA&)> RosSerialPublisher::rosCallbackPixyColor;
std::function<void(const triskar_msgs::PixyServo&)> RosSerialPublisher::rosCallbackPixyServo;

RosSerialPublisher::RosSerialPublisher(const char* name,
		core::os::Thread::Priority priority) :
		CoreNode::CoreNode(name, priority),
		twist_pub(twist_name, &ros_twist_msg),
		ir_pub(ir_name, &ros_ir_msg),
		pixy_pub(pixy_name, &ros_pixy_msg),
		enc_pub(enc_name, &ros_enc_msg),
		setpoint_sub(setpointName, RosSerialPublisher::setpointCallback),
		pixy_led_sub(pixy_led_name, RosSerialPublisher::pixyColorCallback),
		pixy_servo_sub(pixy_servo_name, RosSerialPublisher::pixyServoCallback)
{
	_workingAreaSize = 1024;
	twist = false;
	pixy = false;
	ir = false;

	for(unsigned int i = 0; i < 3; i++)
		encoder[i] = false;
}

bool RosSerialPublisher::onPrepareMW() {
	rosCallbackTwist = std::bind(&RosSerialPublisher::setpointCallbackPrivate, this, std::placeholders::_1);
	rosCallbackPixyColor = std::bind(&RosSerialPublisher::pixyColorCallbackPrivate, this, std::placeholders::_1);
	rosCallbackPixyServo = std::bind(&RosSerialPublisher::pixyServoCallbackPrivate, this, std::placeholders::_1);

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

	advertise(_cmd_publisher, setpointName);
	advertise(_led_publisher, pixy_led_name);
	advertise(_servo_publisher, pixy_servo_name);

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
	rosCallbackTwist(setpoint_msg);
}

void RosSerialPublisher::pixyColorCallback(const std_msgs::ColorRGBA& color_msg)
{
	rosCallbackPixyColor(color_msg);
}

void RosSerialPublisher::pixyServoCallback(const triskar_msgs::PixyServo& servo_msgs)
{
	rosCallbackPixyServo(servo_msgs);
}

void RosSerialPublisher::setpointCallbackPrivate(const geometry_msgs::Twist& setpoint_msg)
{
	 core::triskar_msgs::Velocity* msgp;

	 if (_cmd_publisher.alloc(msgp))
	 {
		 msgp->linear[0] = setpoint_msg.linear.x;
		 msgp->linear[1] = setpoint_msg.linear.y;
		 msgp->angular = setpoint_msg.angular.z;

		 _cmd_publisher.publish(*msgp);
	 }
}

void RosSerialPublisher::pixyColorCallbackPrivate(const std_msgs::ColorRGBA& color_msg)
{
	core::pixy_msgs::Led* msgp;

	 if (_led_publisher.alloc(msgp))
	 {
		 msgp->color[0] = color_msg.r*255;
		 msgp->color[1] = color_msg.g*255;
		 msgp->color[2] = color_msg.b*255;

		 _led_publisher.publish(*msgp);
	 }
}

void RosSerialPublisher::pixyServoCallbackPrivate(const triskar_msgs::PixyServo& servo_msg)
{
	core::pixy_msgs::Servo* msgp;

	 if (_servo_publisher.alloc(msgp))
	 {
		 msgp->pan = servo_msg.pan;
		 msgp->tilt = servo_msg.tilt;

		 _servo_publisher.publish(*msgp);
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
	nh.subscribe(pixy_led_sub);
	nh.subscribe(pixy_servo_sub);


	nh.spinOnce();
	core::os::Thread::sleep(core::os::Time::ms(100));

	_stamp = core::os::Time::now();

	return true;
}


bool RosSerialPublisher::onLoop() {

	core::os::Thread::sleep_until(_stamp + core::os::Time::ms(10));

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
