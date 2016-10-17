#include "rosserial.hpp"

ros::NodeHandle nh;

/*const char* imuTopic = "imu";
const char* currentLeftTopic = "current_left";
const char* currentRightTopic = "current_right";
const char* torqueLeftTopic = "torque_left";
const char* torqueRightTopic = "torque_right";
const char* leftName = "encoder_left";
const char* rightName = "encoder_right";*/

const char* setpointName = "cmd_vel";

const float encoderFrequency = 100;

namespace rosserial {

std::function<void(const geometry_msgs::Twist&)> RosSerialPublisher::rosCallback;

RosSerialPublisher::RosSerialPublisher(const char* name,
		core::os::Thread::Priority priority) :
		CoreNode::CoreNode(name, priority),
		/*imu_pub(imuTopic, &ros_imu_msg),
		current_left_pub(currentLeftTopic, &ros_current_left_msg),
		current_right_pub(currentRightTopic, &ros_current_right_msg),
		torque_left_pub(torqueLeftTopic, &ros_torque_left_msg),
		torque_right_pub(torqueRightTopic, &ros_torque_right_msg),
		encoder_left_pub(leftName, &ros_left_msg),
		encoder_right_pub(rightName, &ros_right_msg),*/
		setpoint_sub(setpointName, RosSerialPublisher::setpointCallback)
{
	_workingAreaSize = 512;

	imuNew = false;
	currentLeft = false;
	currentRight = false;
	torqueLeft = false;
	torqueRight = false;
	encoderLeft = false;
	encoderRight = false;
}

bool RosSerialPublisher::onPrepareMW() {
	rosCallback	= std::bind(&RosSerialPublisher::setpointCallbackPrivate, this, std::placeholders::_1);

	/*subscribe(_subscriberImu, imuTopic);
	_subscriberImu.set_callback(imuCallback);

	subscribe(_subscriberCurrentLeft, currentLeftTopic);
	_subscriberCurrentLeft.set_callback(currentLeftCallback);

	subscribe(_subscriberCurrentRight, currentRightTopic);
	_subscriberCurrentRight.set_callback(currentRightCallback);

	subscribe(_subscriberTorqueLeft, torqueLeftTopic);
	_subscriberTorqueLeft.set_callback(torqueLeftCallback);

	subscribe(_subscriberTorqueRight, torqueRightTopic);
	_subscriberTorqueRight.set_callback(torqueRightCallback);

	subscribe(_subscriberLeft, leftName);
	_subscriberLeft.set_callback(encoderLeftCallback);

	subscribe(_subscriberRight, rightName);
	_subscriberRight.set_callback(encoderRightCallback);*/


	advertise(_publisher, setpointName);

	return true;
}

/*bool RosSerialPublisher::imuCallback(
	   const core::sensor_msgs::Imu& msg,
	   core::mw::Node*               node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_imu_msg.orientation.x = msg.orientation[0];
	tmp->ros_imu_msg.orientation.y = msg.orientation[1];
	tmp->ros_imu_msg.orientation.z = msg.orientation[2];
	tmp->ros_imu_msg.orientation.w = msg.orientation[3];

	tmp->ros_imu_msg.linear_acceleration.x = msg.linear_acceleration[0];
	tmp->ros_imu_msg.linear_acceleration.y = msg.linear_acceleration[1];
	tmp->ros_imu_msg.linear_acceleration.z = msg.linear_acceleration[2];

	tmp->ros_imu_msg.angular_velocity.x = msg.angular_velocity[0];
	tmp->ros_imu_msg.angular_velocity.y = msg.angular_velocity[1];
	tmp->ros_imu_msg.angular_velocity.z = msg.angular_velocity[2];

	tmp->ros_imu_msg.header.frame_id = "imu_link";

	core::os::Time stamp = core::os::Time::now();

	auto sec = stamp.s();
	auto nsec = stamp.to_us()*1e3 - sec*1e9;

	tmp->ros_imu_msg.header.stamp.sec = sec;
	tmp->ros_imu_msg.header.stamp.nsec = nsec;



	tmp->imuNew = true;

   return true;
}

bool RosSerialPublisher::currentLeftCallback(
	   const core::actuator_msgs::Setpoint_f32& msg,
	   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_current_left_msg.data = msg.value;

	tmp->currentLeft = true;

   return true;
}

bool RosSerialPublisher::currentRightCallback(
	   const core::actuator_msgs::Setpoint_f32& msg,
	   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_current_right_msg.data = msg.value;

	tmp->currentRight = true;

   return true;
}

bool RosSerialPublisher::torqueLeftCallback(
	   const core::actuator_msgs::Setpoint_f32& msg,
	   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_torque_left_msg.data = msg.value;

	tmp->torqueLeft = true;

   return true;
}

bool RosSerialPublisher::torqueRightCallback(
	   const core::actuator_msgs::Setpoint_f32& msg,
	   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_torque_right_msg.data = msg.value;

	tmp->torqueRight = true;

   return true;
}

bool RosSerialPublisher::encoderLeftCallback(const core::sensor_msgs::Delta_f32& msg,
			   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);


	tmp->ros_left_msg.data = msg.value*encoderFrequency;
	tmp->encoderLeft = true;

	return true;
}

bool RosSerialPublisher::encoderRightCallback(const core::sensor_msgs::Delta_f32& msg,
				   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);


	tmp->ros_right_msg.data = msg.value*encoderFrequency;
	tmp->encoderRight = true;

	return true;
}*/

bool RosSerialPublisher::onLoop() {

	if(this->spin(core::os::Time::ms(1)))
	{
		/*if(imuNew)
		{
			imu_pub.publish(&ros_imu_msg);
			imuNew = false;
		}

		if(currentLeft)
		{
			current_left_pub.publish(&ros_current_left_msg);
			currentLeft = false;
		}

		if(currentRight)
		{
			current_right_pub.publish(&ros_current_right_msg);
			currentRight = false;
		}

		if(torqueLeft)
		{
			torque_left_pub.publish(&ros_torque_left_msg);
			torqueLeft = false;
		}

		if(torqueRight)
		{
			torque_right_pub.publish(&ros_torque_right_msg);
			torqueRight = false;
		}

		if(encoderLeft)
		{
			encoder_left_pub.publish(&ros_left_msg);
			encoderLeft = false;
		}

		if(encoderRight)
		{
			encoder_right_pub.publish(&ros_right_msg);
			encoderRight = false;
		}*/
	}

	nh.spinOnce();

	core::os::Thread::sleep(core::os::Time::ms(10));


	return true;
}

bool RosSerialPublisher::onStart() {
	nh.initNode();
	/*nh.advertise(imu_pub);
	nh.advertise(current_left_pub);
	nh.advertise(current_right_pub);
	nh.advertise(torque_left_pub);
	nh.advertise(torque_right_pub);
	nh.advertise(encoder_left_pub);
	nh.advertise(encoder_right_pub);*/

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
