/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <Module.hpp>

#include <core/triskar_kinematics/Inverse.hpp>
#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>

#include <core/triskar_msgs/Velocity.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

namespace core {
namespace triskar_kinematics {
Inverse::Inverse(
   const char*          name,
   os::Thread::Priority priority
) :
   CoreNode::CoreNode(name, priority),
   CoreConfigurable::CoreConfigurable(name)
{
   _workingAreaSize = 1024;
}

Inverse::~Inverse()
{
   teardown();
}

bool
Inverse::onPrepareMW()
{
   _subscriber.set_callback(Inverse::callback);

   this->subscribe(_subscriber, configuration().velocity_input);
   this->advertise(_wheel_publisher[0], configuration().output_0);
   this->advertise(_wheel_publisher[1], configuration().output_1);
   this->advertise(_wheel_publisher[2], configuration().output_2);

   return true;
}

bool
Inverse::onLoop()
{
   if (!this->spin(ModuleConfiguration::SUBSCRIBER_SPIN_TIME)) {}

   return true;
}

bool
Inverse::callback(
   const triskar_msgs::Velocity& msg,
   void*                                    context
)
{
   Inverse* _this = static_cast<Inverse*>(context);

   actuator_msgs::Setpoint_f32* _speed[3];

   float vx     = msg.linear[0];
   float vy     = msg.linear[1];
   float omega = msg.angular;


   /// DO THE MATH


   for(unsigned int i = 0; i < 3; i++)
   {
	   if (_this->_wheel_publisher[i].alloc(_speed[i])) {
	      /// PUBLISH THE RESULTS
	      _speed[i]->value = 0;

	      if (!_this->_wheel_publisher[i].publish(_speed[i]))
	      {
	         return false;
	      }
	   }
   }

   return true;
} // Inverse::callback
}
}
