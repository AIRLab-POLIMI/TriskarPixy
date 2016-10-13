/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <Module.hpp>

#include <core/differential_drive_kinematics/Inverse.hpp>
#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>

#include <core/differential_drive_msgs/Velocity.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

namespace core {
namespace differential_drive_kinematics {
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
   this->advertise(_left_wheel_publisher, configuration().left_output);
   this->advertise(_right_wheel_publisher, configuration().right_output);

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
   const differential_drive_msgs::Velocity& msg,
   void*                                    context
)
{
   Inverse* _this = static_cast<Inverse*>(context);

   actuator_msgs::Setpoint_f32* left_speed;
   actuator_msgs::Setpoint_f32* right_speed;
   float v     = msg.linear;
   float omega = msg.angular;

   float d  = _this->configuration().distance;
   float lr = _this->configuration().left_radius;
   float rr = _this->configuration().right_radius;

   /// DO THE MATH
   if (_this->_left_wheel_publisher.alloc(left_speed)) {
      /// PUBLISH THE RESULTS
      left_speed->value = (1 / lr) * (v + (d / 2) * omega);

      if (!_this->_left_wheel_publisher.publish(left_speed)) {
         return false;
      }
   }

   if (_this->_right_wheel_publisher.alloc(right_speed)) {
      /// PUBLISH THE RESULTS
      right_speed->value = -(1 / rr) * (v - (d / 2) * omega);

      if (!_this->_right_wheel_publisher.publish(right_speed)) {
         return false;
      }
   }

   return true;
} // Inverse::callback
}
}
