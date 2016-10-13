/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <Module.hpp>

#include <core/differential_drive_kinematics/Forward.hpp>
#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>

#include <core/differential_drive_msgs/Velocity.hpp>
#include <core/differential_drive_msgs/Speeds.hpp>

namespace core {
namespace differential_drive_kinematics {
Forward::Forward(
   const char*          name,
   os::Thread::Priority priority
) :
   CoreNode::CoreNode(name, priority),
   CoreConfigurable::CoreConfigurable(name)
{
   _workingAreaSize = 1024;
}

Forward::~Forward()
{
   teardown();
}

bool
Forward::onPrepareMW()
{
   _subscriber_left.set_callback(Forward::callback_left);
   _subscriber_right.set_callback(Forward::callback_right);

   this->subscribe(_subscriber_left, configuration().left_input);
   this->subscribe(_subscriber_right, configuration().right_input);
   this->advertise(_publisher, configuration().output);

   return true;
}

bool
Forward::onLoop()
{
   differential_drive_msgs::Velocity* velocity;

   if (this->spin(ModuleConfiguration::SUBSCRIBER_SPIN_TIME)) {
      if (_publisher.alloc(velocity)) {
         float d  = configuration().distance;
         float lr = configuration().left_radius;
         float rr = configuration().right_radius;

         /// PUBLISH THE RESULTS
         velocity->linear  = ((_speed_left * lr) - (_speed_right * rr)) * 0.5f * 2.0f * core::utils::math::constants::pi<float>();
         velocity->angular = ((_speed_left * lr) + (_speed_right * rr)) / d * 0.5f * 2.0f * core::utils::math::constants::pi<float>();

         _publisher.publish(velocity);
      }
   }

   return true;
} // Forward::onLoop

bool
Forward::callback_left(
   const core::sensor_msgs::Delta_f32& msg,
   void*                               context
)
{
   Forward* _this = static_cast<Forward*>(context);

   _this->_speed_left = msg.value;

   return true;
} // Forward::callback_left

bool
Forward::callback_right(
   const core::sensor_msgs::Delta_f32& msg,
   void*                               context
)
{
   Forward* _this = static_cast<Forward*>(context);

   _this->_speed_right = msg.value;

   return true;
} // Forward::callback_right
}
}
