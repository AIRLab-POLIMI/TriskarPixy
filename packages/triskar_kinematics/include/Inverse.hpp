/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

#include <core/differential_drive_kinematics/InverseConfiguration.hpp>
#include <core/differential_drive_msgs/Velocity.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

namespace core {
namespace differential_drive_kinematics {
class Inverse:
   public mw::CoreNode,
   public mw::CoreConfigurable<InverseConfiguration>
{
public:
   Inverse(
      const char*          name,
      os::Thread::Priority priority = os::Thread::PriorityEnum::NORMAL
   );
   virtual
   ~Inverse();

private:
   bool
   onPrepareMW();

   bool
   onLoop();

   static bool
   callback(
      const differential_drive_msgs::Velocity& msg,
      void*                                    context
   );


private:
   mw::Subscriber<differential_drive_msgs::Velocity, 5> _subscriber;
   mw::Publisher<actuator_msgs::Setpoint_f32> _left_wheel_publisher;
   mw::Publisher<actuator_msgs::Setpoint_f32> _right_wheel_publisher;
};
}
}
