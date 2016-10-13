/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>

#include <ModuleConfiguration.hpp>

#include <core/differential_drive_kinematics/ForwardConfiguration.hpp>
#include <core/differential_drive_msgs/Velocity.hpp>
#include <core/sensor_msgs/Delta_f32.hpp>

namespace core {
namespace differential_drive_kinematics {
class Forward:
   public mw::CoreNode,
   public mw::CoreConfigurable<ForwardConfiguration>
{
public:
   Forward(
      const char*          name,
      os::Thread::Priority priority = os::Thread::PriorityEnum::NORMAL
   );
   virtual
   ~Forward();

private:
   bool
   onPrepareMW();

   bool
   onLoop();

   static bool
   callback_left(
      const core::sensor_msgs::Delta_f32& msg,
      void*                               context
   );

   static bool
   callback_right(
      const core::sensor_msgs::Delta_f32& msg,
      void*                               context
   );


private:
   mw::Subscriber<core::sensor_msgs::Delta_f32, 2>  _subscriber_left;
   mw::Subscriber<core::sensor_msgs::Delta_f32, 2>  _subscriber_right;
   mw::Publisher<differential_drive_msgs::Velocity> _publisher;

   float _speed_right;
   float _speed_left;
};
}
}
