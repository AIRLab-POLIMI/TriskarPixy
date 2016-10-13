/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <Module.hpp>

#include <core/triskar_kinematics/Forward.hpp>
#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>

#include <core/triskar_msgs/Velocity.hpp>
#include <core/triskar_msgs/Speeds.hpp>

namespace core {
namespace triskar_kinematics {
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
   _subscriber[0].set_callback(Forward::callback<0>);
   _subscriber[1].set_callback(Forward::callback<1>);
   _subscriber[2].set_callback(Forward::callback<2>);

   this->subscribe(_subscriber[0], configuration().input_0);
   this->subscribe(_subscriber[1], configuration().input_1);
   this->subscribe(_subscriber[2], configuration().input_2);

   this->advertise(_publisher, configuration().output);

   return true;
}

bool
Forward::onLoop()
{
   triskar_msgs::Velocity* velocity;

   if (this->spin(ModuleConfiguration::SUBSCRIBER_SPIN_TIME)) {
      if (_publisher.alloc(velocity)) {

         /// PUBLISH THE RESULTS
         velocity->linear[0]  = 0;
         velocity->linear[1]  = 0;
         velocity->angular = 0;

         _publisher.publish(velocity);
      }
   }

   return true;
} // Forward::onLoop

}
}
