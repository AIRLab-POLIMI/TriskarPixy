#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/led/Subscriber.hpp>
#include <core/led/Publisher.hpp>
#include <core/pixy_driver/PixyNode.hpp>
#include <core/triskar_kinematics/Forward.hpp>
#include <core/triskar_kinematics/Inverse.hpp>

#include "rosserial.hpp"

// --- BOARD IMPL -------------------------------------------------------------

// --- MISC -------------------------------------------------------------------

// *** DO NOT MOVE ***
Module module;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);
core::led::Publisher  led_publisher("led_publisher");
core::pixy_driver::PixyNode pixy("pixy", core::os::Thread::PriorityEnum::NORMAL);
core::triskar_kinematics::Forward forward("forward", core::os::Thread::PriorityEnum::NORMAL);
core::triskar_kinematics::Inverse inverse("inverse", core::os::Thread::PriorityEnum::NORMAL);
rosserial::RosSerialPublisher ros_node("ros", core::os::Thread::PriorityEnum::NORMAL);

/*
 * Application entry point.
 */
extern "C" {
   int
   main(
      void
   )
   {
      module.initialize();

      // Led publisher node
      core::led::PublisherConfiguration led_publisher_configuration;
      led_publisher_configuration.topic = "led";
      led_publisher_configuration.led   = 1;
      led_publisher.setConfiguration(led_publisher_configuration);
      module.add(led_publisher);

      // Led subscriber node
      core::led::SubscriberConfiguration led_subscriber_configuration;
      led_subscriber_configuration.topic = "led";
      led_subscriber.setConfiguration(led_subscriber_configuration);
      module.add(led_subscriber);

      //Pixy node
      core::pixy_driver::PixyNodeConfiguration pixy_conf;
      pixy_conf.topic = "pixy";
      pixy.setConfiguration(pixy_conf);

      // Add nodes to the node manager (== board)...
      module.add(led_subscriber);
      module.add(led_publisher);
      module.add(pixy);
      module.add(forward);
      module.add(inverse);
      module.add(ros_node);

      // ... and let's play!
      module.setup();
      module.run();

      // Is everything going well?
      for (;;) {
         if (!module.isOk()) {
            module.halt("This must not happen!");
         }

         core::os::Thread::sleep(core::os::Time::ms(500));
      }

      return core::os::Thread::OK;
   } // main
}
