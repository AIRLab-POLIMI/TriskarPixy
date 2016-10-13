#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// MESSAGES
#include <core/common_msgs/Led.hpp>

// NODES
#include <core/led/Subscriber.hpp>
#include <core/ir_publisher/IRNode.hpp>

// BOARD IMPL

// *** DO NOT MOVE ***
Module module;

// TYPES

// NODES
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);
core::ir_publisher::IRNode ir_publisher("ir_publisher", core::os::Thread::PriorityEnum::NORMAL);

// MAIN
extern "C" {
   int
   main()
   {
      module.initialize();

      module.add(led_subscriber);
      module.add(ir_publisher);

      // Led subscriber node
      core::led::SubscriberConfiguration led_subscriber_configuration;
      led_subscriber_configuration.topic = "led";
      led_subscriber.setConfiguration(led_subscriber_configuration);

      //IR node
      core::ir_publisher::IRNodeConfiguration ir_conf;
      ir_conf.topic = "proximity";
      ir_publisher.setConfiguration(ir_conf);


      // Setup and run
      module.setup();
      module.run();

      // Is everything going well?
      for (;;) {
         if (!module.isOk()) {
            module.halt("This must not happen!");
         }

         module.keepAlive();

         core::os::Thread::sleep(core::os::Time::ms(500));
      }

      return core::os::Thread::OK;
   } // main
}
