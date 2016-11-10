#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// MESSAGES
#include <core/common_msgs/Led.hpp>

// NODES
#include <core/led/Subscriber.hpp>
//#include <core/ir_publisher/IRNode.hpp>
#include <core/sonar_publisher/SonarNode.hpp>

// BOARD IMPL

// *** DO NOT MOVE ***
Module module;

// TYPES

// NODES
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);
//core::ir_publisher::IRNode ir_publisher("ir_publisher", core::os::Thread::PriorityEnum::NORMAL);
core::sonar_publisher::SonarNode sonar_publisher("sonar_publisher", core::os::Thread::PriorityEnum::NORMAL);

// MAIN
extern "C" {
   int
   main()
   {
      module.initialize();

      // Led subscriber node
      core::led::SubscriberConfiguration led_subscriber_configuration;
      led_subscriber_configuration.topic = "led";
      led_subscriber.setConfiguration(led_subscriber_configuration);

      //IR node
      /*core::ir_publisher::IRNodeConfiguration ir_conf;
      ir_conf.topic = "proximity";
      ir_conf.frequency = 20;
      ir_conf.volt = {0.4, 0.45, 0.5, 0.6, 0.75, 0.7, 1.1, 1.3, 1.65, 2.3, 2.75, 3, 3.15};
      ir_conf.dist = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.25, 0.20, 0.15, 0.10, 0.08, 0.07, 0.06};
      ir_conf.values = 13;
      ir_publisher.setConfiguration(ir_conf);*/

      //Sonar node
      core::sonar_publisher::SonarNodeConfiguration sonar_conf;
      sonar_conf.topic = "proximity";
      sonar_conf.frequency = 20;
      sonar_publisher.setConfiguration(sonar_conf);


      //add nodes
      module.add(led_subscriber);
      module.add(sonar_publisher);


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
