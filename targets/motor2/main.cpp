//----------------//
// UDC_2 module   //
//----------------//

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/sensor_publisher/Publisher.hpp>
#include <core/actuator_subscriber/Subscriber.hpp>
#include <core/led/Subscriber.hpp>
#include <core/pid/PIDNode.hpp>

// --- BOARD IMPL -------------------------------------------------------------
#include <core/QEI_driver/QEI.hpp>
#include <core/MC33926_driver/MC33926.hpp>

// *** DO NOT MOVE ***
Module module;

// --- TYPES ------------------------------------------------------------------
using QEI_Publisher  = core::sensor_publisher::Publisher<ModuleConfiguration::QEI_DELTA_DATATYPE>;
using PWM_Subscriber = core::actuator_subscriber::Subscriber<float, core::actuator_msgs::Setpoint_f32>;
using PIDNode = pid::PIDNode;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);

QEI_Publisher  encoder("encoder", module.qei, core::os::Thread::PriorityEnum::NORMAL);
PWM_Subscriber motor("actuator_sub", module.pwm, core::os::Thread::PriorityEnum::NORMAL);
PIDNode pid_node("pid", core::os::Thread::PriorityEnum::NORMAL);

// --- MAIN -------------------------------------------------------------------
extern "C" {
   int
   main()
   {
	  const float period = 50.0;

      module.initialize();

      // Add nodes to the node manager (== board)...
      module.add(led_subscriber);
      module.add(encoder);
      module.add(motor);
      module.add(pid_node);

      // Module configuration
      core::QEI_driver::QEI_DeltaConfiguration qei_configuration;
      qei_configuration.period = period;
      qei_configuration.ticks  = 1000;
      module.qei.setConfiguration(qei_configuration);

      // Nodes configuration

      //Led
      core::led::SubscriberConfiguration led_subscriber_configuration;
      led_subscriber_configuration.topic = "led";
      led_subscriber.setConfiguration(led_subscriber_configuration);

      //Encoder
      core::sensor_publisher::Configuration encoder_configuration;
      encoder_configuration.topic = "encoder_2";
      encoder.setConfiguration(encoder_configuration);

      //Motor
      core::actuator_subscriber::Configuration pwm_conf;
      pwm_conf.topic = "pwm_2";
      motor.setConfiguration(pwm_conf);

      //Pid
      pid_node.configuration.kp = 100;
      pid_node.configuration.ti = 0;
      pid_node.configuration.td = 0;
      pid_node.configuration.ts = 1.0/period;
      pid_node.configuration.measure_topic = "encoder_2";
      pid_node.configuration.output_topic = "pwm_2";
      pid_node.configuration.setpoint_topic = "speed_2";
      pid_node.configuration.idle = 0;
      pid_node.configuration.timeout = 500;

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
