/* USB Module template file
 *
 */

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- BOARD IMPL -------------------------------------------------------------

// --- MODULE -----------------------------------------------------------------
Module module;

// *** DO NOT MOVE THE CODE ABOVE THIS COMMENT *** //

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/led/Publisher.hpp>
#include <core/led/Subscriber.hpp>
#include <core/pixy_driver/PixyNode.hpp>
#include <core/triskar_kinematics/Forward.hpp>
#include <core/triskar_kinematics/Inverse.hpp>

#include "rosserial.hpp"

// --- TYPES ------------------------------------------------------------------

// --- CONFIGURATIONS ---------------------------------------------------------
core::led::PublisherConfiguration  led_publisher_configuration_default;
core::led::SubscriberConfiguration led_subscriber_configuration_default;

// --- NODES ------------------------------------------------------------------
core::led::Publisher  led_publisher("led_pub", core::os::Thread::PriorityEnum::LOWEST);
core::led::Subscriber led_subscriber("led_sub", core::os::Thread::PriorityEnum::LOWEST);

core::pixy_driver::PixyNode pixy("pixy", core::os::Thread::PriorityEnum::NORMAL);
core::triskar_kinematics::Forward forward("forward", core::os::Thread::PriorityEnum::NORMAL);
core::triskar_kinematics::Inverse inverse("inverse", core::os::Thread::PriorityEnum::NORMAL);
rosserial::RosSerialPublisher ros_node("ros", core::os::Thread::PriorityEnum::NORMAL);

// --- DEVICE CONFIGURATION ---------------------------------------------------

// --- MAIN -------------------------------------------------------------------
extern "C" {
    int
    main(
        void
    )
    {
  	    const float wheelRadius = 0.035f;
  	  	const float centerDistance = 0.160f;

        module.initialize();

        // Device configurations

        // Default configuration
        led_publisher_configuration_default.topic  = "led";
        led_publisher_configuration_default.led    = 1;
        led_subscriber_configuration_default.topic = "led";

        //Pixy node
        core::pixy_driver::PixyNodeConfiguration pixy_conf_default;
        pixy_conf_default.topic = "pixy";
        pixy_conf_default.topicLed = "pixy_led";
        pixy_conf_default.topicServo = "pixy_servo";
        pixy_conf_default.topicBrightness = "pixy_brightness";

        //Forward kinematics configuration
        core::triskar_kinematics::ForwardConfiguration forward_conf_default;
        forward_conf_default.input_0 = "encoder_0";
        forward_conf_default.input_1 = "encoder_1";
        forward_conf_default.input_2 = "encoder_2";
        forward_conf_default.output = "vel";
        forward_conf_default.wheel_radius = wheelRadius;
        forward_conf_default.center_distance = centerDistance;

        //Inverse kinematics configuration
        core::triskar_kinematics::InverseConfiguration inverse_conf_default;
        inverse_conf_default.output_0 = "speed_0";
        inverse_conf_default.output_1 = "speed_1";
        inverse_conf_default.output_2 = "speed_2";
        inverse_conf_default.velocity_input = "cmd_vel";
        inverse_conf_default.wheel_radius = wheelRadius;
        inverse_conf_default.center_distance = centerDistance;

        // Add configurable objects to the configuration manager...
        module.configurations().add(led_publisher, led_publisher_configuration_default);
        module.configurations().add(led_subscriber, led_subscriber_configuration_default);
        module.configurations().add(pixy, pixy_conf_default);
        module.configurations().add(forward, forward_conf_default);
        module.configurations().add(inverse, inverse_conf_default);

        // ... and load the configuration
        module.configurations().loadFrom(module.configurationStorage());

        // Add nodes to the node manager...
        module.nodes().add(led_publisher);
        module.nodes().add(led_subscriber);
        module.nodes().add(pixy);
        module.nodes().add(forward);
        module.nodes().add(inverse);
        module.nodes().add(ros_node);

        // ... and let's play!
        module.nodes().setup();
        module.nodes().run();

        // Is everything going well?
        for (;;) {
            if (!module.nodes().areOk()) {
                module.halt("This must not happen!");
            }

            core::os::Thread::sleep(core::os::Time::ms(500));

            // Remember to feed the (watch)dog!
            module.keepAlive();
        }

        return core::os::Thread::OK;
    } // main
}
