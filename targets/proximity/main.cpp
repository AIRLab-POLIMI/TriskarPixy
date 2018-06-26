/* Proximity Module template file
 *
 */

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- BOARD IMPL -------------------------------------------------------------
#include <core/hw/GPIO.hpp>
#include <core/hw/EXT.hpp>

// --- MODULE -----------------------------------------------------------------
Module module;

// *** DO NOT MOVE THE CODE ABOVE THIS COMMENT *** //

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/led/Subscriber.hpp>
#include <core/sonar_publisher/SonarNode.hpp>

// --- TYPES ------------------------------------------------------------------

// --- CONFIGURATIONS ---------------------------------------------------------
core::led::SubscriberConfiguration led_subscriber_configuration_default;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_sub", core::os::Thread::PriorityEnum::LOWEST);
core::sonar_publisher::SonarNode sonar_publisher("sonar_publisher", core::os::Thread::PriorityEnum::NORMAL);

// --- DEVICE CONFIGURATION ---------------------------------------------------

// --- MAIN -------------------------------------------------------------------
extern "C" {
    int
    main()
    {
        module.initialize();

        // Device configurations

        // Default configuration
        led_subscriber_configuration_default.topic = "led";

        //Sonar node
        core::sonar_publisher::SonarNodeConfiguration sonar_conf_default;
        sonar_conf_default.topic = "proximity";
        sonar_conf_default.frequency = 20;
        sonar_conf_default.pulse_width = 20;

        // Add configurable objects to the configuration manager...
        module.configurations().add(led_subscriber, led_subscriber_configuration_default);
        module.configurations().add(sonar_publisher, sonar_conf_default);

        // ... and load the configuration
        module.configurations().loadFrom(module.configurationStorage());

        // Add nodes to the node manager...
        module.nodes().add(led_subscriber);
        module.nodes().add(sonar_publisher);

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
