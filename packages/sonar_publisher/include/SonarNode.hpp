#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Publisher.hpp>

#include <core/sonar_publisher/SonarNodeConfiguration.hpp>
#include <core/sensor_msgs/Proximity.hpp>

#include <core/hw/GPIO.hpp>

#include "hal.h"


namespace core
{
namespace sonar_publisher {
class SonarNode: public core::mw::CoreNode,
				   public core::mw::CoreConfigurable<core::sonar_publisher::SonarNodeConfiguration>::CoreConfigurable
{
public:
	SonarNode(const char* name, core::os::Thread::Priority priority =
			core::os::Thread::PriorityEnum::NORMAL);

	virtual
	~SonarNode();

private:
	bool
	onConfigure();

	bool
	onPrepareMW();

	bool
	onStart();

	bool
	onLoop();


public:
	static void ext_cb(expchannel_t channel);

private:
	static void startSonarLow();
	static void startSonarHigh();

private:
	core::mw::Publisher<sensor_msgs::Proximity> _pub;

	core::os::Time _Ts;
	core::os::Time _stamp;

private:
	static time_measurement_t tm[8];
	static core::hw::Pad* channels[8];
	bool low;

};

}
}
