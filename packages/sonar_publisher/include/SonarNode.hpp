#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Publisher.hpp>

#include <core/sonar_publisher/SonarNodeConfiguration.hpp>
#include <core/sensor_msgs/Proximity.hpp>

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
	static void ext_cb(EXTDriver *extp, expchannel_t channel);

private:
	static void start_measure(int id);
	static void stop_measure(int id);
	static void startSonarLow();
	static void startSonarHigh();

private:
	core::mw::Publisher<sensor_msgs::Proximity> _pub;

	core::os::Time _Ts;
	core::os::Time _stamp;

private:
	static uint32_t start[8];
	static uint32_t diff[8];
	bool low;

};

}
}
