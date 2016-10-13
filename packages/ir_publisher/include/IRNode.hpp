#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Publisher.hpp>

#include <core/ir_publisher/IRNodeConfiguration.hpp>
#include <core/sensor_msgs/Proximity.hpp>

namespace core
{
namespace ir_publisher {
class IRNode: public core::mw::CoreNode,
				   public core::mw::CoreConfigurable<core::ir_publisher::IRNodeConfiguration>::CoreConfigurable
{
public:
	IRNode(const char* name, core::os::Thread::Priority priority =
			core::os::Thread::PriorityEnum::NORMAL);

	virtual
	~IRNode();

private:
	bool
	onConfigure();

	bool
	onPrepareMW();

	bool
	onStart();

	bool
	onLoop();

private:
	core::mw::Publisher<sensor_msgs::Proximity> _pub;

	core::os::Time _Ts;
	core::os::Time _stamp;

};

}
}
