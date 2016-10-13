#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Publisher.hpp>

namespace core
{
namespace ir_publisher {
class PixyNode: public core::mw::CoreNode
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
