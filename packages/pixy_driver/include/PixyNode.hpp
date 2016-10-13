#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Publisher.hpp>

#include <core/pixy_msgs/Pixy.hpp>
#include <core/pixy_driver/PixyNodeConfiguration.hpp>

namespace core
{
namespace pixy_driver {
class PixyNode: public core::mw::CoreNode, public core::mw::CoreConfigurable<core::pixy_driver::PixyNodeConfiguration>
{
public:
	PixyNode(const char* name, core::os::Thread::Priority priority =
			core::os::Thread::PriorityEnum::NORMAL);

	virtual
	~PixyNode();

private:
	bool
	onPrepareMW();

	bool
	onStart();

	bool
	onLoop();

private:
	uint16_t getWord();
	bool getStart();
	void setPixyMsg(pixy_msgs::Pixy* msgp,
				int checksum,int signature,
				int x,int y,
				int width,int height);


private:
	enum BlockType
	{
		NORMAL_BLOCK,
		CC_BLOCK // color code block
	};

private:
	core::mw::Publisher<pixy_msgs::Pixy> _pub;

	BlockType g_blockType; // use this to remember the next object block type between function calls
	int g_skipStart = 0;

};

}
}
