#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Publisher.hpp>

#include <core/pixy_msgs/Pixy.hpp>
#include <core/pixy_msgs/Servo.hpp>
#include <core/pixy_msgs/Led.hpp>
#include <core/pixy_msgs/Brightness.hpp>
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
	static bool
	ledCallback(const core::pixy_msgs::Led& msg,
						   void* node);

	static bool
	servoCallback(const core::pixy_msgs::Servo& msg,
							   void* node);

	static bool
	brightnessCallback(const core::pixy_msgs::Brightness& msg,
							   void* node);
private:
	uint16_t getWord();
	bool getStart();
	void publishPixyMsg(int checksum,int signature,
				int x,int y,
				int width,int height);

	int send(uint8_t *data, int len);
	int setServos(uint16_t s0, uint16_t s1);
	int setBrightness(uint8_t brightness);
	int setLED(uint8_t r, uint8_t g, uint8_t b);

	void readFrames();


private:
	enum BlockType
	{
		NORMAL_BLOCK,
		CC_BLOCK // color code block
	};

private:
	core::mw::Publisher<pixy_msgs::Pixy> _pub;

	core::mw::Subscriber<pixy_msgs::Led, 5> _ledSub;
	core::mw::Subscriber<pixy_msgs::Servo, 5> _servoSub;
	core::mw::Subscriber<pixy_msgs::Brightness, 5> _brightnessSub;


	BlockType _blockType; // use this to remember the next object block type between function calls
	int _skipStart;

};

}
}
