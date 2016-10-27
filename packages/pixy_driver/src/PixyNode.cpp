#include <core/pixy_driver/PixyNode.hpp>

#include "hal.h"

#define PIXY_ARRAYSIZE              100
#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORD_CC          0xaa56
#define PIXY_START_WORDX            0x55aa
#define PIXY_SERVO_SYNC             0xff
#define PIXY_CAM_BRIGHTNESS_SYNC    0xfe
#define PIXY_LED_SYNC               0xfd


namespace core
{

namespace pixy_driver
{

PixyNode::PixyNode(const char* name, core::os::Thread::Priority priority):
								CoreNode::CoreNode(name, priority),
								CoreConfigurable<core::pixy_driver::PixyNodeConfiguration>::CoreConfigurable(name)
{
	_workingAreaSize = 1024;
	_blockType = NORMAL_BLOCK;
	_skipStart = 0;
}

PixyNode::~PixyNode()
{

}


bool PixyNode::ledCallback(const core::pixy_msgs::Led& msg,
					   void* node)
{
	PixyNode* _this = static_cast<PixyNode*>(node);
	_this->setLED(msg.color[0], msg.color[1], msg.color[2]);
	return true;
}

bool PixyNode::servoCallback(const core::pixy_msgs::Servo& msg,
						   void* node)
{
	PixyNode* _this = static_cast<PixyNode*>(node);
	_this->setServos(msg.pan, msg.tilt);
	return true;
}

bool PixyNode::brightnessCallback(const core::pixy_msgs::Brightness& msg,
						   void* node)
{
	PixyNode* _this = static_cast<PixyNode*>(node);
	_this->setBrightness(msg.value);
	return true;
}

bool PixyNode::onPrepareMW()
{
	advertise(_pub, configuration().topic);

	_ledSub.set_callback(ledCallback);
	subscribe(_ledSub, configuration().topicLed);

	_servoSub.set_callback(servoCallback);
	subscribe(_servoSub, configuration().topicServo);

	_brightnessSub.set_callback(brightnessCallback);
	subscribe(_brightnessSub, configuration().topicBrightness);

	return true;
}

bool PixyNode::onStart()
{
	sdStart(&SD3, NULL);

	return true;
}

bool PixyNode::onLoop()
{
	readFrames();

	spin(core::os::Time::ms(0));

	return true;
}

//get 32-bit word from SD3
uint16_t PixyNode::getWord()
{
	// this routine assumes little endian
	uint16_t w;
	uint8_t c;

	//get chars ( 16 bit)
	c = sdGetTimeout(&SD3, MS2ST(100));
	w = sdGetTimeout(&SD3, MS2ST(100));

	//convert chars and convert in 32 bit word
	w <<= 8;
	w |= c;
	return w;
}

//return 1 when pixy is in syncro
bool PixyNode::getStart()
{
	uint16_t w, lastw;

	lastw = 0xffff; // some inconsequential initial value

	while(1)
	{
		w = getWord();
		if (w==0 && lastw==0)
			return false; // in I2C and SPI modes this means no data, so return immediately
		else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
		{
			_blockType = NORMAL_BLOCK; // remember block type
			return true; // code found!
		}
		else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
		{
			_blockType = CC_BLOCK; // found color code block
			return true;
		}
		else if (w==PIXY_START_WORDX) // this is important, we might be juxtaposed
			sdGetTimeout(&SD3, MS2ST(100)); // we're out of sync! (backwards)
		lastw = w; // save
	}

}

//set a message of type PixyMsg
void PixyNode::publishPixyMsg(int checksum,int signature,
			int x,int y,
			int width,int height)
{
	core::pixy_msgs::Pixy* msgp;

	if (_pub.alloc(msgp))
	{
		msgp->checksum =checksum;
		msgp->signature=signature;
		msgp->x = x;
		msgp->y = y;
		msgp->width = width;
		msgp->height = height;

		_pub.publish(*msgp);
	}
}

int PixyNode::send(uint8_t *data, int len)
{
  int i;
  for (i=0; i<len; i++)
    sdPut(&SD3,data[i]);

  return len;
}

int PixyNode::setServos(uint16_t s0, uint16_t s1)
{
  uint8_t outBuf[6];

  outBuf[0] = 0x00;
  outBuf[1] = PIXY_SERVO_SYNC;
  *(uint16_t *)(outBuf + 2) = s0;
  *(uint16_t *)(outBuf + 4) = s1;

  return send(outBuf, 6);
}

int PixyNode::setBrightness(uint8_t brightness)
{
  uint8_t outBuf[3];

  outBuf[0] = 0x00;
  outBuf[1] = PIXY_CAM_BRIGHTNESS_SYNC;
  outBuf[2] = brightness;

  return send(outBuf, 3);
}

int PixyNode::setLED(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t outBuf[5];

  outBuf[0] = 0x00;
  outBuf[1] = PIXY_LED_SYNC;
  outBuf[2] = r;
  outBuf[3] = g;
  outBuf[4] = b;

  return send(outBuf, 5);
}

void PixyNode::readFrames()
{
	bool curr = getStart();// if is 1 is in syncro
	if (curr) // two start codes means start of new frame
	{
		bool found=false;
		int checksum = getWord();
		if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame
		{
			_skipStart = 1;
			_blockType = NORMAL_BLOCK;
			found=true;
		}
		else if (checksum==PIXY_START_WORD_CC)
		{
			_skipStart = 1;
			_blockType = CC_BLOCK;
			found=true;
		}
		else if (checksum==0)
			found=true;

		if(!found)
		{
			//block of valid data
			uint16_t tempValues[6], sum=0;
			tempValues[0]=checksum;
			for(int i=1;i<6;i++)
			{
				tempValues[i]=getWord();
				sum = sum + tempValues[i];
			}
			if(checksum==sum)
			{
				//order : checksum,signature,x,y,height,width
				publishPixyMsg(tempValues[0],tempValues[1],tempValues[2],
						tempValues[3],tempValues[4],tempValues[5]);
			}
		}
	}
}




}


}

