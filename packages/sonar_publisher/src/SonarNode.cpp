#include <core/sonar_publisher/SonarNode.hpp>
#include <Module.hpp>
#include <core/hw/GPIO.hpp> //TODO move in module?

#include "hal.h"

namespace core
{

namespace sonar_publisher
{

uint32_t SonarNode::start[8] = {0};
uint32_t SonarNode::diff[8] = {0};

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB | EXT_CH_MODE_AUTOSTART, SonarNode::ext_cb},
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB | EXT_CH_MODE_AUTOSTART, SonarNode::ext_cb},
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB | EXT_CH_MODE_AUTOSTART, SonarNode::ext_cb},
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB | EXT_CH_MODE_AUTOSTART, SonarNode::ext_cb},
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA | EXT_CH_MODE_AUTOSTART, SonarNode::ext_cb},
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA | EXT_CH_MODE_AUTOSTART, SonarNode::ext_cb},
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA | EXT_CH_MODE_AUTOSTART, SonarNode::ext_cb},
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA | EXT_CH_MODE_AUTOSTART, SonarNode::ext_cb},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};


SonarNode::SonarNode(const char* name, core::os::Thread::Priority priority) :
					CoreNode::CoreNode(name, priority),
					CoreConfigurable<core::sonar_publisher::SonarNodeConfiguration>::CoreConfigurable(name)
{
   _workingAreaSize = 768;
   _Ts = 0;
}

SonarNode::~SonarNode()
{

}

bool SonarNode::onConfigure()
{
	_Ts = core::os::Time::hz(configuration().frequency);
	_stamp = core::os::Time::now();


	Module::a1.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a2.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a3.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a4.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a5.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a6.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a7.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a8.setMode(core::hw::Pad::OUTPUT_PUSHPULL); // PAL_STM32_OSPEED_HIGHEST ??

	return true;
}

bool SonarNode::onPrepareMW()
{

	advertise(_pub, configuration().topic);
	
	return true;
}

bool SonarNode::onStart()
{
	palWritePad(GPIOA, GPIOA_PIN12, PAL_LOW); //TODO aggiungere pad a modulo
	palWritePad(GPIOC, GPIOC_PIN15, PAL_LOW);

	extStart(&EXTD1, &extcfg);

	_stamp = core::os::Time::now();

	return true;
}

bool SonarNode::onLoop()
{
	core::os::Thread::sleep_until(_stamp+_Ts);

	core::sensor_msgs::Proximity* msgp;

	//Start sonars (side 1)
	Module::a1.set();
	Module::a2.set();
	Module::a3.set();
	Module::a4.set();
	halPolledDelay(US2RTT(10));
	Module::a1.clear();
	Module::a2.clear();
	Module::a3.clear();
	Module::a4.clear();

	//Start sonars (side 2)
	Module::a5.set();
	Module::a6.set();
	Module::a7.set();
	Module::a8.set();
	halPolledDelay(US2RTT(10));
	Module::a5.clear();
	Module::a6.clear();
	Module::a7.clear();
	Module::a8.clear();


	if (_pub.alloc(msgp))
	{
		msgp->value[0] = diff[0] / 5.8;
		msgp->value[1] = diff[1] / 5.8;
		msgp->value[2] = diff[2] / 5.8;
		msgp->value[3] = diff[3] / 5.8;
		msgp->value[4] = diff[4] / 5.8;
		msgp->value[5] = diff[5] / 5.8;
		msgp->value[6] = diff[6] / 5.8;
		msgp->value[7] = diff[7] / 5.8;

		_pub.publish(msgp);
	}

	_stamp = core::os::Time::now();

	return true;
}


void SonarNode::ext_cb(EXTDriver *extp, expchannel_t channel) {

	(void)extp;

	switch (channel)
	{
	case D1_PIN:
		if (Module::d1.read())
		{
			start_measure(1);

		}
		else
		{
			stop_measure(1);
		}
		break;

	case D2_PIN:
		if (Module::d2.read())
		{
			start_measure(2);
		}
		else
		{
			stop_measure(2);
		}
		break;

	case D3_PIN:
		if (Module::d3.read())
		{
			start_measure(3);
		}
		else
		{
			stop_measure(3);
		}
		break;

	case D4_PIN:
		if (Module::d4.read())
		{
			start_measure(4);
		}
		else
		{
			stop_measure(4);
		}
		break;

	case D5_PIN:
		if (Module::d5.read())
		{
			start_measure(5);
		}
		else
		{
			stop_measure(5);
		}
		break;

	case D6_PIN:
		if (Module::d6.read())
		{
			start_measure(6);
		}
		else
		{
			stop_measure(6);
		}
		break;

	case D7_PIN:
		if (Module::d7.read())
		{
			start_measure(7);
		}
		else
		{
			stop_measure(7);
		}
		break;

	case D8_PIN:
		if (Module::d8.read())
		{
			start_measure(8);
		}
		else
		{
			stop_measure(8);
		}
		break;

	default:
		break;
	}
}

void SonarNode::start_measure(int id)
{
	  start[id - 1] = halGetCounterValue();
}

void SonarNode::stop_measure(int id)
{
	  uint32_t stop = halGetCounterValue();
	  diff[id - 1] = RTT2US(stop - start[id - 1]);
}

}

}

