#include <core/sonar_publisher/SonarNode.hpp>
#include <Module.hpp>
#include <core/hw/EXT.hpp>

namespace core
{

namespace sonar_publisher
{

time_measurement_t SonarNode::tm[8];

core::hw::Pad* SonarNode::channels[8] =
{
   &Module::d7,
   &Module::d8,
   &Module::d6,
   &Module::d5,
   &Module::d3,
   &Module::d4,
   &Module::d2,
   &Module::d1
};

SonarNode::SonarNode(const char* name, core::os::Thread::Priority priority) :
					CoreNode::CoreNode(name, priority),
					CoreConfigurable<core::sonar_publisher::SonarNodeConfiguration>::CoreConfigurable(name)
{
   _workingAreaSize = 768;
   _Ts = 0;
   low = true;
}

SonarNode::~SonarNode()
{

}

bool SonarNode::onConfigure()
{
	_Ts = core::os::Time::hz(configuration().frequency);


	Module::a1.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a2.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a3.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a4.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a5.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a6.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a7.setMode(core::hw::Pad::OUTPUT_PUSHPULL);
	Module::a8.setMode(core::hw::Pad::OUTPUT_PUSHPULL); //TODO PAL_STM32_OSPEED_HIGHEST ??

    core::hw::EXTChannel_<core::hw::EXT_1, 11, EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA> echo1;
    core::hw::EXTChannel_<core::hw::EXT_1, 10, EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA> echo2;
    core::hw::EXTChannel_<core::hw::EXT_1, 8, EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA> echo3;
    core::hw::EXTChannel_<core::hw::EXT_1, 9, EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA> echo4;
    core::hw::EXTChannel_<core::hw::EXT_1, 7, EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB> echo5;
    core::hw::EXTChannel_<core::hw::EXT_1, 6, EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB> echo6;
    core::hw::EXTChannel_<core::hw::EXT_1, 4, EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB> echo7;
    core::hw::EXTChannel_<core::hw::EXT_1, 5, EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB> echo8;

    echo1.setCallback(ext_cb);
    echo2.setCallback(ext_cb);
    echo3.setCallback(ext_cb);
    echo4.setCallback(ext_cb);
    echo5.setCallback(ext_cb);
    echo6.setCallback(ext_cb);
    echo7.setCallback(ext_cb);
    echo8.setCallback(ext_cb);

    echo1.enable();
    echo2.enable();
    echo3.enable();
    echo4.enable();
    echo5.enable();
    echo6.enable();
    echo7.enable();
    echo8.enable();

	return true;
}

bool SonarNode::onPrepareMW()
{

	advertise(_pub, configuration().topic);
	
	return true;
}

bool SonarNode::onStart()
{
	for (int i = 0; i < 8; i++)
	{
	   	chTMObjectInit(tm + i);
	}

	palWritePad(GPIOA, GPIOA_PIN12, PAL_LOW); //TODO aggiungere pad a modulo
	palWritePad(GPIOC, GPIOC_PIN15, PAL_LOW);

	_stamp = core::os::Time::now();

	return true;
}

bool SonarNode::onLoop()
{
	core::os::Thread::sleep_until(_stamp+_Ts);

	core::sensor_msgs::Proximity* msgp;

	if(low)
		startSonarLow();
	else
		startSonarHigh();

	low = !low;

	if (_pub.alloc(msgp))
	{
		msgp->value[0] = RTC2US(STM32_SYSCLK, tm[7].last) / 5.8f;
		msgp->value[1] = RTC2US(STM32_SYSCLK, tm[6].last) / 5.8f;
		msgp->value[2] = RTC2US(STM32_SYSCLK, tm[4].last) / 5.8f;
		msgp->value[3] = RTC2US(STM32_SYSCLK, tm[5].last) / 5.8f;
		msgp->value[4] = RTC2US(STM32_SYSCLK, tm[3].last) / 5.8f;
		msgp->value[5] = RTC2US(STM32_SYSCLK, tm[2].last) / 5.8f;
		msgp->value[6] = RTC2US(STM32_SYSCLK, tm[0].last) / 5.8f;
		msgp->value[7] = RTC2US(STM32_SYSCLK, tm[1].last) / 5.8f;

		_pub.publish(msgp);
	}

	_stamp = core::os::Time::now();

	return true;
}

void SonarNode::startSonarLow()
{
	//Start sonars (side 1)
	Module::a1.set();
	Module::a2.set();
	Module::a3.set();
	Module::a4.set();
	osalSysPolledDelayX(OSAL_US2ST(10));
	Module::a1.clear();
	Module::a2.clear();
	Module::a3.clear();
	Module::a4.clear();
}

void SonarNode::startSonarHigh()
{
	//Start sonars (side 2)
	Module::a5.set();
	Module::a6.set();
	Module::a7.set();
	Module::a8.set();
	osalSysPolledDelayX(OSAL_US2ST(10));
	Module::a5.clear();
	Module::a6.clear();
	Module::a7.clear();
	Module::a8.clear();
}



void SonarNode::ext_cb(expchannel_t channel)
{
	uint16_t index = channel - 4;

	if ((index < 0) || (index > 7))
		return;

	if (channels[index]->read())
	{
		chTMStartMeasurementX(tm + index);
	}
	else
	{
		chTMStopMeasurementX(tm + index);
	}
}

}

}

