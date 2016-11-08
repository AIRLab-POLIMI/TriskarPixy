#include <core/ir_publisher/IRNode.hpp>
#include <Module.hpp>
#include <core/hw/GPIO.hpp> //TODO move in module?

#include "hal.h"


/*===========================================================================*/
/* ADC related.                                                              */
/*===========================================================================*/

#define ADC_NUM_CHANNELS 8
#define ADC_BUF_DEPTH 2

static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];
static const ADCConversionGroup adc_group_config = {
  FALSE,
  ADC_NUM_CHANNELS,
  NULL,
  NULL,
  ADC_CFGR_CONT,                                                /* CFGR     */
  ADC_TR(0, 4095),                                              /* TR1      */
  ADC_CCR_DUAL(1),                                              /* CCR      */
  {                                                             /* SMPR[2]  */
	    ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_601P5) | ADC_SMPR1_SMP_AN2(ADC_SMPR_SMP_601P5),
	    ADC_SMPR1_SMP_AN3(ADC_SMPR_SMP_601P5) | ADC_SMPR1_SMP_AN4(ADC_SMPR_SMP_601P5),
  },
  {                                                             /* SQR[4]   */
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN2) |
    ADC_SQR1_SQ3_N(ADC_CHANNEL_IN3) | ADC_SQR1_SQ4_N(ADC_CHANNEL_IN4),
    0,
    0
  },
  {                                                             /* SSMPR[2] */
	    ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_601P5) | ADC_SMPR1_SMP_AN2(ADC_SMPR_SMP_601P5),
	    ADC_SMPR1_SMP_AN3(ADC_SMPR_SMP_601P5) | ADC_SMPR1_SMP_AN4(ADC_SMPR_SMP_601P5),
  },
  {                                                             /* SSQR[4]  */
	ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN2) |
	ADC_SQR1_SQ3_N(ADC_CHANNEL_IN3) | ADC_SQR1_SQ4_N(ADC_CHANNEL_IN4),
    0,
    0
  }
};

namespace core
{

namespace ir_publisher
{


IRNode::IRNode(const char* name, core::os::Thread::Priority priority) :
					CoreNode::CoreNode(name, priority),
					CoreConfigurable<core::ir_publisher::IRNodeConfiguration>::CoreConfigurable(name)
{
   _workingAreaSize = 768;
   _Ts = 0;
}

IRNode::~IRNode()
{

}

bool IRNode::onConfigure()
{
	_Ts = core::os::Time::hz(configuration().frequency);
	_stamp = core::os::Time::now();

	Module::a1.setMode(core::hw::Pad::INPUT_ANALOG);
	Module::a2.setMode(core::hw::Pad::INPUT_ANALOG);
	Module::a3.setMode(core::hw::Pad::INPUT_ANALOG);
	Module::a4.setMode(core::hw::Pad::INPUT_ANALOG);
	Module::a5.setMode(core::hw::Pad::INPUT_ANALOG);
	Module::a6.setMode(core::hw::Pad::INPUT_ANALOG);
	Module::a7.setMode(core::hw::Pad::INPUT_ANALOG);
	Module::a8.setMode(core::hw::Pad::INPUT_ANALOG);

	return true;
}

bool IRNode::onPrepareMW()
{

	advertise(_pub, configuration().topic);
	
	return true;
}

bool IRNode::onStart()
{
	palWritePad(GPIOA, GPIOA_PIN12, PAL_LOW); //TODO aggiungere pad a modulo
	palWritePad(GPIOC, GPIOC_PIN15, PAL_LOW);

	adcStart(&ADCD1, NULL);

	_stamp = core::os::Time::now();

	return true;
}

bool IRNode::onLoop()
{
	core::os::Thread::sleep_until(_stamp+_Ts);

	core::sensor_msgs::Proximity* msgp;

	adcConvert(&ADCD1, &adc_group_config, adc_samples, ADC_BUF_DEPTH);

	const float adcPrecision = 3.3/4095.0;

	if (_pub.alloc(msgp)) {
			msgp->value[0] = 1000.0*voltToDist(adc_samples[7]*adcPrecision);
			msgp->value[1] = 1000.0*voltToDist(adc_samples[5]*adcPrecision);
			msgp->value[2] = 1000.0*voltToDist(adc_samples[3]*adcPrecision);
			msgp->value[3] = 1000.0*voltToDist(adc_samples[1]*adcPrecision);
			msgp->value[4] = 1000.0*voltToDist(adc_samples[0]*adcPrecision);
			msgp->value[5] = 1000.0*voltToDist(adc_samples[2]*adcPrecision);
			msgp->value[6] = 1000.0*voltToDist(adc_samples[6]*adcPrecision);
			msgp->value[7] = 1000.0*voltToDist(adc_samples[4]*adcPrecision);

			_pub.publish(msgp);
	}

	_stamp = core::os::Time::now();

	return true;
}

float IRNode::voltToDist(volatile float val)
{
	auto volt = configuration().volt;
	auto dist = configuration().dist;
	volatile unsigned int values = configuration().values;
	volatile unsigned int size = values > dist.size() ? dist.size() : values;

	// take care the value is within range
	// val = constrain(val, _in[0], _in[size-1]);
	if (val <= volt[0]) return dist[0];
	if (val >= volt[size-1]) return dist[size-1];

	// search right interval
	volatile unsigned int pos = 1;  // _in[0] already tested
	while(val > volt[pos]) pos++;

	// this will handle all exact "points" in the _in array
	if (val == volt[pos]) return dist[pos];

	// interpolate in the right segment for the rest
	return linearize(val, volt[pos-1], volt[pos], dist[pos-1], dist[pos]);
}

}

}

