#include <core/ir_publisher/IRNode.hpp>

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


	palSetPadMode(GPIOA, GPIOA_PIN0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, GPIOA_PIN1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, GPIOA_PIN2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, GPIOA_PIN3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, GPIOA_PIN4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, GPIOA_PIN5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, GPIOA_PIN6, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, GPIOA_PIN7, PAL_MODE_INPUT_ANALOG);

	return true;
}

bool IRNode::onPrepareMW()
{

	advertise(_pub, configuration().topic);
	
	return true;
}

bool IRNode::onStart()
{
	palWritePad(GPIOA, GPIOA_PIN12, PAL_LOW);
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

	if (_pub.alloc(msgp)) {
			msgp->value[0] = adc_samples[7];
			msgp->value[1] = adc_samples[5];
			msgp->value[2] = adc_samples[3];
			msgp->value[3] = adc_samples[1];
			msgp->value[4] = adc_samples[0];
			msgp->value[5] = adc_samples[2];
			msgp->value[6] = adc_samples[6];
			msgp->value[7] = adc_samples[4];

			_pub.publish(msgp);
	}

	_stamp = core::os::Time::now();

	return true;
}

}

}

