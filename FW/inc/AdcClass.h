/*
 * AdcClass.h
 *
 *  Created on: 02.05.2012
 *      Author: Stefan
 */

#ifndef ADCCLASS_H_
#define ADCCLASS_H_
#include "stm32f10x_conf.h"
#include "stm32f10x.h"

#define ADC_PIN  		GPIO_Pin_0
#define ADC_GPIO     	GPIOA
#define ADC_GPIO_CLK 	RCC_APB2Periph_GPIOA

#define ADC_AVERAGE_COUNT   16  // Number of times to measure

class AdcClass {
private:
	uint16_t ADCValues[ADC_AVERAGE_COUNT];
public:
    void Init(void);
    uint32_t GetValue(void);
};

//extern AdcClass Adc;

#endif /* ADCCLASS_H_ */
