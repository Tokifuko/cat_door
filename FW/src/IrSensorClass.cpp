/*
 * IrSensorClass.cpp
 *
 *  Created on: 20 сент. 2015 г.
 *      Author: MOON
 */
#include "IrSensorClass.h"
#include "delay_util.h"
#include "kl_lib.h"

void IrSensorClass::Init(GPIO_TypeDef *PGpioPort, uint16_t APinMask)
{
    pPort=PGpioPort;
    pPin=APinMask;
    klGpioSetupByMsk(pPort,pPin,GPIO_Mode_IN_FLOATING);
}

void IrSensorClass::Task(void)
{
    // изменилось состояние сенсора
    if (klGpioIsSetByMsk(pPort,pPin)!=bOldSensorState)
    {
        bOldSensorState=klGpioIsSetByMsk(pPort,pPin);
        Delay.Reset(&dwSwitchTimer);
    }

    // появился сигнал от сенсора
    if (bOldSensorState)
    {
        if (Delay.Elapsed(&dwSwitchTimer,dwOnDelay)) bActiveFlag=false;
    }

    // пропал сигнал от сенсора
    if (!(bOldSensorState))
    {
        if (Delay.Elapsed(&dwSwitchTimer,dwOffDelay)) bActiveFlag=true;
    }
}
