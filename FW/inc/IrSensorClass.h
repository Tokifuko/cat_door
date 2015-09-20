/*
 * IrSensorClass.h
 *
 *  Created on: 20 сент. 2015 г.
 *      Author: MOON
 */

#ifndef IRSENSORCLASS_H_
#define IRSENSORCLASS_H_

#include "stm32f10x_conf.h"


class IrSensorClass {
private:
    bool bActiveFlag;
    GPIO_TypeDef *pPort;
    uint16_t pPin;
    uint32_t dwSwitchTimer;
    uint32_t dwOnDelay=0;
    uint32_t dwOffDelay=0;
    bool bOldSensorState=true;
public:
    void Init(GPIO_TypeDef *PGpioPort, uint16_t APinMask);
    void SetOnDelay(uint32_t dwTime) {dwOnDelay=dwTime;}
    void SetOffDelay(uint32_t dwTime) {dwOffDelay=dwTime;}
    void Task (void);
    bool IsActive (void){return bActiveFlag;}
};
#endif /* IRSENSORCLASS_H_ */
