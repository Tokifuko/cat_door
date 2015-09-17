#include "GPIO_config.h"

void Led1On	(void)			{GPIO_ResetBits(LED_1_PORT,LED_1_PIN);}
void Led1Off (void)			{GPIO_SetBits(LED_1_PORT,LED_1_PIN);}
void Led2On	(void)			{GPIO_ResetBits(LED_2_PORT,LED_2_PIN);}
void Led2Off (void)			{GPIO_SetBits(LED_2_PORT,LED_2_PIN);}
void Led3On	(void)			{GPIO_ResetBits(LED_3_PORT,LED_3_PIN);}
void Led3Off (void)			{GPIO_SetBits(LED_3_PORT,LED_3_PIN);}
void Led4On	(void)			{GPIO_ResetBits(LED_4_PORT,LED_4_PIN);}
void Led4Off (void)			{GPIO_SetBits(LED_4_PORT,LED_4_PIN);}

bool KeyDownPressed(void) { return (GPIO_ReadInputDataBit(KEY_DOWN_PORT, KEY_DOWN_PIN) == Bit_RESET); }
bool KeyUpPressed(void) { return (GPIO_ReadInputDataBit(KEY_UP_PORT, KEY_UP_PIN) == Bit_RESET); }

bool StopUp1Pressed(void) { return (GPIO_ReadInputDataBit(STOP_UP_1_PORT, STOP_UP_1_PIN) == Bit_RESET); }
bool StopUp2Pressed(void) { return (GPIO_ReadInputDataBit(STOP_UP_2_PORT, STOP_UP_2_PIN) == Bit_RESET); }
bool StopDown1Pressed(void) { return (GPIO_ReadInputDataBit(STOP_DOWN_1_PORT, STOP_DOWN_1_PIN) == Bit_RESET); }
bool StopDown2Pressed(void) { return (GPIO_ReadInputDataBit(STOP_DOWN_2_PORT, STOP_DOWN_2_PIN) == Bit_RESET); }
bool WarningDownPressed(void) { return (GPIO_ReadInputDataBit(WARNING_DOWN_PORT, WARNING_DOWN_PIN) == Bit_RESET); }
bool WarningUpPressed(void) { return (GPIO_ReadInputDataBit(WARNING_UP_PORT, WARNING_UP_PIN) == Bit_RESET); }

bool Sensor1Pressed(void) { return (GPIO_ReadInputDataBit(SENSOR_1_PORT, SENSOR_1_PIN) == Bit_SET); }
bool Sensor2Pressed(void) { return (GPIO_ReadInputDataBit(SENSOR_2_PORT, SENSOR_2_PIN) == Bit_SET); }


void PowAtxOn	(void)			{GPIO_ResetBits(POWER_ON_GPIO,POWER_ON_PIN);}
void PowAtxOff   (void)			{GPIO_SetBits(POWER_ON_GPIO,POWER_ON_PIN);}
bool PowAtxIsOk(void)			{ return (GPIO_ReadInputDataBit(POWER_OK_GPIO, POWER_OK_PIN) == Bit_SET); }
