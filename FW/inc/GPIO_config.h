#ifndef SIMPLE_LED_H
#define SIMPLE_LED_H
#include "stm32f10x_conf.h"
/* светодиоды, понятно */
#define LED_1_PORT             GPIOB
#define LED_1_CLK              RCC_APB2Periph_GPIOB
#define LED_1_PIN              GPIO_Pin_7
void Led1On	(void);
void Led1Off (void);


#define LED_2_PORT             GPIOB
#define LED_2_CLK              RCC_APB2Periph_GPIOB
#define LED_2_PIN              GPIO_Pin_6
void Led2On	(void);
void Led2Off (void);


#define LED_3_PORT             GPIOB
#define LED_3_CLK              RCC_APB2Periph_GPIOB
#define LED_3_PIN              GPIO_Pin_5
void Led3On	(void);
void Led3Off (void);

#define LED_4_PORT             GPIOB
#define LED_4_CLK              RCC_APB2Periph_GPIOB
#define LED_4_PIN              GPIO_Pin_4
void Led4On	(void);
void Led4Off (void);

/* кнопки ручного управления */
#define KEY_DOWN_PORT          GPIOA
#define KEY_DOWN_CLK           RCC_APB2Periph_GPIOA
#define KEY_DOWN_PIN           GPIO_Pin_15
bool KeyDownPressed(void);

#define KEY_UP_PORT          	GPIOB
#define KEY_UP_CLK           	RCC_APB2Periph_GPIOB
#define KEY_UP_PIN           	GPIO_Pin_3
bool KeyUpPressed(void);

/*концевые выключатели */
#define STOP_UP_1_PORT          	GPIOA
#define STOP_UP_1_CLK           	RCC_APB2Periph_GPIOA
#define STOP_UP_1_PIN           	GPIO_Pin_10
bool StopUp1Pressed(void);

#define STOP_UP_2_PORT          	GPIOA
#define STOP_UP_2_CLK           	RCC_APB2Periph_GPIOA
#define STOP_UP_2_PIN           	GPIO_Pin_9
bool StopUp2Pressed(void);

#define STOP_DOWN_1_PORT          	GPIOA
#define STOP_DOWN_1_CLK           	RCC_APB2Periph_GPIOA
#define STOP_DOWN_1_PIN           	GPIO_Pin_5
bool StopDown1Pressed(void);

#define STOP_DOWN_2_PORT          	GPIOA
#define STOP_DOWN_2_CLK           	RCC_APB2Periph_GPIOA
#define STOP_DOWN_2_PIN           	GPIO_Pin_1
bool StopDown2Pressed(void);

/* датчик повреждения кошки */
#define WARNING_DOWN_PORT          	GPIOB
#define WARNING_DOWN_CLK           	RCC_APB2Periph_GPIOB
#define WARNING_DOWN_PIN           	GPIO_Pin_0
bool WarningDownPressed(void);

#define WARNING_UP_PORT          	GPIOB
#define WARNING_UP_CLK           	RCC_APB2Periph_GPIOB
#define WARNING_UP_PIN           	GPIO_Pin_2
bool WarningUpPressed(void);

/* датчики движения */
#define SENSOR_1_PORT          	GPIOA
#define SENSOR_1_CLK           	RCC_APB2Periph_GPIOA
#define SENSOR_1_PIN           	GPIO_Pin_11
bool Sensor1Pressed(void);

#define SENSOR_2_PORT          	GPIOA
#define SENSOR_2_CLK           	RCC_APB2Periph_GPIOA
#define SENSOR_2_PIN           	GPIO_Pin_12
bool Sensor2Pressed(void);

/* управление питанием */
#define POWER_ON_PIN  		GPIO_Pin_8
#define POWER_ON_GPIO     	GPIOB
#define POWER_ON_GPIO_CLK 	RCC_APB2Periph_GPIOB
void PowAtxOn	(void);
void PowAtxOff   (void);

#define POWER_OK_PIN  		GPIO_Pin_9
#define POWER_OK_GPIO     	GPIOB
#define POWER_OK_GPIO_CLK 	RCC_APB2Periph_GPIOB
bool PowAtxIsOk(void);

#endif /* SIMPLE_LED_H */
