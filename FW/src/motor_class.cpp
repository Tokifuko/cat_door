#include "stm32f10x.h"
#include "motor_class.h"
#include "delay_util.h"
#include "GPIO_config.h"

void MotorClass::Init(void)
{
    RCC_APB2PeriphClockCmd(
    	MOTOR_A1_GPIO_CLK |
    	MOTOR_A2_GPIO_CLK |
    	MOTOR_B1_GPIO_CLK |
    	MOTOR_B2_GPIO_CLK |
        RCC_APB2Periph_AFIO,
        ENABLE );

    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure MOTOR_A1_PIN as Output */
    GPIO_InitStructure.GPIO_Pin =  MOTOR_A1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( MOTOR_A1_GPIO, &GPIO_InitStructure );
    GPIO_ResetBits(MOTOR_A1_GPIO,MOTOR_A1_PIN);  //N-канал выключается GND

    /* Configure MOTOR_A2_PIN as Output */
    GPIO_InitStructure.GPIO_Pin =  MOTOR_A2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( MOTOR_A2_GPIO, &GPIO_InitStructure );
    GPIO_ResetBits(MOTOR_A2_GPIO,MOTOR_A2_PIN); //N-канал выключается GND

    /* Configure MOTOR_B1_PIN as Output */
    GPIO_InitStructure.GPIO_Pin =  MOTOR_B1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( MOTOR_B1_GPIO, &GPIO_InitStructure );
    GPIO_SetBits(MOTOR_B1_GPIO,MOTOR_B1_PIN);		//P-канал выключается VCC

    /* Configure MOTOR_B2_PIN as Output */
    GPIO_InitStructure.GPIO_Pin =  MOTOR_B2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( MOTOR_B2_GPIO, &GPIO_InitStructure );
    GPIO_SetBits(MOTOR_B2_GPIO,MOTOR_B2_PIN);		//P-канал выключается VCC

    chState=MOTOR_STOP;

    CurrentAdc.Init();
}

void MotorClass::Task(void)
{
	bOverloadFlag=0;
	if (iOverloadDelay==OVERLOAD_DELAY)
	{
		iCurrentConsumption=CurrentAdc.GetValue();
		switch (chState)
		{
		case MOTOR_UPWARD:
			if (iCurrentConsumption>OVERLOAD_LEVEL_UP) bOverloadFlag=1;
			break;
		case MOTOR_DOWNWARD:
			if (iCurrentConsumption>OVERLOAD_LEVEL_DOWN) bOverloadFlag=1;
			break;
		}
	}
	else iOverloadDelay++;

	if (bOverloadFlag) Led4On();
	else Led4Off();
	return;
}

void MotorClass::Stop(void)
{
	GPIO_ResetBits(MOTOR_A1_GPIO,MOTOR_A1_PIN);
	GPIO_ResetBits(MOTOR_A2_GPIO,MOTOR_A2_PIN);
	GPIO_SetBits(MOTOR_B1_GPIO,MOTOR_B1_PIN);
	GPIO_SetBits(MOTOR_B2_GPIO,MOTOR_B2_PIN);
	iOverloadDelay=0;
	Led2Off();
	Led3Off();
	chState=MOTOR_STOP;
}
void MotorClass::Upward(void) // A1, B2 - вверх
{
	// отключаем A2 и B1
	GPIO_ResetBits(MOTOR_A2_GPIO,MOTOR_A2_PIN);
	GPIO_SetBits(MOTOR_B1_GPIO,MOTOR_B1_PIN);
	Led2Off();
	// ждем закрытия транзисторов
	Delay.ms(SWITCH_DELAY);
	// включаем A1 и B2
	GPIO_SetBits(MOTOR_A1_GPIO,MOTOR_A1_PIN);
	GPIO_ResetBits(MOTOR_B2_GPIO,MOTOR_B2_PIN);
	iOverloadDelay=0;
	chState=MOTOR_UPWARD;
	Led3On();
}

void MotorClass::Downward(void)	// A2, B1 - вниз
{
	// отключаем A1 и B2
	Led3Off();
	GPIO_ResetBits(MOTOR_A1_GPIO,MOTOR_A1_PIN);
	GPIO_SetBits(MOTOR_B2_GPIO,MOTOR_B2_PIN);
	// ждем закрытия транзисторов
	Delay.ms(SWITCH_DELAY);
	// включаем A2 и B1
	GPIO_SetBits(MOTOR_A2_GPIO,MOTOR_A2_PIN);
	GPIO_ResetBits(MOTOR_B1_GPIO,MOTOR_B1_PIN);
	iOverloadDelay=0;
	chState=MOTOR_DOWNWARD;
	Led2On();
}
