/*
 * File:   main.cpp
 * Author: Kreyl
 *
 * Created on May 27, 2011, 6:37 PM
 */

#include "stm32f10x.h"

#include "delay_util.h"
#include "UARTClass.h"
#include "GPIO_config.h"
#include "motor_class.h"
#include <string.h>
#include <stdio.h>

#define DEBUG_LED_TIMEOUT	100
#define OPEN_DELAY			1500
#define SENSOR2_IGNOR_DELAY 1300
// состояния автомата ручного управления
#define MC_IDLE			0
#define MC_MOVE_DOWN	1
#define MC_MOVE_UP		2

// состояния автомата автоматического управления
#define AC_INIT						0
#define AC_OPENING					1
#define AC_OPENING_TEMP_DOWN		2
#define AC_OPEN						3
#define AC_CLOSING					4
#define AC_CLOSING_TEMP_UP			5
#define AC_CLOSE_INDOOR_SENS_OFF	6
#define AC_CLOSE_INDOOR_SENS_ON		7


#define ATX_ON_TIMEOUT				100000 // время ожидания включения блока питания ATX

UART_Class* pUART1;
UART_Class* pUART2;
UART_Class* pUART3;
UART_Class* pUART4;
UART_Class* pUART5;


void GeneralInit(void);
void MainControl(void);
void AutoControl (void);
void SensorDebug(void);
uint32_t PowerSupplyOn(void);
uint32_t PowerSupplyOff(void);
UART_Class DbgUART;
uint8_t chManualControlState;
uint8_t chAutoControlState;
uint8_t chSensor1Debug;
uint8_t chSensor2Debug;
MotorClass Motor;
uint32_t iOpenTimer;
uint32_t iSensor2IgnorTimer;

int main(void) {
	uint32_t AdcValue;
	uint32_t iDebugLedTimer;

	Motor.Init();
    DbgUART.UART_Init(USART2);
    Delay.ms(100);
    uint32_t flag=0;
    GeneralInit();



    //sprintf(str,"Hello word %в  \n",56);
   // DbgUART.FIFO_TxData.WriteData(24,(uint8_t*) str);
    DbgUART.SendPrintF("Hello word %d \n",24);
    //DbgUART.SendPrintF("00000000001111111111222222222233333333334444444444555555555566666666667777777777");
    //DbgUART.SendPrintF("8888888888");
    //DbgUART.SendPrintF("00000000001111111111222222222233333333334444444444555555555566666666667777777777");
    Delay.Reset(&iDebugLedTimer);
    Motor.Stop();
   // Motor.Downward();
    // ==== Main cycle ====
    chManualControlState=MC_IDLE;
    PowAtxOff();
    while(1)
    {
    	SensorDebug();
    	MainControl();
    	Motor.Task();
    	if (Motor.GetOverloadFlag()==1)
    	{
    		AdcValue=Motor.GetCurrentConsumption();
    		DbgUART.SendPrintF("ADC value= %d \n",AdcValue);
    	}
    	if (Delay.Elapsed(&iDebugLedTimer,DEBUG_LED_TIMEOUT))
    	{
    		if (flag==0)
    			{
    				if (!(PowAtxIsOk())) Led1On(); // зажикая светодиод тушим подсветку кнопки
    				flag=1;
    			}
    			else
    				{
    					Led1Off();
    					flag=0;
    				}
    	}
        //i2cMgr.Task();
       // ESnd.Task();
        //ESns.Task();
       // EUSB.Task();
        //Leds.Task();
       // ELock.Task();
       // EKeyboard.Task();
        //EIRSirc.Task();
        //Lcd.Task();
    } // while(1)
    return 0;
}

void SensorDebug()
{
	if (Sensor1Pressed())
	{
		if (chSensor1Debug==0)
		{
			chSensor1Debug=1;
			DbgUART.SendPrintF("Sens 1 - activated \n");
		}
	}
	else
	{
		if (chSensor1Debug==1)
		{
			chSensor1Debug=0;
			DbgUART.SendPrintF("Sens 1 - deactivated \n");
		}
	}
	if (Sensor2Pressed())
	{
		if (chSensor2Debug==1) return;
		chSensor2Debug=1;
		DbgUART.SendPrintF("Sens 2 - activated \n");
	}
	else
	{
		if (chSensor2Debug==0) return;
		chSensor2Debug=0;
		DbgUART.SendPrintF("Sens 2 - deactivated \n");
	}

}
void MainControl ()
{
	switch (chManualControlState)
	{
	case MC_IDLE: // ожидаем нажатия кнопки
		AutoControl();
		if (KeyDownPressed())
			{
				DbgUART.SendPrintF("key_Down PRESSED \n");
				if (PowerSupplyOn()==1)	chManualControlState=MC_MOVE_DOWN;
			}
		if (KeyUpPressed())
			{
				DbgUART.SendPrintF("key_UP  PRESSED\n");
				if (PowerSupplyOn()==1)	chManualControlState=MC_MOVE_UP;
			}
		break;
	case MC_MOVE_DOWN: // дверь опускается
		if (!KeyDownPressed()) // кнопка отжата
		{
			Motor.Stop();
			DbgUART.SendPrintF("key_Down RELISE \n");
			if (PowerSupplyOff()==1)	chManualControlState=MC_IDLE;
			chAutoControlState=AC_OPEN;  // запоминаем положение двери
		}
		if (StopDown1Pressed()) // сработал концевой выключатель
		{
			Motor.Stop();
			DbgUART.SendPrintF("STOP_Down 1  \n");
			chManualControlState=MC_IDLE;
			chAutoControlState=AC_CLOSE_INDOOR_SENS_OFF;  // запоминаем положение двери
		}
		if (StopDown2Pressed()) // сработал концевой выключатель
		{
			Motor.Stop();
			DbgUART.SendPrintF("STOP_Down 2  \n");
			chManualControlState=MC_IDLE;
			chAutoControlState=AC_CLOSE_INDOOR_SENS_OFF;  // запоминаем положение двери
		}
		if (chManualControlState==MC_MOVE_DOWN) Motor.Downward();
		break;
	case MC_MOVE_UP: // дверь поднимается
		if (!KeyUpPressed()) // кнопка отжата
		{
			Motor.Stop();
			DbgUART.SendPrintF("key_Up RELISE \n");
			if (PowerSupplyOff()==1)	chManualControlState=MC_IDLE;
			chAutoControlState=AC_OPEN;  // запоминаем положение двери
		}
		if (StopUp1Pressed()) // сработал концевой выключатель
		{
			Motor.Stop();
			DbgUART.SendPrintF("STOP_Up 1  \n");
			chManualControlState=MC_IDLE;
			chAutoControlState=AC_OPEN;  // запоминаем положение двери
		}
		if (StopUp2Pressed()) // сработал концевой выключатель
		{
			Motor.Stop();
			DbgUART.SendPrintF("STOP_Up 2  \n");
			chManualControlState=MC_IDLE;
			chAutoControlState=AC_OPEN;  // запоминаем положение двери
		}
		if (chManualControlState==MC_MOVE_UP) Motor.Upward();
		break;
	}

}

void AutoControl (void)
{
	switch (chAutoControlState)
	{
	case AC_INIT:
		DbgUART.SendPrintF("Auto Control Init  \n");
		if (PowerSupplyOn()==1)
		{
			chAutoControlState=AC_OPENING;
			Motor.Upward();
		}
		break;
	case AC_OPENING:
		if (WarningUpPressed())
		{
			Motor.Downward();
			chAutoControlState=AC_OPENING_TEMP_DOWN;
			DbgUART.SendPrintF("War Up go to temp_down  \n");
		}
		if (Motor.GetOverloadFlag())
		{
			Motor.Downward();
			chAutoControlState=AC_OPENING_TEMP_DOWN;
			DbgUART.SendPrintF("Motor overload, go temp down  \n");
		}
		if (StopUp1Pressed() | StopUp2Pressed())
		{
			Motor.Stop();
			DbgUART.SendPrintF("Stop Up. new state - OPEN  \n");
			chAutoControlState=AC_OPEN;
			Delay.Reset(&iOpenTimer);
		}
		break;
	case AC_OPENING_TEMP_DOWN:
		if (WarningDownPressed())
		{
			Motor.Upward();
			DbgUART.SendPrintF("War down. go to Movw UP  \n");
			chAutoControlState=AC_OPENING;
		}
		if (Motor.GetOverloadFlag())
		{
			Motor.Upward();
			DbgUART.SendPrintF("Motor overload, go Movw UP  \n");
			chAutoControlState=AC_OPENING;
		}
		if (StopDown1Pressed() | StopDown2Pressed())
		{
			Motor.Upward();
			DbgUART.SendPrintF("stop down, temp down complite, move UP  \n");
			chAutoControlState=AC_OPENING;
		}
		break;
	case AC_OPEN: // дверь открыта
		if ((Sensor1Pressed()) |(Sensor2Pressed()))Delay.Reset(&iOpenTimer); // если сенсор сработал, значит кошка в проходе. сбрасываем таймер
		if (Delay.Elapsed(&iOpenTimer,OPEN_DELAY))
		{
			if (PowerSupplyOn()==1)
			{
				Motor.Downward();
				chAutoControlState=AC_CLOSING;
				DbgUART.SendPrintF("Open Timeout Elapsed, go to CLOSING \n");
			}
		}
		break;
	case AC_CLOSING: // начинаем закрываться
		if (WarningDownPressed())
		{
			Motor.Upward();
			chAutoControlState=AC_CLOSING_TEMP_UP;
			DbgUART.SendPrintF("War DOWN go to temp_up  \n");
		}
		if (Motor.GetOverloadFlag())
		{
			Motor.Upward();
			chAutoControlState=AC_CLOSING_TEMP_UP;
			DbgUART.SendPrintF("Motor overload, go temp_up  \n");
		}
		if (StopDown1Pressed() | StopDown2Pressed())
		{
			Motor.Stop();
			DbgUART.SendPrintF("Stop Down. new state - CLOSE  \n");
			chAutoControlState=AC_CLOSE_INDOOR_SENS_OFF;
			Delay.Reset(&iSensor2IgnorTimer);
			DbgUART.SendPrintF("Close state. indoor sens OFF  \n");
		}
		if ((Sensor1Pressed()) /*| (Sensor2Pressed())*/)
		{
			chAutoControlState=AC_OPENING;
			Motor.Upward();
			DbgUART.SendPrintF("Sensor act, open up  \n");
		}
		break;
	case AC_CLOSING_TEMP_UP:
		if (WarningUpPressed())
		{
			Motor.Downward();
			DbgUART.SendPrintF("War UP. go to Move Down  \n");
			chAutoControlState=AC_CLOSING;
		}
		if (Motor.GetOverloadFlag())
		{
			Motor.Downward();
			DbgUART.SendPrintF("Motor overload, go to Move Down  \n");
			chAutoControlState=AC_CLOSING;
		}
		if (StopUp1Pressed() | StopUp2Pressed())
		{
			Motor.Downward();
			DbgUART.SendPrintF("stop Up, temp Up complite, move down  \n");
			chAutoControlState=AC_CLOSING;
		}
		if ((Sensor1Pressed()) /*| (Sensor2Pressed())*/)
		{
			chAutoControlState=AC_OPENING;
			Motor.Upward();
			DbgUART.SendPrintF("Sensor act, open up  \n");
		}
		break;
	case AC_CLOSE_INDOOR_SENS_OFF: // дверь закрыта игнорируем датчик
		if (PowAtxIsOk())PowerSupplyOff(); // выключаем, если включен
		if (Delay.Elapsed(&iSensor2IgnorTimer,SENSOR2_IGNOR_DELAY))
		{
			chAutoControlState=AC_CLOSE_INDOOR_SENS_ON;
			DbgUART.SendPrintF("Close state. indoor sens ON  \n");
		}
		if (Sensor1Pressed())
		{
			if (PowerSupplyOn())
			{
				chAutoControlState=AC_OPENING;
				Motor.Upward();
				DbgUART.SendPrintF("Sensor act, open up  \n");
			}

		}
		break;
	case AC_CLOSE_INDOOR_SENS_ON: // дверь закрыта и внутриенний датчик активен
		if (PowAtxIsOk())PowerSupplyOff(); // выключаем, если включен
		if ((Sensor1Pressed()) | (Sensor2Pressed()))
		{
			if (PowerSupplyOn())
			{
				chAutoControlState=AC_OPENING;
				Motor.Upward();
				DbgUART.SendPrintF("Sensor act, open up  \n");
			}

		}
		break;
	}
}

void GeneralInit(void) {
    // Disable JTAG
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    // Configure two bits for preemption priority
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

    // Init peripheral
    //i2cMgr.Init();
    Delay.Init();

    RCC_APB2PeriphClockCmd(
    		LED_1_CLK |
    		LED_2_CLK |
    		LED_3_CLK |
    		LED_4_CLK |
    		KEY_DOWN_CLK |
    		KEY_UP_CLK |
    		STOP_UP_1_CLK |
    		STOP_UP_2_CLK |
    		STOP_DOWN_1_CLK |
    		STOP_DOWN_2_CLK |
    		WARNING_DOWN_CLK |
    		WARNING_UP_CLK |
    		SENSOR_1_CLK |
    		SENSOR_2_CLK|
    		POWER_ON_GPIO_CLK|
    		POWER_OK_GPIO_CLK|
        RCC_APB2Periph_AFIO,
        ENABLE );

    GPIO_InitTypeDef GPIO_InitStructure;

    // светодиоды
    /* Configure LED_1_PIN as Output */
    GPIO_InitStructure.GPIO_Pin =  LED_1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( LED_1_PORT, &GPIO_InitStructure );
    /* Configure LED_2_PIN as Output */
    GPIO_InitStructure.GPIO_Pin =  LED_2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( LED_2_PORT, &GPIO_InitStructure );
    /* Configure LED_3_PIN as Output */
    GPIO_InitStructure.GPIO_Pin =  LED_3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( LED_3_PORT, &GPIO_InitStructure );
    /* Configure LED_4_PIN as Output */
    GPIO_InitStructure.GPIO_Pin =  LED_4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( LED_4_PORT, &GPIO_InitStructure );

    // кнопки ручного управления
    /* Configure KEY_DOWN_PIN as INPUT PULLUP */
    GPIO_InitStructure.GPIO_Pin =  KEY_DOWN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( KEY_DOWN_PORT, &GPIO_InitStructure );
    /* Configure KEY_UP_PIN as INPUT PULLUP */
    GPIO_InitStructure.GPIO_Pin =  KEY_UP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( KEY_UP_PORT, &GPIO_InitStructure );

    // концевые выключатели
    /* Configure STOP_UP_1_PIN as INPUT PULLUP */
    GPIO_InitStructure.GPIO_Pin =  STOP_UP_1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( STOP_UP_1_PORT, &GPIO_InitStructure );
    /* Configure STOP_UP_2_PIN as INPUT PULLUP */
    GPIO_InitStructure.GPIO_Pin =  STOP_UP_2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( STOP_UP_2_PORT, &GPIO_InitStructure );
    /* Configure STOP_DOWN_1_PIN as INPUT PULLUP */
    GPIO_InitStructure.GPIO_Pin =  STOP_DOWN_1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( STOP_DOWN_1_PORT, &GPIO_InitStructure );
    /* Configure STOP_DOWN_2_PIN as INPUT PULLUP */
    GPIO_InitStructure.GPIO_Pin =  STOP_DOWN_2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( STOP_DOWN_2_PORT, &GPIO_InitStructure );

    // Датчики нажатия на кошку
    /* Configure WARNING_DOWN_PIN as INPUT PULLUP */
    GPIO_InitStructure.GPIO_Pin =  WARNING_DOWN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( WARNING_DOWN_PORT, &GPIO_InitStructure );
    /* Configure WARNING_UP_PIN as INPUT PULLUP */
    GPIO_InitStructure.GPIO_Pin =  WARNING_UP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( WARNING_UP_PORT, &GPIO_InitStructure );

    // Датчики движения
    /* Configure SENSOR_1_PIN as INPUT_FLOATING */
    GPIO_InitStructure.GPIO_Pin =  SENSOR_1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( SENSOR_1_PORT, &GPIO_InitStructure );
    /* Configure SENSOR_2_PIN as INPUT_FLOATING */
    GPIO_InitStructure.GPIO_Pin =  SENSOR_2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( SENSOR_2_PORT, &GPIO_InitStructure );

    // Управление питанием
    /* Configure POWER_ON_PIN as Output */
    GPIO_InitStructure.GPIO_Pin =  POWER_ON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( POWER_ON_GPIO, &GPIO_InitStructure );
    /* Configure POWER_OK_PIN as INPUT PULLUP */
    GPIO_InitStructure.GPIO_Pin =  POWER_OK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( POWER_OK_GPIO, &GPIO_InitStructure );

    //ROW_1_ON();
    // Leds
    //Leds.Init();
    // Sensors
    //ESns.Init();
  //  EIRSirc.Init();

    // Sound
    //SD.Init();
    //Vs.Init();
    //ESnd.Init();
    //ELock.Init();
    //EKeyboard.Init();
    //EUSB.Init();


    // LCD
    //Lcd.Init();
    //Lcd.DrawImage(0,0, ImageLogo, NotInverted);    // Artifact
//    ERock.Init();

    // Read initial charge
//    ERock.H.Counter = ReadInt32("Init", "HCounter", "rock.ini");
//    ERock.H.Value   = ReadInt32("Init", "HValue",   "rock.ini");
//    ERock.E.Counter = ReadInt32("Init", "ECounter", "rock.ini");
//    ERock.E.Value   = ReadInt32("Init", "EValue",   "rock.ini");
//    ERock.C.Counter = ReadInt32("Init", "CCounter", "rock.ini");
//    ERock.C.Value   = ReadInt32("Init", "CValue",   "rock.ini");
//    ERock.ChooseType();

    //ESnd.Play("magnet.wav");
}

uint32_t PowerSupplyOn(void)
{
	uint32_t iTimeToWateAtxSupply;
	PowAtxOn();
	iTimeToWateAtxSupply=0;
	while ((!(PowAtxIsOk())) & (iTimeToWateAtxSupply!=ATX_ON_TIMEOUT))
	{
		iTimeToWateAtxSupply++;
	}
	if (PowAtxIsOk())
	{
		DbgUART.SendPrintF("ATX power supply ON SUCSESS  \n");
		return 1;
	}
	else
	{
		DbgUART.SendPrintF("ATX power supply ON ERROR  \n");
		return 0;
	}
}

uint32_t PowerSupplyOff(void)
{
	uint32_t iTimeToWateAtxSupply;
	PowAtxOff();
	iTimeToWateAtxSupply=0;
	while ((PowAtxIsOk()) & (iTimeToWateAtxSupply!=ATX_ON_TIMEOUT))
	{
		iTimeToWateAtxSupply++;
	}
	if (!(PowAtxIsOk()))
	{
		DbgUART.SendPrintF("ATX power supply OFF SUCSESS  \n");
		return 1;
	}
	else
	{
		DbgUART.SendPrintF("ATX power supply OFF ERROR  \n");
		return 0;
	}
}
