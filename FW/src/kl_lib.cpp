/*
 * kl_gpio.cpp
 *
 *  Created on: 18.02.2012
 *      Author: kreyl
 */

#include "kl_lib.h"
#include "misc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_flash.h"
#include <stdarg.h>




// ======== GPIO =========
void klGpioSetupByMsk (GPIO_TypeDef *PGpioPort, uint16_t APinMask, GPIOMode_TypeDef AMode) {
    // Clock
    int IClock = 0;
    if      (PGpioPort == GPIOA) IClock = RCC_APB2Periph_GPIOA;
    else if (PGpioPort == GPIOB) IClock = RCC_APB2Periph_GPIOB;
    else if (PGpioPort == GPIOC) IClock = RCC_APB2Periph_GPIOC;
    else if (PGpioPort == GPIOD) IClock = RCC_APB2Periph_GPIOD;
    if ((AMode == GPIO_Mode_AF_OD) || (AMode == GPIO_Mode_AF_PP)) IClock |= RCC_APB2Periph_AFIO;
    RCC_APB2PeriphClockCmd(IClock, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = APinMask;
    GPIO_InitStructure.GPIO_Mode = AMode;
    GPIO_Init(PGpioPort, &GPIO_InitStructure);
}


void klPin_t::Init(GPIO_TypeDef *PGpioPort, uint16_t APinNumber, GPIOMode_TypeDef AMode) {
    IPinNumber = APinNumber;
    IPinMask = (uint16_t)(1<<APinNumber);
    IGPIO = PGpioPort;
    klGpioSetupByMsk(PGpioPort, IPinMask, AMode);
}

// ======== IRQ pin =========
void klPinIrq_t::IrqSetup(EXTITrigger_TypeDef ATriggerType) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // Get IRQ channel
    if      ((IPinNumber >= 0)  and (IPinNumber <= 4))  IChannel = EXTI0_IRQn + IPinNumber;
    else if ((IPinNumber >= 5)  and (IPinNumber <= 9))  IChannel = EXTI9_5_IRQn;
    else if ((IPinNumber >= 10) and (IPinNumber <= 15)) IChannel = EXTI15_10_IRQn;
    // EXTI line config
    if      (IGPIO == GPIOA) GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, IPinNumber);
    else if (IGPIO == GPIOB) GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, IPinNumber);
    else if (IGPIO == GPIOC) GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, IPinNumber);
    else if (IGPIO == GPIOD) GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, IPinNumber);
    // Configure EXTI line
    EXTI_InitTypeDef   EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = IPinMask;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = ATriggerType;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void klPinIrq_t::IrqEnable() {
    EXTI_ClearITPendingBit(IPinMask);
    // Enable and set EXTI Interrupt
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = IChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void klPinIrq_t::IrqDisable() {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = IChannel;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
    EXTI_ClearITPendingBit(IPinMask);
}

// ================================= Timer =====================================
void klTimer_t::Init(TIM_TypeDef* PTimer, uint16_t ATopValue, uint32_t AFreqHz) {
    ITimer = PTimer;
    // Clock
    if      (ITimer == TIM1) RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    else if (ITimer == TIM2) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    else if (ITimer == TIM3) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    else if (ITimer == TIM4) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    // Timebase
    uint16_t IPrescaler = 0;
    if (AFreqHz < TIM_FREQ_MAX) {
        IPrescaler = SystemCoreClock / (ATopValue * AFreqHz);
        if (IPrescaler != 0) IPrescaler--;   // do not decrease in case of high freq
    }
    ITimer->ARR = ATopValue;
    ITimer->PSC = IPrescaler;
    ITimer->CR1 = 0;
}
void klTimer_t::SetFreqHz(uint32_t AFreqHz) {
    uint16_t IPrescaler = 0, FTopValue = ITimer->ARR;
    if (AFreqHz < TIM_FREQ_MAX) {
        IPrescaler = SystemCoreClock / (FTopValue * AFreqHz);
        if (IPrescaler != 0) IPrescaler--;   // do not decrease in case of high freq
    }
    ITimer->ARR = FTopValue;
}

void klPwmChannel_t::Init(TIM_TypeDef* PTimer, uint16_t ATopValue, uint32_t AFreqHz, uint8_t ANumber, uint16_t APolarity) {
    klTimer_t::Init(PTimer, ATopValue, AFreqHz);
    INumber = ANumber;

    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = APolarity;
    switch (ANumber) {
        case 1:
            TIM_OC1Init(PTimer, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(PTimer, TIM_OCPreload_Enable);
            break;
        case 2:
            TIM_OC2Init(PTimer, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(PTimer, TIM_OCPreload_Enable);
            break;
        case 3:
            TIM_OC3Init(PTimer, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(PTimer, TIM_OCPreload_Enable);
            break;
        case 4:
            TIM_OC4Init(PTimer, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(PTimer, TIM_OCPreload_Enable);
            break;
        default: break;
    }
    TIM_ARRPreloadConfig(PTimer, ENABLE);   // Enable autoreload of preload
    klTimer_t::Enable();
}

void klPwmChannel_t::Enable() {
    switch (INumber) {
        case 1: ITimer->CCER |= TIM_CCER_CC1E; break;
        case 2: ITimer->CCER |= TIM_CCER_CC2E; break;
        case 3: ITimer->CCER |= TIM_CCER_CC3E; break;
        case 4: ITimer->CCER |= TIM_CCER_CC4E; break;
        default: break;
    }
}

void klPwmChannel_t::Disable() {
    switch (INumber) {
        case 1: ITimer->CCER &= ~TIM_CCER_CC1E; break;
        case 2: ITimer->CCER &= ~TIM_CCER_CC2E; break;
        case 3: ITimer->CCER &= ~TIM_CCER_CC3E; break;
        case 4: ITimer->CCER &= ~TIM_CCER_CC4E; break;
        default: break;
    }
}

void klPwmChannel_t::Set(uint16_t AValue) {
    switch (INumber) {
        case 1: ITimer->CCR1 = AValue; break;
        case 2: ITimer->CCR2 = AValue; break;
        case 3: ITimer->CCR3 = AValue; break;
        case 4: ITimer->CCR4 = AValue; break;
        default: break;
    }
}

// ============================== System clock =================================
void InitClock(Clk_t AClk) {
    RCC_DeInit();                                           // RCC system reset(for debug purpose)
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);   // Enable Prefetch Buffer
    RCC_PCLK2Config(RCC_HCLK_Div1);                         // PCLK2 = HCLK   (to APB2, 72MHz max)
    // External clock
    if ((AClk == clk8MHzExternal)
#if defined STM32F10X_HD
    		or (AClk == clk36MHzExternal) or (AClk == clk72MHzExternal)
#endif
    ) {
        RCC_HCLKConfig(RCC_SYSCLK_Div1);                            // HCLK = SYSCLK
        RCC_HSEConfig(RCC_HSE_ON);                                  // Enable HSE
        ErrorStatus HSEStartUpStatus = RCC_WaitForHSEStartUp();     // Wait till HSE is ready
        if (HSEStartUpStatus == SUCCESS) {
            if (AClk == clk8MHzExternal) {
                FLASH_SetLatency(FLASH_Latency_0);                      // Flash 1 wait state
                RCC_PCLK1Config(RCC_HCLK_Div1);                         // PCLK1 = HCLK (to APB1, 36MHz max)
                RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
                while (RCC_GetSYSCLKSource() != 0x04);
                SystemCoreClock = 8000000;
            }
            else {
                switch (AClk) {
#if defined STM32F10X_HD
                    case clk36MHzExternal:
                        FLASH_SetLatency(FLASH_Latency_1);                      // Flash 1 wait state
                        RCC_PCLK1Config(RCC_HCLK_Div1);                         // PCLK1 = HCLK (to APB1, 36MHz max)
                        RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);    // PLLCLK = 8MHz / 2 * 9 = 36 MHz
                        SystemCoreClock = 36000000;
                        break;

                    case clk72MHzExternal:
                        FLASH_SetLatency(FLASH_Latency_2);                      // Flash 2 wait state
                        RCC_PCLK1Config(RCC_HCLK_Div2);                         // PCLK1 = HCLK/2 (to APB1, 36MHz max)
                        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);    // PLLCLK = 8MHz * 9 = 72 MHz
                        SystemCoreClock = 72000000;
                        break;
#endif
                    default: break;
                } // switch
                RCC_PLLCmd(ENABLE);                                     // Enable PLL
                while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);    // Wait till PLL is ready
                RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);              // Select PLL as system clock source
                while(RCC_GetSYSCLKSource() != 0x08);                   // Wait till PLL is used as system clock source
            }
        }
        else {    // If HSE fails to start-up, the application will have wrong clock configuration.
            while (1);
        }
    } // if external
    else {  // Internal clock: 8 and lower
        switch (AClk) {
            case clk8MHzInternal:
                RCC_HCLKConfig(RCC_SYSCLK_Div1);        // HCLK = SYSCLK
                SystemCoreClock = 8000000;
                break;
            case clk4MHzInternal:
                RCC_HCLKConfig(RCC_SYSCLK_Div2);        // HCLK = SYSCLK / 2
                SystemCoreClock = 4000000;
                break;
            case clk2MHzInternal:
                RCC_HCLKConfig(RCC_SYSCLK_Div4);        // HCLK = SYSCLK / 4
                SystemCoreClock = 2000000;
                break;
            case clk1MHzInternal:
                RCC_HCLKConfig(RCC_SYSCLK_Div8);        // HCLK = SYSCLK / 8
                SystemCoreClock = 1000000;
                break;

            default: break;
        }
        FLASH_SetLatency(FLASH_Latency_0);      // Flash 0 wait state
        RCC_PCLK1Config(RCC_HCLK_Div1);         // PCLK1 = HCLK (to APB1, 36MHz max)
        RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
        while (RCC_GetSYSCLKSource() != 0x00);
    }
}

// ============================ Unique ID ======================================
#define IDBASE      0x1FFFF7E8  // Memory address of ID
void GetUniqueID(uint32_t *IDArr) {
    uint32_t tmp1, tmp2;
    tmp1 = *((uint16_t*)(IDBASE));      // Lower 16 bits
    tmp2 = *((uint16_t*)(IDBASE+2));    // Next 16 bits
    tmp2 <<= 16;
    IDArr[0] = tmp2 + tmp1;
    IDArr[1] = *((uint32_t*)(IDBASE+4));    // Next 32 bits
    IDArr[2] = *((uint32_t*)(IDBASE+8));    // Last 32 bits
    // DEBUG
//    IDArr[0] = 0x01020304;
//    IDArr[1] = 0x05060708;
//    IDArr[2] = 0x0A0B0C0D;
}

// ============================== Delay ========================================
//Delay_t Delay;
//uint32_t ITickCounter;
/* Configure SysTick timer: Reload Value = SysTick Counter Clock (Hz) x Desired Time base (s)
 * Use HCLK_Div8 as systick clk source. Desired timebase is 1 ms.
 * Rise IRQ priority.
 */
/*
void Delay_t::Init(void) {
    SysTick_Config((uint32_t)(SystemCoreClock / (1000 * 8)));   // 1/1000 s and clk = sysclk/8
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

bool Delay_t::Elapsed(uint32_t *AVar, const uint32_t ADelay) {
    if ((uint32_t)(ITickCounter - (*AVar)) >= ADelay) {
        *AVar = ITickCounter; // Reset delay
        return true;
    }
    else return false;
}

void Delay_t::ms(uint32_t Ams) {
    uint32_t __ticks = (SystemCoreClock / 10000) * Ams;
    Loop(__ticks);
}

// IRQ
void SysTick_Handler(void) {
    ITickCounter++;
}
*/
// ============================== UART command =================================
CmdUnit_t Uart;

void CmdUnit_t::IPrintUint(uint32_t ANumber, uint8_t ACharCount) {
    uint8_t digit = '0';
    bool ShouldPrint = false;
    const uint32_t m[9] = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10};

    for(uint8_t i=0; i<9; i++) {
        while (ANumber >= m[i]) {
            digit++;
            ANumber -= m[i];
        }
        if ((digit != '0') or ShouldPrint or (9-i < ACharCount)) {
            IBufWrite(digit);
            ShouldPrint = true;
        }
        digit = '0';
    } // for
    IBufWrite((uint8_t)('0'+ANumber));
}
void CmdUnit_t::IPrintInt (int32_t ANumber, uint8_t ACharCount) {
    if (ANumber < 0) {
        IBufWrite('-');
        ANumber = -ANumber;
    }
    IPrintUint(ANumber, ACharCount);
}

void CmdUnit_t::IPrintAsHex (uint32_t ANumber, uint8_t ACharCount) {
    bool MustPrint = false;
    if ((ANumber & 0xFF000000) or (ACharCount > 6)) {
        IBufWrite(UintToHexChar (0x0F & (ANumber >> 28)));
        IBufWrite(UintToHexChar (0x0F & (ANumber >> 24)));
        MustPrint = true;
    }
    if ((ANumber & 0x00FF0000) or MustPrint or (ACharCount > 4)) {
        IBufWrite(UintToHexChar (0x0F & (ANumber >> 20)));
        IBufWrite(UintToHexChar (0x0F & (ANumber >> 16)));
        MustPrint = true;
    }
    if ((ANumber & 0x0000FF00) or MustPrint or (ACharCount > 2)) {
        IBufWrite(UintToHexChar (0x0F & (ANumber >> 12)));
        IBufWrite(UintToHexChar (0x0F & (ANumber >> 8)));
    }
    IBufWrite(UintToHexChar (0x0F & (ANumber >> 4)));
    IBufWrite(UintToHexChar (0x0F & (ANumber)));
}

void CmdUnit_t::IPrint8HexArray(uint8_t *Arr, uint32_t ALength) {
    for (uint32_t i=0; i<ALength; i++) {
        IPrintAsHex(Arr[i], 2);
        IBufWrite(' ');
    }
}

/*
 * c: character
 * i: signed int
 * u: unsigned int
 * s, S: string
 * X: hexadecimal, capital letters
 * (1...9) width - Minimum number of characters to be printed
 * A - array as 8-bit hexadecimal, length is to be next after uint8_t*
 */
/*
void CmdUnit_t::Printf(const char *S, ...) {
    char c;
    bool WasPercent = false;
    va_list Arg;
    va_start(Arg, S);    // Set pointer to first argument
    while ((c = *S) != 0) {
        if (c == '%') {
            if (WasPercent) {
                IBufWrite(c);  // %%
                WasPercent = false;
            }
            else WasPercent = true;
        }
        else { // not %
            if (WasPercent) {
                // Get number of characters
                char c1 = *(S+1);
                uint8_t N;
                if ((c1 >= '0') and (c1 <= '9')) {
                    N = c1 - '0';
                    S++;
                }
                else N = 0;
                // Parse c
                if (c == 'c') IBufWrite((uint8_t)va_arg(Arg, int32_t));
                else if (c == 'u') IPrintUint(va_arg(Arg, uint32_t), N);
                else if (c == 'i') IPrintInt(va_arg(Arg, uint32_t), N);
                else if ((c == 's') || (c == 'S')) IPrintString(va_arg(Arg, char*));
                else if (c == 'X') IPrintAsHex(va_arg(Arg, uint32_t), N);
                else if (c == 'A') IPrint8HexArray(va_arg(Arg, uint8_t*), va_arg(Arg, uint32_t));
                WasPercent = false;
            } // if was percent
            else IBufWrite(c);
        }
        S++;
    } // while
    va_end(Arg);
    if (IDmaIsIdle) IStartTx();
}
*/

void CmdUnit_t::IBufWrite(uint8_t AByte) {
    if(TxIndx < UART_TXBUF_SIZE) PBuf[TxIndx++] = AByte;    // Write byte to current buffer if possible
    else {
        if(IDmaIsIdle) IStartTx();  // otherwise, start transmission of current buffer and write byte to next one
        else FlushTx();
        PBuf[TxIndx++] = AByte;
    }
}

// Empty buffers now
void CmdUnit_t::FlushTx() {
    while(!IDmaIsIdle or (TxIndx != 0)) {
        if(!IDmaIsIdle) {   // wait DMA
            while(DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);
            DMA_ClearFlag(DMA1_FLAG_TC4);
            IDmaIsIdle = true;
        }
        if(TxIndx != 0) IStartTx();
    }
}

// ==== Init & DMA ====
void CmdUnit_t::Init(uint32_t ABaudrate) {
    PBuf = TXBuf1;
    TxIndx = 0;
    IDmaIsIdle = true;
    // ==== Clocks init ====
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);      // UART clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // ==== GPIO init ====
    klGpioSetupByN(GPIOA, 9, GPIO_Mode_AF_PP);          // TX1
#ifdef RX_ENABLED
    klGpioSetupByN(GPIOA, 10, GPIO_Mode_IPU);   // RX1
#endif
    // ==== USART configuration ====
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = ABaudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
#ifdef RX_ENABLED
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
#else
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
#endif
    USART_Init(USART1, &USART_InitStructure);
    // ==== DMA ====
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(UART_DMA_CHNL);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t) PBuf;
    DMA_InitStructure.DMA_BufferSize         = UART_TXBUF_SIZE;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(UART_DMA_CHNL, &DMA_InitStructure);
    // Start DMA
    USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
    DMA_Cmd(UART_DMA_CHNL, DISABLE);

#ifdef RX_ENABLED
    // ==== NVIC ====
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);     // Configure the NVIC Preemption Priority Bits
    // Enable the USART Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // Enable RX interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#endif
    // Enable USART
    USART_Cmd(USART1, ENABLE);
}

void CmdUnit_t::Task() {
#ifdef RX_ENABLED
    if (CmdState == csReady) {
        NewCmdHandler();
        CmdReset();
    }
#endif
    if (DMA_GetFlagStatus(DMA1_FLAG_TC4)) {	// if transmission completed
        DMA_ClearFlag(DMA1_FLAG_TC4);
        // Switch to next buffer if needed
        if(TxIndx != 0) IStartTx();
        else IDmaIsIdle = true;
    }
}

void CmdUnit_t::IStartTx(void) {
    IDmaIsIdle = false;
    // Start DMA
    USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
    DMA_Cmd(UART_DMA_CHNL, DISABLE);
    UART_DMA_CHNL->CMAR = (uint32_t) PBuf;  // Set memory base address
    UART_DMA_CHNL->CNDTR = TxIndx;          // Set count to transmit
    DMA_Cmd(UART_DMA_CHNL, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    // Switch to next buf
    PBuf = (PBuf == TXBuf1)? TXBuf2 : TXBuf1;
    TxIndx = 0;
}

// ==== IRQs ====
#ifdef RX_ENABLED
void CmdUnit_t::IRQHandler() {
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        char b = USART1->DR;
        if (b != '\n') switch (CmdState) {  // Ignore \n
            case csNone:
                RXBuf[RxIndx++] = b;
                CmdState = csInProgress;
                break;
            case csInProgress:
                // Check if end of cmd
                if (b == '\r') {
                    CmdState = csReady;
                    RXBuf[RxIndx] = 0;
                }
                else {
                    RXBuf[RxIndx++] = b;
                    // Check if too long
                    if (RxIndx == UART_RXBUF_SIZE) CmdReset();
                }
                break;
            case csReady:   // New byte received, but command still not handled
                break;
        } // switch
    } // if rx
}

void USART1_IRQHandler(void) {
    CmdUnit.IRQHandler();
}
#endif

