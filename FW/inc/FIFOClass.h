/* тут описывается класс для создания и работы с FIFO буферами */

#ifndef FIFOCLASS_H
#define FIFOCLASS_H

#define FIFO_NO_ERROR	0
#define FIFO_NO_MEMORY	1
#define FIFO_SIZE	64 // не работает malloc

#include "stm32f10x_conf.h"

class FIFO_Class {
private:
	uint8_t  *pTail;
	uint8_t  *pHead;
	//uint8_t  *pStart;
	uint8_t pStart[FIFO_SIZE];
	uint8_t	*pEnd;
	bool	chFullFlag;
	bool	chEmptyFlag;
	uint16_t iBufSize;
public:
	uint8_t Init(uint16_t iSize);
	uint16_t Length( void );
	uint16_t WriteByte(uint8_t chByte);
	uint16_t WriteData(uint16_t iDataSize,uint8_t* pDataBuf);
	uint8_t ReadByte(uint8_t* pData);
	uint8_t SimpleReadByte(void);
	uint16_t ReadData(uint16_t iDataSize,uint8_t* pDataBuf);
	bool IsEmpty (void);
	bool IsFull (void);

    // ~FIFO_Class(void); // деструктор
};

#endif /* FIFOCLASS_H */
