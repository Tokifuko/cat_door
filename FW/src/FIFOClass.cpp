#include "FIFOClass.h"
#include <stdio.h>
#include <stdlib.h>

uint8_t FIFO_Class :: Init(uint16_t iSize)
{
	//pStart=(uint8_t*) malloc (iSize);
	//if (pStart==NULL)return FIFO_NO_MEMORY;
    if (iSize>FIFO_SIZE) return FIFO_NO_MEMORY;
	iBufSize=iSize;
	pTail=pStart;
	pHead=pStart;
	pEnd=pStart+iSize;
	chEmptyFlag=true;
	chFullFlag=false;
	return FIFO_NO_ERROR;
}
uint16_t FIFO_Class ::Length( void )
{
	if (IsEmpty()) return 0;
	if (IsFull()) return 1;
	if (pHead<pTail) return (pTail-pHead);
	return (iBufSize-(pHead-pTail));
}

uint16_t FIFO_Class ::WriteByte(uint8_t chByte)
{
	if (IsFull()) return 0;
	*pTail=chByte;
	pTail++;
	if (pTail==pEnd) pTail=pStart;
	chEmptyFlag=false;
	if (pTail==pHead) chFullFlag=true;
	return 1;
}
uint16_t FIFO_Class ::WriteData(uint16_t iDataSize,uint8_t* pDataBuf)
{
	uint16_t i=0;
	while (i<iDataSize)
	{
		if (WriteByte(*pDataBuf)==0) return i;
		i++;
		pDataBuf++;
	}
	return i;
}
uint8_t FIFO_Class ::ReadByte(uint8_t* pData)
{
	if (IsEmpty()) return 0;
	*pData=*pHead;
	pHead++;
	if (pHead==pEnd) pHead=pStart;
	chFullFlag=false;
	if (pTail==pHead) chEmptyFlag=true;
	return 1;
}
uint8_t FIFO_Class ::SimpleReadByte(void)
{
	uint8_t chData;
	ReadByte(&chData);
	return chData;
}
uint16_t FIFO_Class ::ReadData(uint16_t iDataSize,uint8_t* pDataBuf)
{
	uint16_t i=0;
	while (i<iDataSize)
	{
		if (ReadByte(pDataBuf)==0) return i;
		i++;
		pDataBuf++;
	}
	return i;
}
bool FIFO_Class ::IsEmpty (void) {return chEmptyFlag;}

bool FIFO_Class ::IsFull (void) {return chFullFlag;}
