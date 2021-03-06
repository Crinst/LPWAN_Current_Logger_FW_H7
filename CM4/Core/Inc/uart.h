/*
 * uart.h
 *
 *  Created on: Feb 23, 2020
 *      Author: Michal-Dell
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "ringbuff.h"
#include "main.h"

typedef uint8_t  u8;

typedef uint16_t u16;

typedef uint32_t u32;

#define UART_TX_RINGBUFF_SZ 16384
#define UART_TX_MAX_MESSAGE_LEN 512


u8 txBuf[UART_TX_RINGBUFF_SZ];
u16 txLen;
ringbuff_t txRing;


void UARTTXInit(void) ;
void UARTAddToTxBuff(const void *data,u16 len);

void UARTTxData(void);

void UARTTxComplete(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);


void UARTAddToTxBuff2(const void *data,u16 len);

//static void UARTTxData2(void);
void UARTTxComplete2(void);



#define UART_RX_RINGBUFF_SZ 2048
#define UART_RX_MAX_MESSAGE_LEN 512
#define UART_DMA_WRITE_PTR ((UART_RX_RINGBUFF_SZ - huart6.hdmarx->Instance->NDTR) & (UART_RX_RINGBUFF_SZ - 1))

u8 rxBuf[UART_RX_RINGBUFF_SZ],rxLen;
ringbuff_t rxRing;
u16 rxLastPos,rxThisPos;

void UARTRXInit(void);

void UARTRxComplete(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

uint8_t UART_RX_Read(uint8_t *dataBuffer, uint8_t size);

uint16_t UART_RX_AVAILABLE_BYTE();

#endif /* INC_UART_H_ */
