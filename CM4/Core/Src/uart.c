#include "ringbuff.h"
#include "uart.h"
#include "main.h"

typedef uint8_t  u8;

typedef uint16_t u16;

typedef uint32_t u32;

#define UART_TX_RINGBUFF_SZ 16384
#define UART_TX_MAX_MESSAGE_LEN 512
#define UART_TX_MIN_MESSAGE_LEN 128


u8 txBuf[UART_TX_RINGBUFF_SZ];
u16 txLen;
ringbuff_t txRing;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern uint8_t isWaitingForData;
//extern uint8_t isReadyForNext;

//extern uint8_t processBuf[250];


void UARTTXInit(void) {
  ringbuff_init(&txRing,txBuf,UART_TX_RINGBUFF_SZ);
}

void UARTAddToTxBuff(const void *data,u16 len) {
  ringbuff_write(&txRing,data,len);
  UARTTxData();
}



void UARTTxData(void) {
  if(txLen) return; //If len > 0, DMA transfer is on-going. This function will be called again at transfer completion
  txLen=ringbuff_get_linear_block_read_length(&txRing); //Get maximal length of buffer to read data as linear memory
  //if(txLen >= UART_TX_MIN_MESSAGE_LEN){
  if(txLen){
   u16* ringData=ringbuff_get_linear_block_read_address(&txRing); // Get pointer to read memory
   HAL_UART_Transmit_DMA(&huart2,ringData,txLen); // Start DMA transfer
   //HAL_UART_Transmit_DMA(&huart2, ringData, txLen);
  }
}

void UARTTxComplete(void) {
  if (txLen) {
   ringbuff_skip(&txRing,txLen); // Now skip the data (move read pointer) as they were successfully transferred over DMA
   txLen=0; // Reset length = DMA is not active
   UARTTxData(); // Try to send more
  }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//isReadyForNext = 1;

  if (huart->Instance == USART2) {
	  //isReadyForNext = 1;

	  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE)); // Disable TXEIE and TCIE interrupts
	  huart->gState = HAL_UART_STATE_READY; // Tx process is ended, restore huart->gState to Ready
	  UARTTxComplete();
  }



}

/*

#define UART_RX_RINGBUFF_SZ 2048 ///512
#define UART_RX_MAX_MESSAGE_LEN 512
#define UART_DMA_WRITE_PTR ((UART_RX_RINGBUFF_SZ - huart2.hdmarx->Instance->NDTR) & (UART_RX_RINGBUFF_SZ - 1))

u8 rxBuf[UART_RX_RINGBUFF_SZ],rxLen;
ringbuff_t rxRing;
u16 rxLastPos,rxThisPos;

void UARTRXInit(void) {
  ringbuff_init(&rxRing,rxBuf,UART_RX_RINGBUFF_SZ);
  rxLastPos=0;
  rxThisPos=0;
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);   // enable idle line interrupt
  hdma_usart2_rx.Instance->CR &= ~DMA_SxCR_HTIE;  // disable uart half tx interrupt
  HAL_UART_Receive_DMA(&huart2,rxBuf,UART_RX_RINGBUFF_SZ);
}

void UARTRxComplete(void) {
  u8 addr;
  u16 len;
  rxThisPos=UART_DMA_WRITE_PTR; //get current write pointer
  len=(rxThisPos-rxLastPos+UART_RX_RINGBUFF_SZ)%UART_RX_RINGBUFF_SZ; //calculate how far the DMA write pointer has moved
  if(len<=UART_RX_MAX_MESSAGE_LEN) { //check message size
   ringbuff_advance(&rxRing,len); //move the ring buffer write pointer
   rxLastPos=rxThisPos;
   //ringbuff_read(&rxRing,processBuf,len); //read out the data to an array for processing
  }
  else {
    //while(1); //implement message to large exception
	  //HAL_Delay(10000);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	isWaitingForData = 0;

  if (huart->Instance == USART2) {
    if (__HAL_UART_GET_FLAG (&huart2, UART_FLAG_IDLE)) {
      UARTRxComplete();
    }
  }
}

uint8_t UART_RX_Read(uint8_t *dataBuffer, uint8_t size){

	return ringbuff_read(&rxRing, dataBuffer, size);

}

uint16_t UART_RX_AVAILABLE_BYTE(){

	return ringbuff_get_full(&rxRing);

}
*/
