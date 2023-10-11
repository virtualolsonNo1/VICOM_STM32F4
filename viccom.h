#pragma once
#define STM32F411

#include "stdint.h"
// #include ""
#include "stdbool.h"
#include <stdint.h>

#define MAX_PAYLOAD_LENGTH 256
#define HEADER_LENGTH 4

typedef enum {
    STATE_WAITING_FOR_ADDRESS,
    STATE_WAITING_FOR_LENGTH,
    STATE_WAITING_FOR_CRC1,
    STATE_WAITING_FOR_CRC2,
    STATE_WAITING_FOR_DMA_DONE
} rx_state_t;

#ifdef SAMG55
#include "samg55.h"
#endif


#ifdef FREERTOS
    #include "FreeRTOS.h"
    #include "task.h"
#endif


#ifdef NUVOTON

#include "NuMicro.h"

typedef struct {
    bool txBusy;
    bool addressFlag;
    uint8_t address;
    uint8_t rxData[256];
    int rxLength;
    int idx;
    volatile uint32_t* txEnable;
    UART_T * uart;
    int uartIrq;
    int pdmaTxUartNum;
    int pdmaRxUartNum;
    int pdmaRxChannel;
    int pdmaTxChannel;
    
    bool dataAvailable;
    
    #ifdef FREERTOS
    TaskHandle_t handlingTask;
    #endif
} viccom_t;

void VICCOM_nuvoton_init(viccom_t* state, UART_T* uart, volatile uint32_t* txEnable, uint8_t address);

#endif

#ifdef HC32

typedef struct {
    bool txBusy;
    uint8_t address;
    bool addressFlag;
    uint8_t data[64];
    int length;
    int idx;
} viccom_t;

void VICCOM_hc32_init(viccom_t* state, uint8_t address);

#endif

#ifdef SAMG55

typedef struct {
    bool txBusy;
    uint8_t address;
    bool addressFlag;
    uint8_t rxData[256];
    int rxLength;
    int txEnable;
    Flexcom* flexcom;
    Usart* usart;
    Pdc* pdc;
    bool dataAvailable;
} viccom_t;

void VICCOM_init(viccom_t* state, Flexcom* flexcom, Usart* uart, int txEnable, uint8_t address);

#endif

#ifdef STM32F411
// #include "stm32f4xx_hal_uart.h"
// #include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal.h"

typedef struct {
    bool txBusy;
    uint8_t address;
    bool addressFlag;
    uint8_t* payloadData;
    uint8_t rxData[MAX_PAYLOAD_LENGTH + 2]; // payload, payload CRC16
    uint8_t rxHeader[4];
    uint16_t txData[HEADER_LENGTH + MAX_PAYLOAD_LENGTH + 2]; // header, payload, payload CRC16
    int rxLength;
    UART_HandleTypeDef* uart;
    DMA_HandleTypeDef* tx_dma;
    DMA_HandleTypeDef* rx_dma;
    bool dataAvailable;
    rx_state_t currentState;
    bool resetBool;
} viccom_t;


void VICCOM_stm_init(viccom_t* state, uint8_t address, UART_HandleTypeDef* uart, DMA_HandleTypeDef* tx_dma, DMA_HandleTypeDef* rx_dma);

#endif

typedef struct {
    uint8_t* data;
    int length;
} viccom_rx_t;


void VICCOM_task(void);

bool VICCOM_dataAvailable(viccom_t* state);
viccom_rx_t VICCOM_getBuffer(viccom_t* state);
#ifdef FREERTOS
BaseType_t VICCOM_waitForRX(viccom_t* state, TaskHandle_t xHandlingTask);
#endif

uint8_t* VICCOM_getTxBuffer(viccom_t* state);
int VICCOM_getMaxTxLength(viccom_t* state);
void VICCOM_send(viccom_t* state, int len);
void VICCOM_dataReceived(viccom_t* state, uint8_t* data, int len);

void VICCOM_uartHandler(viccom_t* state);
void VICCOM_pdmaHandler(viccom_t* state);

void VICCOM_setAddress(viccom_t* state, int address);

uint8_t* VICCOM_getTxBuffer(viccom_t* state);
int VICCOM_getMaxTxLength(viccom_t* state);