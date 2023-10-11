#include <stdint.h>
#include "stdbool.h"
#include "viccom.h"
#include "main.h"
#include "crc16.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "viccom.h"
#include "pb.h"
extern volatile int msgCount;

void VICCOM_stm_init(viccom_t* state, uint8_t address, UART_HandleTypeDef* uart, DMA_HandleTypeDef* tx_dma, DMA_HandleTypeDef* rx_dma) {
    state->address = address;
    state->addressFlag = false;
    state->currentState = STATE_WAITING_FOR_ADDRESS;
    state->uart = uart;
    state->tx_dma = tx_dma;
    state->rx_dma = rx_dma;
    CRC_init();
}

void VICCOM_send(viccom_t* state, int len) {
    // address, header, and crc
    state->txData[0] = state->address;
    state->txData[1] = len;
    volatile uint16_t addressLengthCRC = CRC_compute16((uint8_t *)state->txData, 2);
    state->txData[0] = state->address | 0x100;
    state->txData[2] = addressLengthCRC & 0xFF;
    state->txData[3]= (addressLengthCRC >> 8) & 0xFF;

    //for loop to copy uint8_t payload array into txData
    for(int i = 0; i < len; i++) {
        state->txData[i + HEADER_LENGTH] = state->payloadData[i];
    }

    //compute and store payload CRC in txData
    volatile uint16_t payloadCRC = CRC_compute16((uint8_t *)(state->txData + HEADER_LENGTH), len);
    state->txData[HEADER_LENGTH + len] = payloadCRC & 0xFF;
    state->txData[HEADER_LENGTH + len + 1] = (payloadCRC >> 8) & 0xFF;

    //enable DMA properly
    //select channel 4 for DMA1 Stream 4, or 100 for bits 27:25
    DMA1_Stream4->CR |= DMA_SxCR_CHSEL_2;
    DMA1_Stream4->CR &= ~(DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1);

    //set address for where to send data to for transfer
    DMA1_Stream4->PAR = (uint32_t)&UART4->DR;
    //Set memory address for where to get data
    DMA1_Stream4->M0AR = (uint32_t)&state->txData;
    //enable minc so that it increments memory address with each transfer
    DMA1_Stream4->CR |= DMA_MINC_ENABLE;
    //set msize to 16 bits to allow for address bit to be 1 for address, in this case the value of 01
    DMA1_Stream4->CR |= DMA_SxCR_MSIZE_0;
    DMA1_Stream4->CR &= ~(DMA_SxCR_MSIZE_1);
    //set psize to 16 bits to allow for address bit to be 1 for address, in this case the value of 01
    DMA1_Stream4->CR |= DMA_SxCR_PSIZE_0;
    DMA1_Stream4->CR &= ~(DMA_SxCR_PSIZE_1);
    //length of data to store, in this case length, crcl, crch
    DMA1_Stream4->NDTR = HEADER_LENGTH + len + 2;
    //enable TCIE
    DMA1_Stream4->CR |= DMA_SxCR_TCIE;
    DMA1_Stream4->CR |= DMA_SxCR_TEIE;

    //set direction to 1 (memory to peripheral). Should already be done, but just in case
    DMA1_Stream4->CR |= DMA_SxCR_DIR_0;
    DMA1_Stream4->CR &= ~(DMA_SxCR_DIR_1);

    //set txBusy to true
    state->txBusy = true;
    state->resetBool = false;

    //disable DMA
    UART4->CR3 &= ~(USART_CR3_DMAR);
    NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    DMA1_Stream2->CR &= ~(DMA_SxCR_EN);

    //enable DMA
    UART4->CR3 |= USART_CR3_DMAT;
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    DMA1_Stream4->CR |= DMA_SxCR_EN;

    return;
}    

void VICCOM_pdmaHandler(viccom_t *state) {
    // handle any RX errors that might occur even if we're transmitting
    if(DMA1->LISR & DMA_LISR_TEIF2) {
        //clear Transfer Error Interrupt Flag for stream 2
        DMA1->LIFCR |= DMA_LIFCR_CTEIF2;
        reset_receive(state);

    } else if(DMA1->LISR & DMA_LISR_FEIF2) {
        //clear FIFO error interrupt flag
        DMA1->LIFCR |= DMA_LIFCR_CFEIF2;
        reset_receive(state);

    } else if (DMA1->HISR & DMA_HISR_TEIF4) {
        //clear Transfer Error Interrupt Flag for stream 4
        DMA1->HIFCR |= DMA_HIFCR_CTEIF4;
        reset_transmit(state);
    } 
    else if (DMA1->HISR & DMA_HISR_FEIF4) {
        DMA1->HIFCR |= DMA_HIFCR_CFEIF4;
        reset_transmit(state);
    }
    //if transfer was successfully completed, set txBusy back to false
    else if(state->txBusy && DMA1->HISR & DMA_HISR_TCIF4) {

        state->txBusy = false;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
        state->resetBool = true;
        reset_receive(state);

    } else {
        volatile uint16_t result;
        switch(state->currentState) {
            case STATE_WAITING_FOR_LENGTH:
                //Clear HTIF as it won't be cleared by HAL later
                DMA1->LIFCR |= DMA_LISR_HTIF2;
                state->rxLength = state->rxHeader[1];

                //concatenate low and high crc values into one crc value
                result = CRC_compute((uint8_t *)state->rxHeader, HEADER_LENGTH);
                if(result == 0) {
                    //enable DMA properly
                    //length of data to store, in this case length of payload + 2 for crcl and crch
                    DMA1_Stream2->NDTR = state->rxLength + 2;
                    //set address for where to get data from as UART4->RDR
                    DMA1_Stream2->PAR = (uint32_t)&UART4->DR;
                    //Set memory address for where to store data
                    DMA1_Stream2->M0AR = (uint32_t)&state->rxData;
                    //set state to waiting for dma done
                    state->currentState = STATE_WAITING_FOR_DMA_DONE;

                    UART4->CR3 |= USART_CR3_DMAR;
                    DMA1_Stream2->CR |= DMA_SxCR_TCIE;
                    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
                    DMA1_Stream2->CR |= DMA_SxCR_EN;
                    
                } else {
                    printf("CRC ADDRESS/LENGTH ERROR!!!\n");
                    reset_receive(state);
                }
                break;
            case STATE_WAITING_FOR_DMA_DONE: 
                //compute CRC
                result = CRC_compute((uint8_t *)state->rxData, state->rxLength + 2);
                if(result == 0) {
                    state->currentState = STATE_WAITING_FOR_ADDRESS;
                    state->dataAvailable = true;
                    //disable DMA
                    UART4->CR3 &= ~(USART_CR3_DMAR);
                    NVIC_DisableIRQ(DMA1_Stream2_IRQn);
                    DMA1_Stream2->CR &= ~(DMA_SxCR_EN);

                    //re-enable UART4 interrupt to wait for its address again
                    NVIC_EnableIRQ(UART4_IRQn);
                    //enable rx interrupt
                    UART4->CR1 |= USART_CR1_RXNEIE;
                    msgCount++;
                    VICCOM_send(state, state->rxLength);
                } else {
                    printf("CRC ADDRESS/LENGTH ERROR!!!\n");
                    reset_receive(state);
                }
                break;
        }
    }
}

void VICCOM_uartHandler(viccom_t *state) {
    if(UART4->SR & USART_SR_ORE) {
        volatile uint32_t statusReg = UART4->SR;
    }
    volatile uint16_t data = UART4->DR;
    if(state->currentState == STATE_WAITING_FOR_ADDRESS && data & 0x100  && (data & 0xff) == state->address) {
        //store in rxHeader[0]
        state->rxHeader[0] = data;
        // we received an address byte containing our address, now need to get length and crc's
        state->currentState = STATE_WAITING_FOR_LENGTH;

        //disable UART4 interrupt
        NVIC_DisableIRQ(UART4_IRQn);
        UART4->CR1 &= ~(USART_CR1_RXNEIE);
 

        //enable DMA properly
        //select channel 4 for DMA1 Stream 4, or 100 for bits 27:25
        DMA1_Stream2->CR |= DMA_SxCR_CHSEL_2;
        DMA1_Stream2->CR &= ~(DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1);
        //set address for where to get data from as UART4->RDR
        DMA1_Stream2->PAR = (uint32_t)&UART4->DR;
        //Set memory address for where to store data
        DMA1_Stream2->M0AR = (uint32_t)state->rxHeader + 1;
        //enable minc so that it increments memory address with each transfer
        DMA1_Stream2->CR |= DMA_MINC_ENABLE;
        //length of data to store, in this case length, crcl, crch
        DMA1_Stream2->NDTR = 3;
        //enable TCIE
        DMA1_Stream2->CR |= DMA_SxCR_TCIE;
        //enable FIFO overrun/underrun error
        DMA1_Stream2->CR |= DMA_SxCR_TEIE;
        DMA1_Stream2->FCR |= DMA_SxFCR_FEIE;
        DMA1_Stream2->CR |= DMA_SxCR_DMEIE;

        //enable DMA
        UART4->CR3 |= USART_CR3_DMAR;
        NVIC_EnableIRQ(DMA1_Stream2_IRQn);
        DMA1_Stream2->CR |= DMA_SxCR_EN;
    } else {
        reset_receive(state);
    }

    return;

}

void reset_receive(viccom_t *state) {
    //set state back to waiting for address
    state->currentState = STATE_WAITING_FOR_ADDRESS;

    //disable DMA
    UART4->CR3 &= ~(USART_CR3_DMAR);
    NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    DMA1_Stream2->CR &= ~(DMA_SxCR_EN);

    //re-enable UART4 interrupt to wait for its address again
    UART4->CR1 |= USART_CR1_WAKE;
    UART4->CR2 |= (USART_CR2_ADD & state->address);

    //enable rx interrupt
    NVIC_EnableIRQ(UART4_IRQn);
    UART4->CR1 |= USART_CR1_RXNEIE;

    volatile uint32_t statusReg = UART4->SR;
    volatile uint16_t dataReg = UART4->DR;
}

void reset_transmit(viccom_t *state) {
    //disable DMA
    UART4->CR3 &= ~(USART_CR3_DMAT);
    NVIC_DisableIRQ(DMA1_Stream4_IRQn);
    DMA1_Stream4->CR &= ~(DMA_SxCR_EN);

    //set state's txbusy field back to false
    state->txBusy = false;

    //clear errors
    volatile uint32_t statusReg = UART4->SR;
    volatile uint16_t dataReg = UART4->DR;
}