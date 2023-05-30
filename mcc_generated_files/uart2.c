/**
  UART2 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart2.c

  @Summary
    This is the generated source file for the UART2 driver using Foundation Services Library

  @Description
    This source file provides APIs for driver for UART2. 

    Generation Information : 
        Product Revision  :  Foundation Services Library - pic24-dspic-pic32mm : v1.26
        Device            :  PIC32MM0256GPM064
    The generated drivers are tested against the following:
        Compiler          :  XC32 1.42
        MPLAB 	          :  MPLAB X 3.45
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include <stdbool.h>
#include <stdint.h>
#include "xc.h"
#include "uart2.h"
#include "../MyMain.h"

/**
  Section: Data Type Definitions
*/

/** UART Driver Queue Status

  @Summary
    Defines the object required for the status of the queue.
*/

static uint8_t * volatile rxTail;
static uint8_t * rxHead;
static uint8_t * txTail;
static uint8_t * volatile txHead;
static bool volatile rxOverflowed;
extern bool isSensitiveScaleZero,sensitiveDataCompleted,startSave,requestZero,Is_Around_Tolerance;
extern uint8_t sensitiveScaleDataPack[50],sPack[50],Prn_Write_Buf[PRN_MAX_WRITE],Prn_Read_Buf[PRN_MAX_READ];
extern uint16_t  PrnOutputWrp,PrnOutputRrp,PrnInputWrp,PrnInputRrp;

/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART2_CONFIG_TX_BYTEQ_LENGTH (8+1)
#define UART2_CONFIG_RX_BYTEQ_LENGTH (8+1)

/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t txQueue[UART2_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t rxQueue[UART2_CONFIG_RX_BYTEQ_LENGTH] ;

void (*UART2_TxDefaultInterruptHandler)(void);
void (*UART2_RxDefaultInterruptHandler)(void);

/**
  Section: Driver Interface
*/
extern uint32_t ComputeCrc(uint32_t DataLenght);
extern uint16_t rnginc(uint16_t temp1,uint16_t temp2);
extern uint8_t waitForSensitivReceivedWeightCounter;

void UART2_Initialize (void)
{
   IEC1bits.U2TXIE = 0;
   IEC1bits.U2RXIE = 0;
   
   // STSEL 1; PDSEL 8N; RTSMD disabled; OVFDIS disabled; ACTIVE disabled; RXINV disabled; WAKE disabled; BRGH enabled; IREN disabled; ON enabled; SLPEN disabled; SIDL disabled; ABAUD disabled; LPBACK disabled; UEN TX_RX; CLKSEL SYSCLK; 
   U2MODE = (0x28008 & ~(1<<15));  // disabling UART
   // UTXISEL TX_ONE_CHAR; UTXINV disabled; ADDR 0; MASK 0; URXEN disabled; OERR disabled; URXISEL RX_ONE_CHAR; UTXBRK disabled; UTXEN disabled; ADDEN disabled; 
   U2STA = 0x0;
   // BaudRate = 9600; Frequency = 24000000 Hz; BRG 624; 
   U2BRG = 0x270;
   
   txHead = txQueue;
   txTail = txQueue;
   rxHead = rxQueue;
   rxTail = rxQueue;
   
   rxOverflowed = 0;

   UART2_SetTxInterruptHandler(UART2_Transmit_ISR);

   UART2_SetRxInterruptHandler(UART2_Receive_ISR);
   IEC1bits.U2RXIE = 1;
   
    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
   U2STASET = _U2STA_UTXEN_MASK;
   U2MODESET = _U2MODE_ON_MASK;  // enabling UART ON bit
   U2STASET = _U2STA_URXEN_MASK; 
}

/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void UART2_SetTxInterruptHandler(void* handler){
    UART2_TxDefaultInterruptHandler = handler;
}

void __attribute__ ((vector(_UART2_TX_VECTOR), interrupt(IPL1SOFT))) _UART2_TX ( void )
{
    (*UART2_TxDefaultInterruptHandler)();
}



void UART2_Transmit_ISR ( void )
{ 
    if(IEC1bits.U2TXIE)
    {
        if(PrnOutputWrp!=PrnOutputRrp)
        {
            U2TXREG=Prn_Write_Buf[PrnOutputRrp];
            PrnOutputRrp=rnginc(PrnOutputRrp,PRN_MAX_WRITE); 
            IFS1CLR= 1 << _IFS1_U2TXIF_POSITION;
        }
        else
        {
            IEC1bits.U2TXIE=0;
        }
    }
}

void UART2_SetRxInterruptHandler(void* handler){
    UART2_RxDefaultInterruptHandler = handler;
}

void __attribute__ ((vector(_UART2_RX_VECTOR), interrupt(IPL1SOFT))) _UART2_RX( void )
{
    (*UART2_RxDefaultInterruptHandler)();
}

void UART2_Receive_ISR(void)
{
    Prn_Read_Buf[PrnInputWrp]=U2RXREG;
    PrnInputWrp=rnginc(PrnInputWrp,PRN_MAX_READ);
    IFS1CLR= 1 << _IFS1_U2RXIF_POSITION;
}

void __attribute__ ((vector(_UART2_ERR_VECTOR), interrupt(IPL1SOFT))) _UART2_ERR ( void )
{
    if ((U2STAbits.OERR == 1))
    {
        U2STACLR = _U2STA_OERR_MASK; 
    }

     IFS1CLR= 1 << _IFS1_U2EIF_POSITION;
}

/**
  Section: UART Driver Client Routines
*/
void SensitiveParseData(uint8_t data)
{
    static uint8_t mashinState=0,packLen=0;
    static bool isNegative=false,addZero=false;
    uint16_t crc,index=0;
    
    if (mashinState == 0 && (data == 'U' || data == 'S'))
    {
        mashinState = 1;
    }
    else
        if (mashinState == 1 && (data == 'S' || data == 'T'))
    {
        mashinState = 2;
    }
    else
        if (mashinState == 2 && data == ',')
    {
        mashinState = 3;
    }
    else
        if (mashinState == 3)
    {
        if (data == '+')
            isNegative = false;
        else
        if (data == '-')
            isNegative = true;
        mashinState = 4;
    }
    else
        if (mashinState == 4)
    {
        startSave=false;
        mashinState = 5;
        sensitiveScaleDataPack[0] = 0x01;
        sensitiveScaleDataPack[1] = 0x14;
        sensitiveScaleDataPack[2] = 0x00;
        packLen = 3;
        if(data!='0')
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 5)
    {
        mashinState = 6;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
        
    }
    else
        if (mashinState == 6)
    {
        mashinState = 7;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 7)
    {
        mashinState = 8;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 8)
    {
        mashinState = 9;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 9 && data=='.')
    {
        mashinState = 10;
    }
    else
        if (mashinState == 10)
    {
        mashinState = 11;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 11)
    {
        mashinState = 12;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 12 && data==' ')
    {
        mashinState = 13;
        sensitiveScaleDataPack[packLen] = '0';
        packLen++;
        sensitiveScaleDataPack[packLen] = ',';
        packLen++;
        
        if(sensitiveScaleDataPack[packLen-2]=='0' && sensitiveScaleDataPack[packLen-3]=='1' && sensitiveScaleDataPack[packLen-5]==0x14 && sensitiveScaleDataPack[packLen-6]==0x1)
            Is_Around_Tolerance=true;
        else
            Is_Around_Tolerance=false;

                        
        for (index = 0; index < 6; index++)
        {
            sensitiveScaleDataPack[packLen] = '1'+index;
            packLen++;
        }
        if (isNegative == false)
        {
            sensitiveScaleDataPack[2] = packLen - 3;
            crc = (uint16_t)ComputeCrc(packLen);
            sensitiveScaleDataPack[packLen] = (uint8_t)(crc >> 8);
            packLen++;
            sensitiveScaleDataPack[packLen] = (uint8_t)(crc & 0xff);
            isSensitiveScaleZero=true;
            index=3;
            packLen++;
            for(;sensitiveScaleDataPack[index]!=',';index++)
                if(sensitiveScaleDataPack[index]!='0')
                    isSensitiveScaleZero=false;
        }
        else
        {
            sensitiveScaleDataPack[0] = 0x01;
            sensitiveScaleDataPack[1] = 0x14;
            sensitiveScaleDataPack[2] = 0x09;
            sensitiveScaleDataPack[3] = '-';
            sensitiveScaleDataPack[4] = '1';
            sensitiveScaleDataPack[5] = ',';
            sensitiveScaleDataPack[6] = '1';
            sensitiveScaleDataPack[7] = '2';
            sensitiveScaleDataPack[8] = '3';
            sensitiveScaleDataPack[9] = '4';
            sensitiveScaleDataPack[10] ='5';
            sensitiveScaleDataPack[11] ='6';
            crc = (uint16_t)ComputeCrc(12);
            sensitiveScaleDataPack[12] = (uint8_t)(crc >> 8);
            sensitiveScaleDataPack[13] = (uint8_t)(crc & 0xff);
            packLen=14;
        }
    }
    else
        if (mashinState == 13 && data==' ')
    {
        mashinState = 14;
    }
    else
        if (mashinState == 14 && data=='g')
    {
        mashinState = 15;
    }
    else
        if (mashinState == 15 && data==13)
    {
        mashinState = 16;
    }
    else
        if (mashinState == 16 && data==10)
    {
        mashinState = 0;
        for(index=0;index<packLen;index++)
            sPack[index]=sensitiveScaleDataPack[index];
        sensitiveDataCompleted=true;
        waitForSensitivReceivedWeightCounter=0;
        if(requestZero==true)
        {
            requestZero=false;
            UART2_Write('Z');
            UART2_Write(13);
            UART2_Write(10);
        }
    }
    else
    {
        mashinState = 0;
        packLen = 0;
        addZero=false;
        PrnInputWrp=PrnInputRrp;
    }
}

/*

 void SensitiveParseData(uint8_t data)
{
    static uint8_t mashinState=0,packLen=0;
    static bool isNegative=false,addZero=false;
    uint16_t crc,index=0;
    
    if (mashinState == 0 && data == 'S')// || data == 'U'))
    {
        mashinState = 1;
    }
    else
        if (mashinState == 1 && data == 'T')// || data == 'S'))
    {
        mashinState = 2;
    }
    else
        if (mashinState == 2 && data == ',')
    {
        mashinState = 3;
    }
    else
        if (mashinState == 3)
    {
        if (data == '+')
            isNegative = false;
        else
        if (data == '-')
            isNegative = true;
        mashinState = 4;
    }
    else
        if (mashinState == 4)
    {
        startSave=false;
        mashinState = 5;
        sensitiveScaleDataPack[0] = 0x01;
        sensitiveScaleDataPack[1] = 0x14;
        sensitiveScaleDataPack[2] = 0x00;
        packLen = 3;
        if(data!='0')
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 5)
    {
        mashinState = 6;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
        
    }
    else
        if (mashinState == 6)
    {
        mashinState = 7;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 7)
    {
        mashinState = 8;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 8)
    {
        mashinState = 9;
        if (data != '.')
        {
            if(data!='0' || startSave==true)
            {
                sensitiveScaleDataPack[packLen] = data;
                packLen++;
                startSave=true;
            }
            addZero = true;
        }
    }
    else
        if (mashinState == 9)
    {
        mashinState = 10;
        if (data != '.')
        {
            if(data!='0' || startSave==true)
            {
                sensitiveScaleDataPack[packLen] = data;
                packLen++;
                startSave=true;
            }
        }
    }
    else
        if (mashinState == 10)
    {
        mashinState = 11;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 11)
    {
        mashinState = 12;
        if(data!='0' || startSave==true)
        {
            sensitiveScaleDataPack[packLen] = data;
            packLen++;
            startSave=true;
        }
    }
    else
        if (mashinState == 12)
    {
        mashinState = 13;
        if (addZero == true && data == ' ')
        {
            sensitiveScaleDataPack[packLen] = '0';
            packLen++;
        }
        sensitiveScaleDataPack[packLen] = ',';
        packLen++;

        for (index = 0; index < 6; index++)
        {
            sensitiveScaleDataPack[packLen] = '1'+index;
            packLen++;
        }
        if (isNegative == false)
        {
            sensitiveScaleDataPack[2] = packLen - 3;
            crc = (uint16_t)ComputeCrc(packLen);
            sensitiveScaleDataPack[packLen] = (uint8_t)(crc >> 8);
            packLen++;
            sensitiveScaleDataPack[packLen] = (uint8_t)(crc & 0xff);
            isSensitiveScaleZero=true;
            index=3;
            for(;sensitiveScaleDataPack[index]!=',';index++)
                if(sensitiveScaleDataPack[index]!='0')
                    isSensitiveScaleZero=false;
        }
        else
        {
            sensitiveScaleDataPack[0] = 0x01;
            sensitiveScaleDataPack[1] = 0x14;
            sensitiveScaleDataPack[2] = 0x09;
            sensitiveScaleDataPack[3] = '-';
            sensitiveScaleDataPack[4] = '1';
            sensitiveScaleDataPack[5] = ',';
            sensitiveScaleDataPack[6] = '1';
            sensitiveScaleDataPack[7] = '2';
            sensitiveScaleDataPack[8] = '3';
            sensitiveScaleDataPack[9] = '4';
            sensitiveScaleDataPack[10] ='5';
            sensitiveScaleDataPack[11] ='6';
            crc = (uint16_t)ComputeCrc(12);
            sensitiveScaleDataPack[12] = (uint8_t)(crc >> 8);
            sensitiveScaleDataPack[13] = (uint8_t)(crc & 0xff);
        }
        sensitiveDataCompleted=true;
        packLen = 0;
        addZero=false;
    }
    else
        if (mashinState == 13 && data==' ')
    {
        mashinState = 14;
    }
    else
        if (mashinState == 14 && data=='g')
    {
        mashinState = 15;
    }
    else
        if (mashinState == 15 && data==13)
    {
        mashinState = 16;
    }
    else
        if (mashinState == 16 && data==10)
    {
        mashinState = 0;
        
    }
    else
    {
        mashinState = 0;
        packLen = 0;
        addZero=false;
    }
} 
 */


uint8_t UART2_Read( void)
{
    uint8_t data = 0;

    while (rxHead == rxTail )
    {
    }
    
    data = *rxHead;

    rxHead++;

    if (rxHead == (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH))
    {
        rxHead = rxQueue;
    }
    return data;
}


void SensitiveScaleReadData( void )
{
    uint8_t data = 0;
    if(PrnInputWrp!=PrnInputRrp)
    {
        data=Prn_Read_Buf[PrnInputRrp];
        PrnInputRrp=rnginc(PrnInputRrp,PRN_MAX_READ);
        SensitiveParseData(data);
    }
}


void UART2_Write( uint8_t byte)
{
    while(UART2_IsTxReady() == 0)
    {
    }

    *txTail = byte;

    txTail++;
    
    if (txTail == (txQueue + UART2_CONFIG_TX_BYTEQ_LENGTH))
    {
        txTail = txQueue;
    }

    IEC1bits.U2TXIE = 1;
}

bool UART2_IsRxReady(void)
{    
    return !(rxHead == rxTail);
}

bool UART2_IsTxReady(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART2_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    
    return (size != 0);
}

bool UART2_IsTxDone(void)
{
    if(txTail == txHead)
    {
        return (bool)U2STAbits.TRMT;
    }
    
    return false;
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
static uint8_t UART2_RxDataAvailable(void)
{
    uint16_t size;
    uint8_t *snapshot_rxTail = (uint8_t*)rxTail;
    
    if (snapshot_rxTail < rxHead) 
    {
        size = ( UART2_CONFIG_RX_BYTEQ_LENGTH - (rxHead-snapshot_rxTail));
    }
    else
    {
        size = ( (snapshot_rxTail - rxHead));
    }
    
    if(size > 0xFF)
    {
        return 0xFF;
    }
   
    return size;
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
uint8_t __attribute__((deprecated)) UART2_is_rx_ready(void)
{
    return UART2_RxDataAvailable();
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
static uint8_t UART2_TxDataAvailable(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART2_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    
    if(size > 0xFF)
    {
        return 0xFF;
    }
    
    return size;
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
uint8_t __attribute__((deprecated)) UART2_is_tx_ready(void)
{
    return UART2_TxDataAvailable();
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
bool __attribute__((deprecated)) UART2_is_tx_done(void)
{
    return UART2_IsTxDone();
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
unsigned int __attribute__((deprecated)) UART2_ReadBuffer( uint8_t *buffer ,  unsigned int numbytes)
{
    unsigned int rx_count = UART2_RxDataAvailable();
    unsigned int i;
    
    if(numbytes < rx_count)
    {
        rx_count = numbytes;
    }
    
    for(i=0; i<rx_count; i++)
    {
        *buffer++ = UART2_Read();
    }
    
    return rx_count;    
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
unsigned int __attribute__((deprecated)) UART2_WriteBuffer( uint8_t *buffer , unsigned int numbytes )
{
    unsigned int tx_count = UART2_TxDataAvailable();
    unsigned int i;
    
    if(numbytes < tx_count)
    {
        tx_count = numbytes;
    }
    
    for(i=0; i<tx_count; i++)
    {
        UART2_Write(*buffer++);
    }
    
    return tx_count;  
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
UART2_TRANSFER_STATUS __attribute__((deprecated)) UART2_TransferStatusGet (void )
{
    UART2_TRANSFER_STATUS status = 0;
    uint8_t rx_count = UART2_RxDataAvailable();
    uint8_t tx_count = UART2_TxDataAvailable();
    
    switch(rx_count)
    {
        case 0:
            status |= UART2_TRANSFER_STATUS_RX_EMPTY;
            break;
        case UART2_CONFIG_RX_BYTEQ_LENGTH:
            status |= UART2_TRANSFER_STATUS_RX_FULL;
            break;
        default:
            status |= UART2_TRANSFER_STATUS_RX_DATA_PRESENT;
            break;
    }
    
    switch(tx_count)
    {
        case 0:
            status |= UART2_TRANSFER_STATUS_TX_FULL;
            break;
        case UART2_CONFIG_RX_BYTEQ_LENGTH:
            status |= UART2_TRANSFER_STATUS_TX_EMPTY;
            break;
        default:
            break;
    }

    return status;    
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
uint8_t __attribute__((deprecated)) UART2_Peek(uint16_t offset)
{
    uint8_t *peek = rxHead + offset;
    
    while(peek > (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH))
    {
        peek -= UART2_CONFIG_RX_BYTEQ_LENGTH;
    }
    
    return *peek;
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
bool __attribute__((deprecated)) UART2_ReceiveBufferIsEmpty (void)
{
    return (UART2_RxDataAvailable() == 0);
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
bool __attribute__((deprecated)) UART2_TransmitBufferIsFull (void)
{
    return (UART2_TxDataAvailable() == 0);
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
UART2_STATUS __attribute__((deprecated)) UART2_StatusGet (void )
{
    return U2STA;
}

