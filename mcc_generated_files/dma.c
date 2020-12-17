/*******************************************************************************
  DMA Generated Driver File

  Company:
    Microchip Technology Inc.

  File Name:
    dma.c

  Summary:
    This is the generated driver implementation file for the DMA driver using PIC32MX MCUs

  Description:
    This source file provides implementations for driver APIs for DMA.
    Generation Information :
        Product Revision  :  PIC32MX MCUs - pic32mx : v1.35
        Device            :  PIC32MX270F256D
        Version           :  1.0
    The generated drivers are tested against the following:
        Compiler          :  XC32 1.42
        MPLAB             :  MPLAB X 3.55
 *******************************************************************************/

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

#include <xc.h>
#include "dma.h"
/**
  Prototype:        void DMA_Initialize(void)
  Input:            none
  Output:           none
  Description:      DMA_Initialize is an
                    initialization routine that takes inputs from the GUI.
  Comment:          
  Usage:            DMA_Initialize();
 */
void DMA_Initialize(void) 
{ 
    // DMABUSY disabled; SUSPEND Normal DMA operation; ON enabled; 
    DMACON = 0x8000;
    // CHSDIE enabled; CHSDIF No interrupt; CHERIF No interrupt; CHSHIE disabled; CHSHIF No interrupt; CHERIE enabled; CHCCIF No interrupt; CHBCIE disabled; CHBCIF No interrupt; CHDDIF No interrupt; CHDDIE enabled; CHCCIE disabled; CHDHIF No interrupt; CHDHIE disabled; CHTAIE disabled; CHTAIF No interrupt; 
    DCH0INT= 0xA10000;
    // CHSSA 0; 
    DCH0SSA= 0x0;
    // CHDSA 0; 
    DCH0DSA= 0x0;
    // CHSSIZ 0; 
    DCH0SSIZ= 0x0;
    // CHDSIZ 0; 
    DCH0DSIZ= 0x0;
    // CHCSIZ 1; 
    DCH0CSIZ= 0x1;
    // DCHPDAT 0; 
    DCH0DAT= 0x0;
    // Clearing Channel 0 Interrupt Flag
    IFS1CLR= 1 << _IFS1_DMA0IF_POSITION;
    // Enable Channel 0 Interrupt
    IEC1bits.DMA0IE = true;
    // CHCHNS Chain to channel higher; CHAEN disabled; CHEN enabled; CHCHN disabled; CHBUSY disabled; CHAED disabled; CHPRI Priority 0; 
    DCH0CON= 0x80;
    // AIRQEN CHAIRQ ignored; CABORT disabled; SIRQEN CHSIRQ ignored; CHSIRQ 0; CHAIRQ 0; PATEN disabled; CFORCE disabled; 
    DCH0ECON= 0x0;

}




void DMA_TransferCountSet(DMA_CHANNEL channel, uint16_t count)
{
    switch(channel) {
        case DMA_CHANNEL_0:
           DCH0CSIZ = count;
        default: break;
    }
}

void DMA_SoftwareTriggerEnable(DMA_CHANNEL channel )
{
    switch(channel) {
        case DMA_CHANNEL_0:
           DCH0ECONbits.CFORCE = 1;
           break;
        default: break;
    }
}

void DMA_SourceAddressSet(DMA_CHANNEL  channel, uint32_t address) {
    switch(channel) {
        case DMA_CHANNEL_0:
           // Convert virtual address to physical address
           DCH0SSA = (address & 0x1FFFFFFF);
           break;
        default: break;
    }    
}

void DMA_DestinationAddressSet(DMA_CHANNEL  channel, uint32_t address) {
    switch(channel) {
        case DMA_CHANNEL_0:
           // Convert virtual address to physical address
           DCH0DSA = (address & 0x1FFFFFFF);
           break;
        default: break;
    }    
}

void DMA_SourceSizeSet(DMA_CHANNEL  channel, uint16_t sourceSize) {
    switch(channel) {
        case DMA_CHANNEL_0:
           DCH0SSIZ = sourceSize;
           break;
        default: break;
    }    
}

void DMA_DestinationSizeSet(DMA_CHANNEL  channel, uint16_t dstSize) {
    switch(channel) {
        case DMA_CHANNEL_0:
           DCH0DSIZ = dstSize;
           break;
        default: break;
    }    
}

