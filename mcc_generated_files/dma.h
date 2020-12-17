/*******************************************************************************
  DMA Generated Driver API Header File

  Company:
    Microchip Technology Inc.

  File Name:
    dma.h

  Summary:
    This is the generated header file for the DMA driver using PIC32MX MCUs

  Description:
    This header file provides APIs for driver for DMA.
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

#ifndef DMA_H
#define DMA_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/attribs.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
        
/**
  Section: Data Types
*/

/** DMA Channel Definition
 
 @Summary 
   Defines the channels available for DMA
 
 @Description
   This routine defines the channels that are available for the module to use.
 
 Remarks:
   None
 */
typedef enum 
{
    DMA_CHANNEL_0 =  0,       
    DMA_NUMBER_OF_CHANNELS = 4
} DMA_CHANNEL;
/**
  Section: Interface Routines
*/

/**
  @Summary
    This function initializes DMA instance : 

  @Description
    This routine initializes the DMA driver instance for : 
    index, making it ready for clients to open and use it. It also initializes any
    internal data structures.
    This routine must be called before any other DMA routine is called. 

  @Preconditions
    None.

  @Param
    None.

  @Returns
    None.

  @Comment
    
 
  @Example
    <code>
        unsigned short int srcArray[100];
        unsigned short int dstArray[100];
        int i;
        int count;
        for (i=0; i<100; i++)
        {
            srcArray[i] = i+1;
            dstArray[i] = 0;
        }
        
        DMA_Initialize();
        // To transfer data between two unknown address 
        DMA_SourceAddressSet(CHANNEL, &srcArray[0]);
        DMA_DestinationAddressSet(CHANNEL, &dstArray[0]);
        DMA_SourceSizeSet(CHANNEL, 100);
        DMA_DestinationSizeSet(CHANNEL, 100);
        DMA_TransferCountSet(CHANNEL, 10);
        for(i=0;i<10;i++)
        {
            DMA_SoftwareTriggerEnable(CHANNEL);
        }
    </code>

*/
void DMA_Initialize(void);

/**
  @Summary
    Sets the transfer count of DMA.

  @Description
    This routine is used to set the DMA transfer count. This routine
    sets the value of the DCHxCSIZ register. 
 
  @Preconditions
    DMA_Initialize() function should have been 
    called before calling this function.
 
  @Returns
    None

  @Param
    channel - Identifies the DMA channel.
    count   - This contains number of bytes to be transmitted per cell transfer
  
  @Example
    Refer to DMA_Initialize() for an example
 */
void DMA_TransferCountSet(DMA_CHANNEL channel, uint16_t count);

/**
  @Summary
    Enables the software trigger of the DMA

  @Description
    This routine is used to enable the software trigger of the DMA. This routine
    sets the value of the CFORCE bit to 1.
 
  @Preconditions
    DMA_Initialize() function should have been 
    called before calling this function.
 
  @Returns
    None

  @Param
    channel - Identifies the DMA channel.
  
  @Example
    Refer to DMA_Initialize() for an example
 */
void DMA_SoftwareTriggerEnable(DMA_CHANNEL channel);

/**
  @Summary
    Sets the source address for the DMA

  @Description
    This routine is used to set the source address for a DMA channel. 
 
  @Preconditions
    DMA_Initialize() function should have been 
    called before calling this function.
 
  @Returns
    None

  @Param
    channel - Identifies the DMA channel.
    address - Contains the DMA channel source address.

  @Example
    Refer to DMA_Initialize() for an example
 */
void DMA_SourceAddressSet(DMA_CHANNEL channel, uint32_t address);

/**
  @Summary
    Sets the destination address for the DMA

  @Description
    This routine is used to set the destination address for a DMA channel. 
 
  @Preconditions
    DMA_Initialize() function should have been 
    called before calling this function.
 
  @Returns
    None

  @Param
    channel - Identifies the DMA channel.
    address - Contains the DMA channel destination address.
  
  @Example
    Refer to DMA_Initialize() for an example
 */
void DMA_DestinationAddressSet(DMA_CHANNEL channel, uint32_t address);

/**
  @Summary
    Sets the DMA channel destination size.

  @Description
    This routine is used to set the DMA channel destination size. 
 
  @Preconditions
    DMA_Initialize() function should have been 
    called before calling this function.
 
  @Returns
    None

  @Param
    channel - Identifies the DMA channel.
    dstSize - Contains the DMA channel destination size.
  
  @Example
    Refer to DMA_Initialize() for an example
 */
void DMA_DestinationSizeSet(DMA_CHANNEL channel, uint16_t dstSize);

/**
  @Summary
    Sets the DMA channel source size.

  @Description
    This routine is used to set the DMA channel source size. 
 
  @Preconditions
    DMA_Initialize() function should have been 
    called before calling this function.
 
  @Returns
    None

  @Param
    channel - Identifies the DMA channel.
    sourceSize - Contains the DMA channel source size.
  
  @Example
    Refer to DMA_Initialize() for an example
 */
void DMA_SourceSizeSet(DMA_CHANNEL channel, uint16_t sourceSize);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
    
#endif  // DMA.h