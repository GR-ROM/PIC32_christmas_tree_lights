/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC32MX MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC32MX MCUs - pic32mx : v1.35
        Device            :  PIC32MX270F256D
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC32 1.42
        MPLAB             :  MPLAB X 3.55
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

#include "mcc_generated_files/mcc.h"

/*
                         Main application
 */

#define DELAY_LEN 48
#define NUM_LEDS 50
#define DMA_BUF_LEN NUM_LEDS*24+DELAY_LEN
#define LOW 0x14
#define HIGH 0x28

uint8_t dma_buf[DMA_BUF_LEN];
volatile int transfer_done;

void init_ws2812b(){
    int i;
    for (i=0;i!=DELAY_LEN;i++) dma_buf[i]=0;    
	for (i=DELAY_LEN;i!=DMA_BUF_LEN;i++){
		dma_buf[i]=LOW;
	}
}

void rgb_to_dma_ws2812b(uint8_t R, uint8_t G, uint8_t B, uint8_t start_led, uint8_t led_num){
	uint8_t t;
    int b, i;
	for (i=start_led;i!=led_num;i++){
		t=G;
		uint8_t c=0;
		for (b=0;b!=8;b++){
			if (t & 0x80)	dma_buf[DELAY_LEN+(i*24)+(c*8)+b]=HIGH; else dma_buf[DELAY_LEN+(i*24)+(c*8)+b]=LOW;
			t<<=1;
		}
		t=R;
		c++;
		for (b=0;b!=8;b++){
			if (t & 0x80)	dma_buf[DELAY_LEN+(i*24)+(c*8)+b]=HIGH; else dma_buf[DELAY_LEN+(i*24)+(c*8)+b]=LOW;
			t<<=1;
		}
		t=B;
		c++;
		for (b=0;b!=8;b++){
			if (t & 0x80)	dma_buf[DELAY_LEN+(i*24)+(c*8)+b]=HIGH; else dma_buf[DELAY_LEN+(i*24)+(c*8)+b]=LOW;
			t<<=1;
		}
	}
}


void __ISR(_TIMER_2_VECTOR, IPL1AUTO) _T2Interrupt (  ){
    IFS0CLR= 1 << _IFS0_T2IF_POSITION;
}

void __ISR(_DMA0_VECTOR, ipl1AUTO) _IntHandlerSysDmaCh0(void){
    if(DCH0INTbits.CHBCIF) {
        DCH0INTbits.CHBCIF=0;
        IFS0bits.T2IF = 0;
        T2CONCLR = 0x8000;
        OC1RS = 0x00;
        LATBbits.LATB3=0;
        transfer_done = 1;
    }
    IFS1CLR= 1 << _IFS1_DMA0IF_POSITION;
    
}

#define PERIOD 0x3e

  void setupdmaxfer(){
    IEC1bits.DMA0IE = 1;
    IFS1bits.DMA0IF = 0;
    IFS1CLR= 1 << _IFS1_DMA0IF_POSITION;
    DMACONSET  = 0x8000;                                       // enable DMA.
    DCH0CON = 0x0000;
    DCRCCON = 0x00;                                            //   
    DCH0INTCLR = 0xff00ff;                                     // clear DMA interrupts register.
    DCH0INTbits.CHBCIE = 1;                                    // DMA Interrupts when channel block transfer complete enabled.
    DCH0ECON = 0x00;
    //DCH0INTbits.CHSDIE=1;
    DCH0ECON = 0x00;
    DCH0SSA = ((unsigned int)&dma_buf) & 0x1FFFFFFF;
    DCH0DSA = ((unsigned int)&OC1RS) & 0x1FFFFFFF;
    DCH0SSIZ = DMA_BUF_LEN+1;                 // DMA Source size (default).
    DCH0DSIZ = 1;                            // DMA destination size.
    DCH0CSIZ = 1;                            // DMA cell size.
    DCH0ECONbits.CHSIRQ = _TIMER_2_IRQ;      // DMA transfer triggered by which interrupt? (On PIC32MX - it is by _IRQ suffix!)
    DCH0ECONbits.AIRQEN = 0;                 // do not enable DMA transfer abort interrupt.
    DCH0ECONbits.SIRQEN = 1;                 // enable DMA transfer start interrupt.
    
    DCH0CONbits.CHAEN = 0;                  // DMA Channel 0 is always disabled right after the transfer.
    
    OC1CONSET = 0x8006;
    T2CONCLR  = 0x8000;
    PR2 = PERIOD;
}
 
   
int main(void) {
    // initialize the device
    ANSELA = 0;  // All port A as digital
	ANSELB = 0;  // All port B as digital
	TRISA = 0;
    TRISB = 0;
    TRISC = 0;
    transfer_done=0;
    SYSTEM_Initialize();    
    IPC10bits.DMA0IP = 1;
    IPC10bits.DMA0IS = 0;
    int i;    int l=0;
      
    init_ws2812b();
    setupdmaxfer();      
    uint8_t col=0;
    uint8_t r=8;
    uint8_t g=8;
    uint8_t b=8;
    l=0;
    int dir=1;
    while (1) {
        l++;
        if (l==250000){          
            l=0;
            
            for (i=0;i!=DMA_BUF_LEN;i++) dma_buf[i]=0;    
            rgb_to_dma_ws2812b(r, g, b, 0, NUM_LEDS);
            if (col==0){
                r=dir ? r+1 : r-1;
                g=0; b=0;
            }
            if (col==1){
                r=dir ? r+1 : r-1;
                g=dir ? g+1 : g-1;
                b=0;
            }
            if (col==2){
                g=dir ? g+1 : g-1;
                r=0;
                b=0;
            }
            if (col==3){
                g=dir ? g+1 : g-1;
                b=dir ? b+1 : b-1;
                r=0;
            }
            if (col==4){
                b=dir ? b+1 : b-1;
                r=0;
                g=0;
            }
            if (col==5){
                b=dir ? b+1 : b-1;
                r=dir ? r+1 : r-1;
                g=0;
            }
            
            
            if ((r==1 || g==1 || b==1) && dir==0){ dir=1; if (++col==6) col=0; }
            if (r==32 || g==32 || b==32) dir=0;
            
            DCH0CONbits.CHEN = 1;                   
            DCH0ECONbits.CFORCE = 1;
            TMR2 = 0x0000;
            T2CONSET = 0x8000;
            while(!transfer_done);
            transfer_done = 0;       
        }
    }

    return -1;
}
/**
 End of File
*/