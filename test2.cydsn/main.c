/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"

/* Defines for TRUE and FALSE */
#ifndef TRUE
    #define TRUE 1
#endif
#ifndef FALSE
    #define FALSE 0
#endif


uint16 capturedCount = 0u; // counter value when capture occurs
uint16 overflowCount = 0u; // counter overflows between captures
uint8 capflag = FALSE;     // set by ISR to tell main when capture occurs


/* ISR code : unsure if compiler will use this correctly */
CY_ISR_PROTO(counter_isr) {
    // read and store counter status register
    // determine if interrupt is capture or overflow
    uint32 counter_status = 0u;
    
    // this is a dupe of ReadStatusRegister() therefore clears the interrupt afaik
    counter_status = Counter_1_GetInterruptSource();
    
    // mask status register w pre-defd mask bits
    // these masks come from <counter_1.h>
    if ((counter_status & Counter_1_STATUS_OVERFLOW) == Counter_1_STATUS_OVERFLOW) {
        // interrupt is due to overflow
        overflowCount++;
    }
    
    if ((counter_status & Counter_1_STATUS_CAPTURE) == Counter_1_STATUS_CAPTURE) {
        // interrupt is due to capture
        capturedCount = Counter_1_ReadCapture();
        capflag = TRUE;
    }
}



/* this is ripped char for char from TimeStamp_Measurement example from cypress */
uint32 Hex2Dec_Str(char8 str[], uint32 hex) {
    uint32 i, hex_tmp;
    uint32 res;
    uint32 len;

    /* The max length is 10 */
    len = 10u;
    res = hex;
    for (i = 0u; i < 10u; i++) {
        hex_tmp = res%10u; 
        str[i] = hex_tmp + '0';

        res /= 10u; 
        if (0u == res) {
            len = i + 1u;
            break;
        }    
    }
    return len;
}

int getColour(uint16 periodLen) {
    // initialise
    uint32 rCount, gCount, bCount;
    
    // cycle through colours
    S2_Write(0); S3_Write(0); // RED
            CyDelay(50);
            rCount = (overflowCount * periodLen) + capturedCount;
            overflowCount = 0u;
            
    return 0;
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    // initialise variables
    //uint16 capCount = 0u; // unused
    uint16 periodLen = 0u;   // length of counter period
    uint32 measuredCount = 0u; // counts between capture rising edges
    char8 strHex[10u]; // to store the decimal count as a string
    uint32 lenDec, i;
    
    
    // initialise components
    UART_1_Start();
    Counter_1_Start();
    isr_counter_StartEx(counter_isr); // set the isr_counter code to the proto code above
           
    S0_Write(1); // 20% scaling?
    S1_Write(0);
    
    periodLen = Counter_1_ReadPeriod(); // read length of period register (in counts)
    
    for(;;)
    {
        /* Place your application code here. */
        //UART_1_WriteTxData('s'); // idk
        
//        if (capflag == TRUE) {
//            // if capture has occurred
//            // calculate period count
//            capCount = capturedCount; // i suppose this is being duplicated so its not interrupted by isr? 
//            measuredCount = (overflowCount * periodLen) + capCount;
//            overflowCount = 0u;
//            
//            // convert to period time i guess? may not be necessary
//            
//            // send to UART
//            UART_1_PutString("\nR: ");
//            lenDec = Hex2Dec_Str(strHex, measuredCount);
//            for (i=lenDec; i>0; i--) {
//                UART_1_PutChar(strHex[i-1]);
//            }
//    
//            capflag = FALSE; // clear flag
//        }
        
        S2_Write(0); S3_Write(0); // RED
            CyDelay(100);
            measuredCount = (overflowCount * periodLen) + capturedCount;
            overflowCount = 0u;
            // send to UART            
            UART_1_PutString("\nR: ");
            lenDec = Hex2Dec_Str(strHex, measuredCount);
            for (i=lenDec; i>0; i--) {
                UART_1_PutChar(strHex[i-1]);
            }
        
        S2_Write(1); S3_Write(1); // GREEN
            CyDelay(100);
            measuredCount = (overflowCount * periodLen) + capturedCount;
            overflowCount = 0u;
            // send to UART
            UART_1_PutString("  G: ");
            lenDec = Hex2Dec_Str(strHex, measuredCount);
            for (i=lenDec; i>0; i--) {
                UART_1_PutChar(strHex[i-1]);
            }
        
        S2_Write(0); S3_Write(1); // BLUE
            CyDelay(100);
            measuredCount = (overflowCount * periodLen) + capturedCount;
            overflowCount = 0u;
            // send to UART
            UART_1_PutString("  B: ");
            lenDec = Hex2Dec_Str(strHex, measuredCount);
            for (i=lenDec; i>0; i--) {
                UART_1_PutChar(strHex[i-1]);
            }
        
        S2_Write(1); S3_Write(0); // CLEAR
            CyDelay(100);
            measuredCount = (overflowCount * periodLen) + capturedCount;
            overflowCount = 0u;
            // send to UART
            UART_1_PutString("  C: ");
            lenDec = Hex2Dec_Str(strHex, measuredCount);
            for (i=lenDec; i>0; i--) {
                UART_1_PutChar(strHex[i-1]);
            }
        
        CyDelay(500); // this may be unnecessary idk
        
    }
}


/* [] END OF FILE */
