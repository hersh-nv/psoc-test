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
#include "stdlib.h"

/* Defines for TRUE and FALSE */
#ifndef TRUE
    #define TRUE 1
#endif
#ifndef FALSE
    #define FALSE 0
#endif

/* Defines for puck readings */
int RED[3] = {180, 350, 275};
int GRE[3] = {245, 215, 220};
int BLU[3] = {260, 260, 170};


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
    uint16 rCount, gCount, bCount;
    uint16 rDist, gDist, bDist;
    int temp, min;
    
    // cycle through colours
    S2_Write(0); S3_Write(0); // RED
            CyDelay(50);
            rCount = (overflowCount * periodLen) + capturedCount;
            overflowCount = 0u;
            
    S2_Write(1); S3_Write(1); // GREEN
            CyDelay(50);
            gCount = (overflowCount * periodLen) + capturedCount;
            overflowCount = 0u;
    
    S2_Write(0); S3_Write(1); // BLUE
            CyDelay(50);
            bCount = (overflowCount * periodLen) + capturedCount;
            overflowCount = 0u;
    
    // find L1 distance to each puck location
    rDist = abs(RED[0]-rCount) + abs(RED[1]-gCount) + abs(RED[2]-bCount);
    gDist = abs(GRE[0]-rCount) + abs(GRE[1]-gCount) + abs(GRE[2]-bCount);
    bDist = abs(BLU[0]-rCount) + abs(BLU[1]-gCount) + abs(BLU[2]-bCount);
    
    // which is lowest i guess?
    temp = (rDist < gDist) ? rDist : gDist;
    min = (bDist < temp) ? 2 : 1;
    if (min==1) {
        min = (rDist < gDist) ? 0 : 1;
    }
    return min;
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    // initialise variables
    uint16 periodLen = 0u;   // length of counter period
    int8 col;
    
    
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
        
        col = getColour(periodLen);
        
        UART_1_PutString("\nColour: ");
        if (col==0) {
            UART_1_PutString("R");
        } else if (col==1) {
            UART_1_PutString("G");
        } else {
            UART_1_PutString("B");
        }
          
        CyDelay(500); // this may be unnecessary idk
        
    }
}


/* [] END OF FILE */
