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

// initialise variables
uint16 overflowCount = 0u;


/* ========== FUNCTIONS START HERE ===================== */

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

void printNumUART(uint32 num) {
    uint16 distLen;
    char8 strHex[10u];
    int i;
    
    distLen = Hex2Dec_Str(strHex, num);
    for (i=distLen; i>0; i--) {
        UART_1_PutChar(strHex[i-1]);
    }
}

/* ISR for shaft encoder */
CY_ISR_PROTO(QUAD_ISR) {
    /* isr every time quad encoder overflow occurs */
    uint32 counter_status = 0u;
    UART_1_PutString("\nInterrupt occurred!");
    // double check that the counter is from overflow
    // if so, increment global overflow counter
    if ((counter_status & SE_QUAD_COUNTER_OVERFLOW) == SE_QUAD_COUNTER_OVERFLOW) {
        // interrupt is due to overflow
        UART_1_PutString("\n...because of overflow");
        overflowCount++;
    }
}

/* Reads the quadrature decoder connected to the shaft encoder and returns the distance as a uint32 */
uint32 getDistanceSE(void) {
    
    uint32 distance;
    uint16 SE_COUNT = 0u;
    SE_COUNT = SE_QUAD_GetCounter();
    
    distance = overflowCount * 0xffff + SE_COUNT;
    
    // TODO: convert to real life distance units instead of arbitrary count value
    
    return distance;
}


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* DRIVE FOR 10s WHILE MEASURING DISTANCE */
    /* ======================= */
    
    UART_1_Start();
    PWM_1_Start();
    SE_QUAD_Start();
    ISR_QUAD_StartEx(QUAD_ISR);

    
    uint32 dist = 0u;
    
    // enable
    PWM_1_Enable();
    
    // 50% speed
    PWM_1_WriteCompare(127);
    
    // direction A
    A1_Write(1);
    A2_Write(0);
    CyDelay(5000); // drive for 10 seconds
    
    // stop
    A1_Write(0);
    A2_Write(0);
    
    // get SE reading
    dist = getDistanceSE();
    
    // print to UART
    UART_1_PutString("\nShaft encoder decoded output: ");
    printNumUART(dist);
    
    UART_1_PutString("\nOverflow count: ");
    printNumUART(overflowCount);
    
    
    for(;;)
    {

    }
}

/* [] END OF FILE */
