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

// initialise global variables
int16 overflowCountL = 0u;
int16 overflowCountR = 0u;


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

/* ISR for shaft encoder 1 -- identical to next ISR */
CY_ISR_PROTO(QUAD1_ISR) {
    
    uint32 counter_status = 0u;
    UART_1_PutString("\nQuadDec interrupt occurred");
    
    counter_status = SEL_QUAD_GetEvents();
    // check that the counter is from overflow
    // if so, increment global overflow counter
    if ((counter_status & SEL_QUAD_COUNTER_OVERFLOW) == SEL_QUAD_COUNTER_OVERFLOW) {
        UART_1_PutString("...because of overflow");
        overflowCountL++;
    }
    
    // check other interrupt causes
    if ((counter_status & SEL_QUAD_COUNTER_UNDERFLOW) == SEL_QUAD_COUNTER_UNDERFLOW) {
        UART_1_PutString("...because of underflow");
        overflowCountL--;
    }
    
    if ((counter_status & SEL_QUAD_COUNTER_RESET) == SEL_QUAD_COUNTER_RESET) {
        UART_1_PutString("...because of reset");
    }
    
    if ((counter_status & SEL_QUAD_INVALID_IN) == SEL_QUAD_INVALID_IN) {
        UART_1_PutString("...because of invalid input transition");
    }
}
CY_ISR_PROTO(QUAD2_ISR) {
    
    uint32 counter_status = 0u;
    UART_1_PutString("\nQuadDec interrupt occurred");
    
    counter_status = SER_QUAD_GetEvents();
    // check that the counter is from overflow
    // if so, increment global overflow counter
    if ((counter_status & SER_QUAD_COUNTER_OVERFLOW) == SER_QUAD_COUNTER_OVERFLOW) {
        UART_1_PutString("...because of overflow");
        overflowCountR++;
    }
    
    // check other interrupt causes
    if ((counter_status & SER_QUAD_COUNTER_UNDERFLOW) == SER_QUAD_COUNTER_UNDERFLOW) {
        UART_1_PutString("...because of underflow");
    }
    
    if ((counter_status & SER_QUAD_COUNTER_RESET) == SER_QUAD_COUNTER_RESET) {
        UART_1_PutString("...because of reset");
    }
    
    if ((counter_status & SER_QUAD_INVALID_IN) == SER_QUAD_INVALID_IN) {
        UART_1_PutString("...because of invalid input transition");
    }
}


/* Reads the quadrature decoder connected to the shaft encoder and returns the distance as a int32 */
/* TODO: reset distance after reading ? */
int32 getDistance(int side, int32 startdist) {
    
    int32 distance;
    int16 SE_COUNT = 0u;
    if (side==1) {
        SE_COUNT = SEL_QUAD_GetCounter();
        distance = overflowCountL * 0x7fff + SE_COUNT;
        //SEL_RST_Write(0);
    } else {
        SE_COUNT = SER_QUAD_GetCounter();
        distance = overflowCountR * 0x7fff + SE_COUNT;
        //SER_RST_Write(0);
    }
    
    distance = distance-startdist;
    
    // TODO: convert to real life distance units instead of arbitrary count value
    
    return distance;
}


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* DRIVE FOR 10s WHILE MEASURING DISTANCE */
    /* ======================= */
    
    // start components and ISRs
    UART_1_Start();
    PWM_1_Start();
    SEL_QUAD_Start();
    SER_QUAD_Start();
    ISR_QUAD1_StartEx(QUAD1_ISR);
    ISR_QUAD2_StartEx(QUAD2_ISR);

    // initialise main variables
    uint32 ldist = 0u;
    uint32 rdist = 0u;
    uint32 lsdist,rsdist;
    
    // enable
    PWM_1_Enable();
    
    // 25% speed
    PWM_1_WriteCompare1(63);
    PWM_1_WriteCompare2(63);
            

    
    
    for(int i=0; i<10; i++)
    {   
        // start measuring distance
        lsdist = getDistance(1,0); //left
        rsdist = getDistance(0,0); //right
        
        // forward direction
        A1_Write(0);
        A2_Write(1);
        A3_Write(0);
        A4_Write(1);
        CyDelay(2000); // drive for X seconds

        // stop
        A1_Write(0);
        A2_Write(0);
        A3_Write(0);
        A4_Write(0); 
        CyDelay(500); // pause for X seconds
        
        // get distance from both wheels
        ldist = getDistance(1,lsdist); //left
        rdist = getDistance(0,rsdist); //right
        
        // print to UART
        UART_1_PutString("\nLdist ");
        printNumUART(ldist);
        UART_1_PutString("  Rdist ");
        printNumUART(rdist);
    }
    
    
}

/* [] END OF FILE */
