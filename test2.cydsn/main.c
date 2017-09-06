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
uint16 RED[3] = {205, 930, 210};
uint16 GRE[3] = {445, 240, 345};
uint16 BLU[3] = {830, 450, 218};


uint16 capturedCount = 0u; // counter value when capture occurs
uint16 overflowCount = 0u; // counter overflows between captures
uint8 capflag = FALSE;     // set by ISR to tell main when capture occurs

uint8 echoitr = FALSE;
uint8 echoflag = FALSE;
uint16 echolength = 0u;
uint16 distance = 0u;


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

CY_ISR_PROTO(echo_isr) {
    // IMPORTANT: this code assumes there's no overflows / tc
    uint8 echo_of; //overflow
    
    // check whether rising or falling edge
    // if falling then read capture value
    // and set flag for main
    
    
    uint32 counter_status = 0u;
    counter_status = Counter_2_GetInterruptSource(); // clear interrupt
    
    if ((counter_status & Counter_1_STATUS_OVERFLOW) == Counter_1_STATUS_OVERFLOW) {
        echoflag = TRUE;
    }
    if (!echoitr) {
        // if first capture interrupt
        // don't really do anything i guess
        echoitr = !echoitr;
    } else {
        // must be a falling edge?
        echolength = Counter_2_ReadCapture();
        echoitr = !echoitr;
        echoflag = TRUE; // tell main that capture's been made
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

void printNumUART(uint32 num) {
    uint16 distLen;
    char8 strHex[10u];
    int i;
    
    distLen = Hex2Dec_Str(strHex, num);
    for (i=distLen; i>0; i--) {
        UART_1_PutChar(strHex[i-1]);
    }
}


void getColour(uint16* periodLen, int* col, uint16* dist) {
    // initialise
    uint16 rCount=10000;
    uint16 gCount=10000;
    uint16 bCount=10000;
    uint16 rCountTmp, gCountTmp, bCountTmp;
    uint16 rDist, gDist, bDist;
    int temp, min;
    int rep = 12;
    
    // cycle through colours
    S2_Write(0); S3_Write(0); // RED
        for (int i=0;i<rep;i++) {
            CyDelay(10);
            rCountTmp = (overflowCount * *periodLen) + capturedCount;
            rCount = (rCount < rCountTmp) ? rCount : rCountTmp;
        }
        overflowCount = 0u;
    
    S2_Write(1); S3_Write(1); // GREEN
        for (int i=0;i<rep;i++) {
            CyDelay(10);
            gCountTmp = (overflowCount * *periodLen) + capturedCount;
            gCount = (gCount < gCountTmp) ? gCount : gCountTmp;
        }
        overflowCount = 0u;
        
    S2_Write(0); S3_Write(1); // BLUE
        for (int i=0;i<rep;i++) {
            CyDelay(10);
            bCountTmp = (overflowCount * *periodLen) + capturedCount;
            bCount = (bCount < bCountTmp) ? bCount : bCountTmp;
        }
        overflowCount = 0u;
    
    // print measures
    UART_1_PutString("\n\nSens: R");
    printNumUART(rCount);
    UART_1_PutString(" G");
    printNumUART(gCount);
    UART_1_PutString(" B");
    printNumUART(bCount);
    
    
    // find L1 distance to each puck location
    rDist = abs(RED[0]-rCount) + abs(RED[1]-gCount) + abs(RED[2]-bCount);
        UART_1_PutString("\nDist: R");
        printNumUART(rDist);
    gDist = abs(GRE[0]-rCount) + abs(GRE[1]-gCount) + abs(GRE[2]-bCount);
        UART_1_PutString(" G");
        printNumUART(gDist);
    bDist = abs(BLU[0]-rCount) + abs(BLU[1]-gCount) + abs(BLU[2]-bCount);
        UART_1_PutString(" B");
        printNumUART(bDist);
    
    // which is lowest i guess?
    temp = (rDist < gDist) ? rDist : gDist;
    *dist = (bDist < temp) ? bDist : temp;
    min = (bDist < temp) ? 3 : 2;
    if (min==2) {
        min = (rDist < gDist) ? 1 : 2;
    }
    UART_1_PutString("\n");
    printNumUART(*dist);
    if (*dist<500) {
        *col = min;
    } else {
        col = 0;
    }
}

void getDist(uint16* echolength, uint16* distance) {
    uint16 counter = 0u;
    
    // send trigger; register is set to PULSE mode so will automatically reset after one clock period
    // clock freq = 100kHz so trigger should be set for 10us
    TRIG_CTRL_Write(1);
    // wait for echo complete flag
    while (echoflag==FALSE) {
        CyDelay(1);
        counter++;
        
        if (counter % 0xfff == 0) {
            UART_1_PutString("\nWaiting...");
        }
        
        if (counter == 0xffff) {
            UART_1_PutString("\nTimed out.");
            echoflag = TRUE;
        }
    }
    // convert to dist
    echoflag = FALSE;
    
    *distance = *echolength / 58;
}

void monitorColour(void) {
    /* Place your application code here. */

    // initialise variables
    uint16 periodLen = 0u;   // length of counter period
    uint16 dist;
    int col;
    
    S0_Write(1); // 20% scaling?
    S1_Write(0);
    
    periodLen = Counter_1_ReadPeriod(); // read length of period register (in counts)
    
    for(;;) {
        getColour(&periodLen, &col, &dist);
        //getDist(&echolength, &distance);
        
        
        UART_1_PutString("\nColour: ");
        if (col==1) {
            UART_1_PutString("R ");
        } else if (col==2) {
            UART_1_PutString("G ");
        } else if (col==3) {
            UART_1_PutString("B ");
        } else if (col==0) {
            UART_1_PutString("None ");
        }
        
    }
}

void moveServo(int16 angle) {
    /* INPUTS
    angle = value from -90 to 90 (degrees)
    */
    
    uint16 duty;
    
    duty = 4500 + 31*angle; // range from 3k to 6k duty cycle
                            // pwm period = 60k, 20ms so this corresponds to
                            // 1ms to 2ms range
    PWM_SERVO_WriteCompare(duty);
    UART_1_PutString("\nduty  ");
    printNumUART(duty);
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    
    
    // initialise components
    UART_1_Start();
    UART_1_PutString("UART started\n");
    Counter_1_Start();
    PWM_SERVO_Start();
    
    isr_counter_StartEx(counter_isr); // set the isr_counter code to the proto code above
    
    
    for(;;)
    {
        moveServo(-20);
        CyDelay(2000);
        moveServo(0);
        CyDelay(500);
        
        moveServo(20);
        CyDelay(2000);
        moveServo(0);
        CyDelay(500);
        
    }
}


/* [] END OF FILE */
