/* ========================================
DRIVING TESTS

todo:
    - 
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
uint16 RED[3] = {180, 300, 250};
uint16 GRE[3] = {245, 215, 220};
uint16 BLU[3] = {260, 260, 170};


uint16 capturedCount = 0u; // counter value when capture occurs
uint16 overflowCount = 0u; // counter overflows between captures
uint8 capflag = FALSE;     // set by ISR to tell main when capture occurs

// defines
#define DIST_COEFF 157
/*
conversion coefficient from distance in cm to quadrature encoder counter
i.e. dist * QUAD_COEFF = value that QuadDec should count up to to travel dist in cm.
200 is a roughly correct value i just measured; can be adjusted for preciseness later
*/
#define ROTATE_COEFF 40 // this is a complete estimate; test laterer

// initialise global variables
int16 overflowCountL = 0u; // NB can go negative for underflow (when moving backward)
int16 overflowCountR = 0u;


/* ========== FUNCTIONS START HERE ===================== */

/* these are ripped from TimeStamp_Measurement example from cypress */
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
void printNumUART(int32 num) {
    uint16 distLen;
    char8 strHex[10u];
    int i;
    
    if (num<0) {
        UART_1_PutString("-");
    }
    distLen = Hex2Dec_Str(strHex, abs(num));
    for (i=distLen; i>0; i--) {
        UART_1_PutChar(strHex[i-1]);
    }
}

/* ISR for colour sensor */
CY_ISR_PROTO(COL_ISR) {
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

/* ISR for shaft encoders -- identical, only differ in wheel value */
CY_ISR_PROTO(QUAD1_ISR) {
    
    uint32 counter_status = 0u;
    //UART_1_PutString("\nQuadDec interrupt occurred");
    
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
       // UART_1_PutString("...because of invalid input transition");
    }
}
CY_ISR_PROTO(QUAD2_ISR) {
    
    uint32 counter_status = 0u;
   // UART_1_PutString("\nQuadDec interrupt occurred");
    
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
        //UART_1_PutString("...because of invalid input transition");
    }
}

/* Reads the quadrature decoder connected to the shaft encoder and returns the distance as a int32 */
int32 getDistance(int side, int32 startdist) {
    
    int32 distance;
    int16 SE_COUNT = 0u;
    if (side==1) {
        SE_COUNT = SEL_QUAD_GetCounter();
        distance = overflowCountL * 0x7fff + SE_COUNT;
    } else {
        SE_COUNT = SER_QUAD_GetCounter();
        distance = overflowCountR * 0x7fff + SE_COUNT;
    }
    
    distance = distance-startdist;

    return distance;
}

/* Drives X distance (cm) forward/backward, polls shaft encoders every 50ms until X distance reached */
void driveXdist(int32 Xdist, int dir, uint8 speed) {
    /* 
    INPUTS
    Xdist = distance to move forward (cm)
    dir = binary direction; 1 = forward, 0 = backward
    speed (out of 255 max)
    */

    uint8 lspeed, rspeed;
    int32 ldist = 0;
    int32 rdist = 0;
    int32 lsdist,rsdist;
    int32 deltDist;
    
    int32 XdistSE = Xdist*DIST_COEFF; // Xdist converted to QuadDec counter value
    UART_1_PutString("\nMoving ");
    printNumUART(Xdist);
    UART_1_PutString(" cm = ");
    printNumUART(XdistSE);
    UART_1_PutString(" QuadDec count");
    
    int done=0;
    
    // start shaft encoders
    lsdist = (getDistance(1,0)); // get left starting dist
    rsdist = (getDistance(0,0)); // get right starting dist
    
    // set speed
    lspeed = speed;
    rspeed = speed+3;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    
    // set directions
    A3_Write(!dir); // R
    A4_Write(dir);
    A1_Write(!dir); // L
    A2_Write(dir);

    
    // poll SEs every 50ms, when both shaft encoders read X distance, stop
    while (!done) {

        CyDelay(20); // the smaller this value, the more often the SEs are polled and hence the more accurate the distance
        
        // get relative distance from both wheels
        ldist = (getDistance(1,lsdist)); //left
        rdist = (getDistance(0,rsdist)); //right
        
        UART_1_PutString("\nLdist = ");
        printNumUART(ldist);
        UART_1_PutString("  Rdist = ");
        printNumUART(rdist);
        
                // some kind of closed feedback here
                deltDist = ldist-rdist;
                lspeed = lspeed - (deltDist>>2); // adjust speed by difference/4, but...
                rspeed = rspeed + (deltDist>>2);
                //PWM_1_WriteCompare1(lspeed); // ..turned this feature off because it seems to drive straight anyway
                //PWM_1_WriteCompare2(rspeed);
        
        /* When either wheel reaches target distance, stop both */
        // todo?: turn off each motor individually when it reaches Xdist. for now it travels straight so maybe it doesn't matter
        if ((abs(ldist)>=XdistSE) || (abs(rdist)>=XdistSE)) {
            A1_Write(0);
            A2_Write(0);
            A3_Write(0);
            A4_Write(0);            
            PWM_1_WriteCompare1(0);
            PWM_1_WriteCompare2(0);
            done=1;
        }
    }            
}

/* Turns X degrees CW/CCW, polls shaft encoders every 50 ms until X degrees is reached.
very similar to driveXdist() */
void turnXdegrees(int16 Xdeg, int dir, uint8 speed) {
    /*
    INPUTS
    Xdeg = desired angle to rotate robot (degrees)
    dir = direction to rotate (1=clockwise, 0=anticlockwise)
    speed (out of 255 max)
    */

    uint8 lspeed, rspeed;
    int32 ldist, rdist, lsdist, rsdist;
    int32 XdegSE = Xdeg * ROTATE_COEFF;
    UART_1_PutString("\nTurning ");
    printNumUART(Xdeg);
    UART_1_PutString(" degrees = ");
    printNumUART(XdegSE);
    UART_1_PutString(" QuadDec count");

    int Ldone=0;
    int Rdone=0;
    
    // start shaft encoders
    lsdist = (getDistance(1,0)); // get left starting dist
    rsdist = (getDistance(0,0)); // get right starting dist
    
    // set speed -- backwards wheel moves faster
    lspeed = speed+dir*6;
    rspeed = speed+dir*6;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    
    // set direction
    A1_Write(!dir); // L
    A2_Write(dir);
    A3_Write(dir); // R
    A4_Write(!dir);

    // poll SEs every 50ms, stop when both wheels have rotated enough
    while ((!Ldone) || (!Rdone)) {

        CyDelay(10);

        // get relative distance from both wheels
        if (!Ldone) {
            ldist = (getDistance(1,lsdist)); //left
        }
        if (!Rdone) {
            rdist = (getDistance(0,rsdist)); //right
        }
        UART_1_PutString("\nLdist = ");
        printNumUART(ldist);
        UART_1_PutString("  Rdist = ");
        printNumUART(rdist);

        /* When either wheel reaches target distance, stop that wheel */
        if (abs(ldist)>=XdegSE) {
            A1_Write(0);
            A2_Write(0);
            Ldone=1;
        }
        if (abs(rdist)>=XdegSE) {
            A3_Write(0);
            A4_Write(0);
            Rdone=1;
        }
    }
}


void getColour(uint16* periodLen, int* col, uint16* dist) {
    
    // initialise
    uint16 rCount, gCount, bCount;
    uint16 rDist, gDist, bDist;
    int temp, min;
    
    // cycle through colours
    S2_Write(0); S3_Write(0); // RED
        //capflag = FALSE; while (capflag == FALSE) {
            CyDelay(10);
        //}
        rCount = (overflowCount * *periodLen) + capturedCount;
        overflowCount = 0u;
    
    S2_Write(1); S3_Write(1); // GREEN
        //capflag = FALSE; while (capflag == FALSE) {
            CyDelay(10);
        //}
        gCount = (overflowCount * *periodLen) + capturedCount;
        overflowCount = 0u;
        
    S2_Write(0); S3_Write(1); // BLUE
        //capflag = FALSE; while (capflag == FALSE) {
            CyDelay(10);
        //}
        bCount = (overflowCount * *periodLen) + capturedCount;
        overflowCount = 0u;
    
    // print measures
    UART_1_PutString("\nSens: R");
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
    *col = min;
}


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    // start components and ISRs
    UART_1_Start();
    UART_1_PutString("\nUART started");
    PWM_1_Start();
    SEL_QUAD_Start();
    SER_QUAD_Start();
    ISR_QUAD1_StartEx(QUAD1_ISR);
    ISR_QUAD2_StartEx(QUAD2_ISR);
    ISR_COL_StartEx(COL_ISR);
    
    // variables
    uint8 fwdspeed = 210;
    uint8 bwdspeed = 220;
    uint8 trnspeed = 100;
    
    uint16 periodLen = 0u;   // length of counter period
    uint16 dist;
    int col;
    
    // enable
    PWM_1_Enable();
    CyDelay(500);
    
//    // task 1
//    for (int i=0;i<1;i++) {
//        LED1_Write(1);
//        CyDelay(200);
//        LED1_Write(0);
//        CyDelay(200);
//    }
//    driveXdist(50,1,fwdspeed); // units = cm
//    CyDelay(100);
//    driveXdist(50,0,bwdspeed); //units = cm
//    CyDelay(4000);
//    
//    // task 2
//    for (int i=0;i<2;i++) { // flash LED twice
//        LED1_Write(1);
//        CyDelay(200);
//        LED1_Write(0);
//        CyDelay(200);
//    }
//    driveXdist(50,1,fwdspeed);
//    CyDelay(100);
//    turnXdegrees(180,1,trnspeed);
//    CyDelay(100);
//    driveXdist(50+21,1,fwdspeed);
//    CyDelay(4000);
//    
//    // task 3
//    for (int i=0;i<3;i++) { // flash LED twice
//        LED1_Write(1);
//        CyDelay(200);
//        LED1_Write(0);
//        CyDelay(200);
//    }
//    driveXdist(20,1,fwdspeed);
//    turnXdegrees(90,1,trnspeed);
//    driveXdist(50,1,fwdspeed);
//    turnXdegrees(90,0,trnspeed);
//    driveXdist(75,1,fwdspeed);
//    turnXdegrees(80,0,trnspeed);
//    driveXdist(100,1,fwdspeed);
//    turnXdegrees(80,0,trnspeed);
//    driveXdist(75,1,fwdspeed);
//    turnXdegrees(90,0,trnspeed);
//    driveXdist(64,1,fwdspeed);
//    turnXdegrees(90,1,trnspeed);
//    driveXdist(47,1,fwdspeed);
//    CyDelay(4000);
    
    // task 4
    
    
    S0_Write(1); // 20% scaling?
    S1_Write(0);
    periodLen = Counter_1_ReadPeriod(); // read length of period register (in counts)
    
     // task 4
        for (int i=0;i<4;i++) {
            LED1_Write(1);
            CyDelay(200);
            LED1_Write(0);
            CyDelay(200);
        }
    for(;;)
    {   
       
        getColour(&periodLen, &col, &dist);
        
        UART_1_PutString("\nColour: ");
        if (col==1) {
            UART_1_PutString("R ");
        } else if (col==2) {
            UART_1_PutString("G ");
        } else if (col==3) {
            UART_1_PutString("B ");
        }
        
        CyDelay(500);
    }
    
    
}

/* [] END OF FILE */
