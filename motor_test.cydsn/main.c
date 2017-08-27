/* ========================================
DRIVING TESTS

todo:
    - 
 * ========================================
*/

#include "project.h"
#include "stdlib.h"

// defines
#define DIST_COEFF 200  
/*
conversion coefficient from distance in cm to quadrature encoder counter
i.e. dist * QUAD_COEFF = value that QuadDec should count up to to travel dist in cm.
200 is a roughly correct value i just measured; can be adjusted for preciseness later
*/
#define ROTATE_COEFF 25 // this is a complete estimate; test later

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

/* ISR for shaft encoders -- identical, only differ in wheel value */
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
void driveXdist(int32 Xdist, int dir) {
    /* 
    INPUTS
    Xdist = distance to move forward (cm)
    dir = binary direction; 1 = forward, 0 = backward
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
    
    // 25% speed
    lspeed = 63;
    rspeed = 63;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    
    // start motors
    A1_Write(!dir); // L
    A2_Write(dir);
    A3_Write(!dir); // R
    A4_Write(dir);
    
    // poll SEs every 50ms, when both shaft encoders read X distance, stop
    while (!done) {

        CyDelay(50); // the smaller this value, the more often the SEs are polled and hence the more accurate the distance
        
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
            done=1;
        }
    }            
}

/* Turns X degrees CW/CCW, polls shaft encoders every 50 ms until X degrees is reached.
very similar to driveXdist() */
void turnXdegrees(int16 Xdeg, int dir) {
    /*
    INPUTS
    Xdeg = desired angle to rotate robot (degrees)
    dir = direction to rotate (1=clockwise, 0=anticlockwise)
    */

    uint lspeed, rspeed;
    int32 ldist, rdist, lsdist, rsdist;
    int32 XdegSE = Xdef * ROTATE_COEFF;
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
    
    // 25% speed
    lspeed = 63;
    rspeed = 63;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    
    // start motors
    A1_Write(!dir); // L
    A2_Write(dir);
    A3_Write(dir); // R
    A4_Write(!dir);

    // poll SEs every 50ms, stop when both wheels have rotated enough
    while (!Ldone && !Rdone) {

        CyDelay(50);

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
        if (abs(ldist)>=XdistSE) {
            A1_Write(0);
            A2_Write(0);
            Ldone=1;
        }
        if (abs(rdist)>=XdistSE) {
            A1_Write(0);
            A2_Write(0);
            Rdone=1;
        }
    }
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
    
    // enable
    PWM_1_Enable();
    
    
    // driveXdist();
    driveXdist(20,1); // units = cm
    CyDelay(100);
    //driveXdist(20,0);
    
    
    for(;;)
    {   

    }
    
    
}

/* [] END OF FILE */
