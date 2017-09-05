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

#define SILENT FALSE // if set, suppress UART output


// defines for driving coefficients
#define DIST_COEFF 157
/*
conversion coefficient from distance in cm to quadrature encoder counter
i.e. dist * QUAD_COEFF = value that QuadDec should count up to to travel dist in cm.
200 is a roughly correct value i just measured; can be adjusted for preciseness later
*/
#define ROTATE_COEFF 39 // this is a complete estimate; test laterer


/* Defines for puck readings; calibrate when in new environment / lighting */
uint16 RED[3] = {180, 300, 250};
uint16 GRE[3] = {245, 215, 220};
uint16 BLU[3] = {260, 260, 170};

// for col count
uint16 capturedCount = 0u; // counter value when capture occurs
uint16 overflowCountCOL = 0u; // counter overflows between captures
int capflag = FALSE;     // set by ISR to tell main when capture occurs

// for shaft encoders
int16 overflowCountL = 0u; // NB can go negative for underflow (when moving backward)
int16 overflowCountR = 0u;

// driving speeds
uint8 fwdspeed = 210;
uint8 bwdspeed = 220;
uint8 trnspeed = 100;

// US sensors
uint16 uscount1=0;
float distance_m1=0;

uint16 uscount2=0;
float distance_m2=0;


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
    counter_status = COL_COUNTER_GetInterruptSource();
    
    // mask status register w pre-defd mask bits
    // these masks come from <counter_1.h>
    if ((counter_status & COL_COUNTER_STATUS_OVERFLOW) == COL_COUNTER_STATUS_OVERFLOW) {
        // interrupt is due to overflow
        overflowCountCOL++;
    }
    
    if ((counter_status & COL_COUNTER_STATUS_CAPTURE) == COL_COUNTER_STATUS_CAPTURE) {
        // interrupt is due to capture
        capturedCount = COL_COUNTER_ReadCapture();
        capflag = TRUE;
    }
}

/* ISR for shaft encoders -- identical, only differ in wheel value
NOTE: there's some issue with the invalid input interrupt triggering way more often than it should, so the
print on those interrupts is just removed for now. doesn't solve the problem but at least the UART isn't polluted*/
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


CY_ISR(Timer_ISR_Handler1){

    
    USTimer_1_ReadStatusRegister();
    uscount1=USTimer_1_ReadCounter();
    //printNumUART(uscount1);;
    //UART_1_PutString("usint1");
    
    distance_m1=(65535-uscount1)/58;
  
}
CY_ISR(Timer_ISR_Handler2){

    
    USTimer_2_ReadStatusRegister();
    uscount2=USTimer_2_ReadCounter();
    //printNumUART(uscount2);;
    //UART_1_PutString("\n");
    
    distance_m2=(65535-uscount2)/58;
    
    
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

/* Drives X distance (cm) forward/backward, polls shaft encoders every 20ms until X distance reached */
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
    //int32 deltDist;
    
    int32 XdistSE = Xdist*DIST_COEFF; // Xdist converted to QuadDec counter value
    if (!SILENT) {
        UART_1_PutString("\nMoving ");
        printNumUART(Xdist);
        UART_1_PutString(" cm = ");
        printNumUART(XdistSE);
        UART_1_PutString(" QuadDec count");
    }
    
    int Ldone=0;
    int Rdone=0;
    
    // start shaft encoders
    lsdist = (getDistance(1,0)); // get left starting dist
    rsdist = (getDistance(0,0)); // get right starting dist
    
    // set speed
    lspeed = speed;
    rspeed = speed;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    
    // set directions
    A3_Write(!dir); // R
    A4_Write(dir);
    CyDelay(12); // COMPENSATION DELAY; ADJUST AS NECESSARY
    A1_Write(!dir); // L
    A2_Write(dir);

    
    // poll SEs every 20ms, when both shaft encoders read X distance, stop
    while ((!Ldone)||(!Rdone)) {

        CyDelay(20); // the smaller this value, the more often the SEs are polled and hence the more accurate the distance
        
        // get relative distance from both wheels
        ldist = (getDistance(1,lsdist)); //left
        rdist = (getDistance(0,rsdist)); //right
        
        if (!SILENT) {
            UART_1_PutString("\nLdist = ");
            printNumUART(ldist);
            UART_1_PutString("  Rdist = ");
            printNumUART(rdist);
        }
        
        // some kind of closed feedback here; turned off for now because it's unreliable....
//        deltDist = ldist-rdist;
//        lspeed = lspeed - (deltDist>>3); // adjust speed by difference/4, but...
//        rspeed = rspeed + (deltDist>>3);
//        PWM_1_WriteCompare1(lspeed); // ..turned this feature off because it seems to drive straight anyway
//        PWM_1_WriteCompare2(rspeed);
        
        /* When either wheel reaches target distance, stop it */
        if (abs(ldist)>=XdistSE) {
            A1_Write(0);
            A2_Write(0);
            Ldone=1;
        }
        if (abs(rdist)>=XdistSE) {
            A3_Write(0);
            A4_Write(0);
            Rdone=1;
        }
    }            
}

/* Turns X degrees CW/CCW, polls shaft encoders every 10 ms until X degrees is reached.
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
    
    if (!SILENT) {
        UART_1_PutString("\nTurning ");
        printNumUART(Xdeg);
        UART_1_PutString(" degrees = ");
        printNumUART(XdegSE);
        UART_1_PutString(" QuadDec count");
    }

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
    
    // set direction; delay makes robot rotate at center of mass instead of center of wheel axes
    if (dir) {
        A3_Write(dir); // R
        A4_Write(!dir);
//        CyDelay(40);
        A1_Write(!dir); // L
        A2_Write(dir);
    } else {
        A1_Write(!dir); // L
        A2_Write(dir);
//        CyDelay(40);
        A3_Write(dir); // R
        A4_Write(!dir);
    }
        

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
        
        if (!SILENT) {
            UART_1_PutString("\nLdist = ");
            printNumUART(ldist);
            UART_1_PutString("  Rdist = ");
            printNumUART(rdist);
        }

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

void adjust_dist_US(int dir, uint16 dist, uint8 speed){

    int dflag=0;
    uint8 lspeed, rspeed;
    
        lspeed = speed;
        rspeed = speed+3;
        PWM_1_WriteCompare1(lspeed);
        PWM_1_WriteCompare2(rspeed);
    
            
        // set directions
        A3_Write(!dir); // R
        A4_Write(dir);
        A1_Write(!dir); // L
        A2_Write(dir);
    
    while(!dflag) {
           
        while(ECHO_R_Read()==0)
            {
            //UART_1_PutString("while\n");
            TRIG_R_Write(1);
            CyDelayUs(10);
            TRIG_R_Write(0);
            }
            //UART_1_PutString(".....\n");
            CyDelay(100);
            
        if(distance_m1<=dist) {
            dflag=1;
        }
            
    }
        
    A1_Write(0);
    A2_Write(0);
    A3_Write(0);
    A4_Write(0);            
    PWM_1_WriteCompare1(0);
    PWM_1_WriteCompare2(0);

}

void adjust_angle_US(uint8 speed){
    /*
    INPUTS
    speed (out of 255 max)
    */
    UART_1_PutString("\nAngle adjustment started");
    uint8 lspeed, rspeed;
    
    int done = 0;
    int dir;
    
    // read US
    TRIG_R_Write(1); TRIG_L_Write(1);
    CyDelayUs(10);
    TRIG_R_Write(0); TRIG_L_Write(0);
    CyDelay(10);
    dir = (distance_m1 > distance_m2) ? 0 : 1; // note: US1 is on the right, US2 on left
    UART_1_PutString("\nR:  ");
    printNumUART(distance_m1);
    UART_1_PutString("  L:  ");
    printNumUART(distance_m2);
    UART_1_PutString("\ndir = ");
    printNumUART(dir);
    
    // set speed -- backwards wheel moves faster
    lspeed = speed+dir*6;
    rspeed = speed+dir*6;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    
    
    // set direction; delay makes robot rotate at center of mass instead of center of wheel axes
    if (dir) {
        A3_Write(dir); // R
        A4_Write(!dir);
//        CyDelay(40);
        A1_Write(!dir); // L
        A2_Write(dir);
    } else {
        A1_Write(!dir); // L
        A2_Write(dir);
//        CyDelay(40);
        A3_Write(dir); // R
        A4_Write(!dir);
    }
    
        

    // poll USs, stop when both wheels have rotated enough
    while (!done) {

        // set trigger so distances update
        TRIG_R_Write(1); TRIG_L_Write(1);
        CyDelayUs(10);
        TRIG_R_Write(0); TRIG_L_Write(0);
        CyDelay(10);
        
        // get relative distance from both wheels
        if (dir & (distance_m2 <= distance_m1)) {
            done=1;
        } else if (!dir & (distance_m1 <= distance_m2)) {
            done=1;
        }
        
        UART_1_PutString("\nR:  ");
        printNumUART(distance_m1);
        UART_1_PutString("  L:  ");
        printNumUART(distance_m2);
        UART_1_PutString("\n");
        
    }
    
    // stop wheels
    A1_Write(0);
    A2_Write(0);
    A3_Write(0);
    A4_Write(0);            
    PWM_1_WriteCompare1(0);
    PWM_1_WriteCompare2(0);
};

/* Polls colour sensor several times at 100us intervals to get a reliable colour reading, and returns a integer value
corresponding with the closest colour reading. Outputs 0 if no colour is strongly detected */
int getColour(uint16 periodLen) {
    /* INPUTS
    periodLen = length (in clock counts) of period in COL_COUNTER component
    
    OUTPUTS
    col = closest colour reading within 500-units of L1 distance (units = clock counts of COL_COUNTER)
        0 = no colour
        1 = red
        2 = green
        3 = blue
    */
    
    // initialise
    uint16 rCount, gCount, bCount;
    uint16 temp;
    uint16 rDist, gDist, bDist;
    uint16 minDist;
    int rep = 3;
    
    S0_Write(1); // 20% scaling?
    S1_Write(0);
    
    // cycle through colours
    // get 'rep' number of readings for each colour, then take the smallest
    // this should circumvent problem of missing input pulses
    // note overflowCount isn't being reset, shouldn't be an issue since counter period is easily sufficient to handle COLOUT period
    rCount = gCount = bCount = 0xffff; // set these to max values to begin with so the min sensor reading overwrites them
    
    S2_Write(0); S3_Write(0); // RED
        for (int i=0;i<rep;i++) {
            CyDelayUs(100);
            temp = (overflowCountCOL * periodLen) + capturedCount;
            rCount = (rCount < temp) ? rCount : temp;
        }
    
    S2_Write(1); S3_Write(1); // GREEN
        for (int i=0;i<rep;i++) {
            CyDelayUs(100);
            temp = (overflowCountCOL * periodLen) + capturedCount;
            gCount = (gCount < temp) ? gCount : temp;
        }
        
    S2_Write(0); S3_Write(1); // BLUE
        for (int i=0;i<rep;i++) {
            CyDelayUs(100);
            temp = (overflowCountCOL * periodLen) + capturedCount;
            bCount = (bCount < temp) ? bCount : temp;
        }
    
    // print measures
    if (!SILENT) {
        UART_1_PutString("\nSens: R");
        printNumUART(rCount);
        UART_1_PutString(" G");
        printNumUART(gCount);
        UART_1_PutString(" B");
        printNumUART(bCount);
    }
    
    
    // find L1 distance to each puck location
    rDist = abs(RED[0]-rCount) + abs(RED[1]-gCount) + abs(RED[2]-bCount);
    gDist = abs(GRE[0]-rCount) + abs(GRE[1]-gCount) + abs(GRE[2]-bCount);
    bDist = abs(BLU[0]-rCount) + abs(BLU[1]-gCount) + abs(BLU[2]-bCount);
    
    if (!SILENT) {
        UART_1_PutString("\nDist: R");
        printNumUART(rDist);
        UART_1_PutString(" G");
        printNumUART(gDist);
        UART_1_PutString(" B");
        printNumUART(bDist);
    }
    
    // find which is lowest i guess?
    temp = (rDist < gDist) ? rDist : gDist;
    minDist = (bDist < temp) ? bDist : temp;
    
    if (minDist<500) {
        return 0;
    } else if (minDist==rDist) {
        return 1;
    } else if (minDist==gDist) {
        return 2;
    } else if (minDist==bDist) {
        return 3;
    } else {
        return 0;
    }
}

/* Flashes the PSoC LED (pin 2[1]) X number of times with a 400ms period, useful for very basic signalling without UART */
void flashXtimes(int rep) {
    
    for (int i=0;i<rep;i++) {
        LED1_Write(1);
        CyDelay(200);
        LED1_Write(0);
        CyDelay(200);
    }
    
}

/* Prelim comp tasks */
void task1() {
    // Prelim Task 1
    flashXtimes(1);
    
    driveXdist(50,1,fwdspeed); // units = cm
    CyDelay(100);
    driveXdist(50,0,bwdspeed); //units = cm
    
    flashXtimes(1);
}
void task2() {
    // Prelim Task 2
    flashXtimes(2);
    
    driveXdist(50,1,fwdspeed); CyDelay(100);
    turnXdegrees(180,1,trnspeed); CyDelay(100);
    adjust_angle_US(trnspeed); CyDelay(100);
    driveXdist(50+15,1,fwdspeed);
    
    flashXtimes(2);
}
void task3() {
    // Prelim Task 3
    flashXtimes(3);
    
    driveXdist(20,1,fwdspeed);
    
    turnXdegrees(90,1,trnspeed);
    driveXdist(50,1,fwdspeed);
    
    turnXdegrees(90,0,trnspeed);
    driveXdist(75,1,fwdspeed);
    
    turnXdegrees(80,0,trnspeed);
    driveXdist(90,1,fwdspeed);
    
    adjust_angle_US(trnspeed);
    turnXdegrees(87,0,trnspeed);
    driveXdist(75,1,fwdspeed);
    
    turnXdegrees(90,0,trnspeed);
    driveXdist(50,1,fwdspeed);
    
    turnXdegrees(90,1,trnspeed);
    adjust_angle_US(trnspeed);
    driveXdist(37,1,fwdspeed);
    
    flashXtimes(3);
}
void task4(uint16 periodLen) {
    // Prelim Task 4
    // NOTE: this enters an infinite loop by design; make an exit flag if you want to do something after Task 4
    flashXtimes(4);
    
    int col;
    
    for(;;) {   
        col = getColour(periodLen);
        if (!SILENT) {
            UART_1_PutString("\nColour: ");
            if (col==0) {
                UART_1_PutString("None");
            } else if (col==1) {
                UART_1_PutString("R ");
            } else if (col==2) {
                UART_1_PutString("G ");
            } else if (col==3) {
                UART_1_PutString("B ");
            }
        }
        CyDelay(10);
    }
    
}


/* ===================================================================== */

int main(void)
{   
    
    USTimer_1_Start();
    USTimer_2_Start();
    
    isr_1_StartEx(Timer_ISR_Handler1);
    isr_2_StartEx(Timer_ISR_Handler2);
    
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    
    // start components and ISRs
    UART_1_Start();
        UART_1_PutString("\nUART started");
        if (SILENT) {
            UART_1_PutString(" but SILENT flag is set (output is suppressed)");
        }
    PWM_1_Start();
    SEL_QUAD_Start();
    SER_QUAD_Start();
    ISR_QUAD1_StartEx(QUAD1_ISR);
    ISR_QUAD2_StartEx(QUAD2_ISR);
    ISR_COL_StartEx(COL_ISR);
    
    // variables
    uint16 periodLen = 0u;   // length of counter period
    
    periodLen = COL_COUNTER_ReadPeriod(); // read length of period register (in clock counts)
    
    // briefly wait before starting tasks
    CyDelay(500);
    
    // prelim comp
//    task1(); CyDelay(4000);  
//    task2(); CyDelay(4000);
//    task3(); CyDelay(4000);
//    task4(periodLen);
//    
    task3();
    
    
    for(;;)
    {
//       while(ECHO_R_Read()==0)
//        {
//        TRIG_R_Write(1);
//        CyDelayUs(10);
//        TRIG_R_Write(0);
//        }
//        
//        while(ECHO_L_Read()==0)
//        {
//        TRIG_L_Write(1);
//        CyDelayUs(10);
//        TRIG_L_Write(0);
//        }
//        
//        UART_1_PutString("R:  ");
//        printNumUART(distance_m1);
//        UART_1_PutString("L:  ");
//        printNumUART(distance_m2);
//        UART_1_PutString("\n\n");
//        
//        //UART_1_PutString(".....\n");
//        CyDelay(100);
        

        
    }
    
}

/* [] END OF FILE */
