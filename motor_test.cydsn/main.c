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

#define SILENT FALSE // if set, suppress general UART output
#define DRIVESILENT TRUE // if set, suppress driving UART


// defines for unit-conversion coefficients
#define DIST_COEFF 21
#define ROTATE_COEFF 39
#define PULLEY_DIST_COEFF 50 // TO BE TESTED


/* Defines for puck readings; calibrate when in new environment / lighting */
// sensor 1 : inside claw
uint16 RED[3] = {5000, 5010, 7530};
uint16 GRE[3] = {6750, 5870, 5870};
uint16 BLU[3] = {7900, 7400, 4750};

// sensor 2 : on wall
uint16 RED2[3] = {3000, 3000, 6300};
uint16 GRE2[3] = {4800, 4500, 4400};
uint16 BLU2[3] = {5800, 5800, 3500};

// uint16 redc=250;
// uint16 greenc=60;
// uint16 bluec=70;


// for col count
uint16 capturedCount = 0u; // counter value when capture occurs
uint16 overflowCountCOL = 0u; // counter overflows between captures
int capflag = FALSE;     // set by ISR to tell main when capture occurs

// for shaft encoders
int16 overflowCountL = 0u; // NB can go negative for underflow (when moving backward)
int16 overflowCountR = 0u;

// driving speeds
uint8 fwdspeed = 150;
uint8 bwdspeed = 180;
uint8 trnspeed = 100;
uint8 adjspeed = 36;
uint8 pllspeed = 160;

// US sensors
uint16 uscount1=0;
float distance_m1=0;
uint16 uscount2=0;
float distance_m2=0;
uint16 uscount3=0;
float distance_mid=0;

// for storing colour sequence
int8 SEQ[5];
uint16 mindist = 0u;
uint16 periodLen = 0u;   // length of counter period

// flag for block presence (on initial path)
int blockflag=0;

// puck array
uint8 puckcount=0u; // count of how many pucks in arena have been fetched / checked; max of 25?
uint8 prow=0u; // puckrow; this and puckcol are used to program location of next puck to retrieve
uint8 pcol=0u;

// stacking
int stackcount=0; // count of how many pucks have been stacked; max of 5

// servo angles
uint8 sopen=100;
uint8 sclose=23;

// flag
int alldone=0; // only set when all tasks have been completed; tells robot to exit stack loop


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

/* ISRs for ultrasonics */
CY_ISR(Timer_ISR_HandlerR){

    
    USTimer_R_ReadStatusRegister();
    uscount1=USTimer_R_ReadCounter();
    //printNumUART(uscount1);;
    //UART_1_PutString("usint1");
    
    distance_m1=(65535-uscount1)/5.8;
  
}
CY_ISR(Timer_ISR_HandlerL){

    
    USTimer_L_ReadStatusRegister();
    uscount2=USTimer_L_ReadCounter();
    //printNumUART(uscount2);;
    //UART_1_PutString("\n");
    
    distance_m2=(65535-uscount2)/5.8;
    
    
}
CY_ISR(Timer_ISR_HandlerM){

    
    USTimer_M_ReadStatusRegister();
    uscount3=USTimer_M_ReadCounter();
    //printNumUART(uscount2);;
    //UART_1_PutString("\n");
    
    distance_mid=(65535-uscount3)/5.8;
    
    
}

/* Reads the quadrature decoder connected to the shaft encoder and returns the distance as a int32
   Used for all shaft encoders, on wheel motors and pulley motor */
int32 getDistance(int side, int32 startdist) {
    
    int32 distance;
    int16 SE_COUNT = 0u;
    if (side==0) {
        SE_COUNT = SER_QUAD_GetCounter();
        distance = overflowCountR * 0x7fff + SE_COUNT;
    } else if (side==1) {
        SE_COUNT = SEL_QUAD_GetCounter();
        distance = overflowCountL * 0x7fff + SE_COUNT;
    } else if (side==2) {
        SE_COUNT = SECL_QUAD_GetCounter();
        distance = overflowCountR * 0x7fff + SE_COUNT;
    }
    
    distance = distance-startdist;

    return distance;
}

/* Drives X distance (mm) forward/backward, polls shaft encoders every 10ms until X distance reached */
void driveXdist(int32 Xdist, int dir) {
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
    if (dir) {
        lspeed = fwdspeed;
        rspeed = fwdspeed;
    } else {
        lspeed = bwdspeed;
        rspeed = bwdspeed;
    }
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    
    // set directions
    A3_Write(!dir); // R
    A4_Write(dir);
    // CyDelay(12); // COMPENSATION DELAY; ADJUST AS NECESSARY
    A1_Write(!dir); // L
    A2_Write(dir);

    
    // poll SEs every 20ms, when both shaft encoders read X distance, stop
    while ((!Ldone)||(!Rdone)) {

        CyDelay(1); // the smaller this value, the more often the SEs are polled and hence the more accurate the distance
        
        // get relative distance from both wheels
        ldist = (getDistance(1,lsdist)); //left
        rdist = (getDistance(0,rsdist)); //right
        
        if (!DRIVESILENT) {
            UART_1_PutString("\nLdist = ");
            printNumUART(ldist);
            UART_1_PutString("  Rdist = ");
            printNumUART(rdist);
        }
        
        // some kind of closed feedback here; turned off for now because it's unreliable....
       // deltDist = ldist-rdist;
       // lspeed = lspeed - (deltDist>>3); // adjust speed by difference/4, but...
       // rspeed = rspeed + (deltDist>>3);
       // PWM_1_WriteCompare1(lspeed); // ..turned this feature off because it seems to drive straight anyway
       // PWM_1_WriteCompare2(rspeed);
        
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

    CyDelay(10);
}

/* Turns X degrees CW/CCW, polls shaft encoders every 10 ms until X degrees is reached.
very similar to driveXdist() */
void turnXdegrees(int16 Xdeg, int dir) {
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
    lspeed = trnspeed+dir*6;
    rspeed = trnspeed+dir*6;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    
    // set direction; delay makes robot rotate at center of mass instead of center of wheel axes
    if (dir) {
        A3_Write(dir); // R
        A4_Write(!dir);
       // CyDelay(40);
        A1_Write(!dir); // L
        A2_Write(dir);
    } else {
        A1_Write(!dir); // L
        A2_Write(dir);
       // CyDelay(40);
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
        
        if (!DRIVESILENT) {
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
    CyDelay(10);
}

/* Write to all ultrasonic TRIG pins, which should update all their distance values through respective ISRs. */
void updateUS(void) {
    
    //right
    TRIG_Write(1);
    CyDelayUs(10);
    TRIG_Write(0);
    
}
    
void adjust_dist_US(int dir, uint16 dist, uint8 speed){

    int dflag=0;
    int cdist=0;
    int ccount=0;
    uint8 lspeed, rspeed;
    
    uint8 frontdist=100;

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
        ccount=0;
        cdist=0;
        while(ccount<7){
           
        while(ECHO_R_Read()==0)
            {
            //UART_1_PutString("while\n");
            updateUS();
            }
            //UART_1_PutString(".....\n");
         cdist=cdist+distance_m1;   
        CyDelay(10);
        ccount++;
        }
        
        
        cdist=cdist/7;
        
        if (!SILENT) {
            UART_1_PutString("\n cdist: ");
            printNumUART(cdist);
        }
        
        if(cdist<=(dist+frontdist)) {
            dflag=1;
        }        
             
    }
        
    A1_Write(0);
    A2_Write(0);
    A3_Write(0);
    A4_Write(0);            
    PWM_1_WriteCompare1(0);
    PWM_1_WriteCompare2(0);
    
    CyDelay(10);

}

void adjust_distances(uint16 dist, uint8 speed){

    int dflagl=0;
    int dflagr=0;
    uint8 lspeed, rspeed;
    
    uint8 frontdist=105;

    // calc average us distances
    float distR=0;
    float distL=0;
    for (int runavg=0;runavg<10;runavg++){
        updateUS();
        CyDelay(5);
        distR=distR+distance_m1;
        UART_1_PutString("\nd_M1 ");
        printNumUART(distance_m1);
        distL=distL+distance_m2;
    }
    distR=distR/10;
    distL=distL/10;
    if (!SILENT) {
        UART_1_PutString("\nL  ");
        printNumUART(distL);
        UART_1_PutString("\nR  ");
        printNumUART(distR);
    }
        
    // set directions
    int dirR = (distR > dist+frontdist) ? 1 : 0;
    int dirL = (distL > dist+frontdist) ? 1 : 0;
    if (!SILENT) {
        UART_1_PutString("\ndirL  ");
        printNumUART(dirL);
        UART_1_PutString("  dirR  ");
        printNumUART(dirR);
    }
    lspeed = speed;
    rspeed = speed;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    A3_Write(!dirR); // R
    A4_Write(dirR);
    A1_Write(!dirL); // L
    A2_Write(dirL);
    
    while((!dflagl)||(!dflagr)) {
           
        distR=0;
        distL=0;
        for (int runavg=0;runavg<2;runavg++){
            updateUS();
            CyDelay(4);
            distR=distR+distance_m1;
            distL=distL+distance_m2;
            UART_1_PutString("\nM1  ");
            printNumUART(distance_m1);
            UART_1_PutString(" M2  ");
            printNumUART(distance_m2);
        }
        distR=distR/2;
        distL=distL/2;
        
        if (!SILENT) {
            UART_1_PutString("\nloopL  ");
            printNumUART(distL);
            UART_1_PutString(" loopR  ");
            printNumUART(distR);
        }
        
        // stop each wheel individually
        if((dirR&&distR<=(dist+frontdist))||(!dirR&&distR>=(dist+frontdist))) {
            dflagr=1;
            A3_Write(0);
            A4_Write(0);   
        }
        if((dirL&&distL<=(dist+frontdist))||(!dirL&&distL>=(dist+frontdist))) {
            dflagl=1;
            A1_Write(0);
            A2_Write(0); 
        }            
             
    }

    PWM_1_WriteCompare1(0);
    PWM_1_WriteCompare2(0);
    
    CyDelay(10);

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
    int runavg=0;
    float dist1a=0;
    float dist2a=0;
    
    // read US
    while(runavg<=10){
    updateUS();
    CyDelay(10);
    
    dist1a=dist1a+distance_m1;
    dist2a=dist2a+distance_m2;
    runavg++;
    
    }
    
    dist1a=dist1a/10;
    dist2a=dist2a/10;
    
    dir = (dist1a > dist2a) ? 0 : 1; // note: US1 is on the right, US2 on left
    if(((dist1a-dist2a)*(dist1a-dist2a))<=1){return;}
    UART_1_PutString("\nR:  ");
    printNumUART(dist1a);
    UART_1_PutString("  L:  ");
    printNumUART(dist2a);
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
       // CyDelay(40);
        A1_Write(!dir); // L
        A2_Write(dir);
    } else {
        A1_Write(!dir); // L
        A2_Write(dir);
       // CyDelay(40);
        A3_Write(dir); // R
        A4_Write(!dir);
    }
    
        runavg=0;
        dist1a=0;
        dist2a=0;

    // poll USs, stop when both wheels have rotated enough
    while (!done) {
    
        while(runavg<8){    
        // set trigger so distances update
        updateUS();
        CyDelay(10);
        
        dist1a=dist1a+distance_m1;
        dist2a=dist2a+distance_m2;
        runavg++;
        
        }
        
         dist1a=dist1a/8;
         dist2a=dist2a/8;
    
        
        // get relative distance from both wheels
        if (dir & (dist2a <= dist1a)) {
            done=1;
        } else if (!dir & (dist1a <= dist2a)) {
            done=1;
        }
        
        if(((dist1a-dist2a)*(dist1a-dist2a))<=1){done=1;}
        
       UART_1_PutString("\nR:  ");
       printNumUART(dist1a);
       UART_1_PutString("  L:  ");
       printNumUART(dist2a);
       UART_1_PutString("\n");
        
        runavg=0;
        dist1a=0;
        dist2a=0;
        
    }
    
    // stop wheels
    A1_Write(0);
    A2_Write(0);
    A3_Write(0);
    A4_Write(0);            
    PWM_1_WriteCompare1(0);
    PWM_1_WriteCompare2(0);
    
    CyDelay(10);
    
};

/* Polls colour sensor several times to get a reliable colour reading, and returns a integer value
corresponding with the closest colour reading. Outputs 0 if no colour is strongly detected */
int getColour(int sensor) {
    /* INPUTS */
    // sensor = 1 for claw colour sensor, 2 for side sensor
    
    // initialise
    uint16 rCount=10000;
    uint16 gCount=10000;
    uint16 bCount=10000;
    uint16 rCountTmp, gCountTmp, bCountTmp;
    uint16 rDist, gDist, bDist;
    int temp, min;
    int rep = 12;
    
    uint16 nonethres; // largest size of mindist before sensor decides 'none' colour
   
    // enable output
    if (sensor==1) {
        COL2EN_Write(0);
        COL1EN_Write(1);
        nonethres=3500;
    } else if (sensor==2) {
        COL1EN_Write(0);
        COL2EN_Write(1);
        nonethres=20000;
    }
    
    // cycle through colours
    S2_Write(0); S3_Write(0); // RED
        for (int i=0;i<rep;i++) {
            CyDelay(2);
           // rCountTmp = (overflowCountCOL * periodLen) + capturedCount;
            rCountTmp = capturedCount;
            rCount = (rCount < rCountTmp) ? rCount : rCountTmp;
        }
        overflowCountCOL = 0u;
    
    S2_Write(1); S3_Write(1); // GREEN
        for (int i=0;i<rep;i++) {
            CyDelay(2);
           // gCountTmp = (overflowCountCOL * periodLen) + capturedCount;
            gCountTmp = capturedCount;
            gCount = (gCount < gCountTmp) ? gCount : gCountTmp;
        }
        overflowCountCOL = 0u;
        
    S2_Write(0); S3_Write(1); // BLUE
        for (int i=0;i<rep;i++) {
            CyDelay(2);
           // bCountTmp = (overflowCountCOL * periodLen) + capturedCount;
            bCountTmp = capturedCount;
            bCount = (bCount < bCountTmp) ? bCount : bCountTmp;
        }
        overflowCountCOL = 0u;
        
    S2_Write(0); S3_Write(0);
    
    // disable output
    COL1EN_Write(0);
    COL2EN_Write(0);
    
    // print measures
    if (!SILENT) {
        UART_1_PutString("\n\nSens: R");
        printNumUART(rCount);
        UART_1_PutString(" G");
        printNumUART(gCount);
        UART_1_PutString(" B");
        printNumUART(bCount);
    }
    
    
    // find L1 distance to each puck location
    if (sensor==1) {
        rDist = abs(RED[0]-rCount) + abs(RED[1]-gCount) + abs(RED[2]-bCount);
            UART_1_PutString("\nDist: R");
            printNumUART(rDist);
        gDist = abs(GRE[0]-rCount) + abs(GRE[1]-gCount) + abs(GRE[2]-bCount);
            UART_1_PutString(" G");
            printNumUART(gDist);
        bDist = abs(BLU[0]-rCount) + abs(BLU[1]-gCount) + abs(BLU[2]-bCount);
            UART_1_PutString(" B");
            printNumUART(bDist);
    } else if (sensor==2) {
        rDist = abs(RED2[0]-rCount) + abs(RED2[1]-gCount) + abs(RED2[2]-bCount);
            UART_1_PutString("\nDist: R");
            printNumUART(rDist);
        gDist = abs(GRE2[0]-rCount) + abs(GRE2[1]-gCount) + abs(GRE2[2]-bCount);
            UART_1_PutString(" G");
            printNumUART(gDist);
        bDist = abs(BLU2[0]-rCount) + abs(BLU2[1]-gCount) + abs(BLU2[2]-bCount);
            UART_1_PutString(" B");
            printNumUART(bDist);
    }
    
    // which is lowest i guess?
    temp = (rDist < gDist) ? rDist : gDist;
    mindist = (bDist < temp) ? bDist : temp;
    min = (bDist < temp) ? 3 : 2;
    if (min==2) {
        min = (rDist < gDist) ? 1 : 2;
    }
    
    if (!SILENT) {
        UART_1_PutString("\n");
        printNumUART(mindist);
       // UART_1_PutString("\n Colour: ");
       // char cols[4]="0RGB";
       // char col=cols[min];
       // UART_1_PutString(&col); // not sure if this col printing works
    }
    
    if (mindist<nonethres) {
        return min;
    } else {
        return 0;
    }
}

/* Moves servo to specified angle */
void moveServo(int16 angle) {
    /* INPUTS
    angle = value from -90 to 90 (degrees)
    */
    
    uint16 duty;
    
    duty = 4500 + 16*angle; // range from 3k to 6k duty cycle
                            // pwm period = 60k, 20ms so this corresponds 
                            // 1ms to 2ms range
    PWM_SERVO_WriteCompare(duty);
    
    UART_1_PutString("\nduty  ");
    printNumUART(duty);
}

/* Lift or drop claw using pulley, in dist in mm */
void liftClaw(int16 dist, int dir) {
    int32 pdist = 0;
    int32 psdist = 0; // starting distance
    
    int32 pdistSE = dist*PULLEY_DIST_COEFF; // Xdist converted to QuadDec counter value
    
    int done=0;
    
    // start shaft encoders
    psdist = (getDistance(2,0)); // get right starting dist
    
    // set speed
    if (dir) {
        PWM_2_WriteCompare(pllspeed);
    } else {
        PWM_2_WriteCompare(pllspeed/2);
    }
    
    // set direction
    A5_Write(!dir);
    A6_Write(dir);
    
    // poll SEs every 20ms, when both shaft encoders read X distance, stop
    while (!done) {

        CyDelay(5); // the smaller this value, the more often the SEs are polled and hence the more accurate the distance
        
        // get relative pulley travel distance
        pdist = (getDistance(2,psdist)); 
        
        if (!SILENT) {
            UART_1_PutString("\nPdist = ");
            printNumUART(pdist);
        }
        
        /* When pulley reaches target distance, stop it */
        if (abs(pdist)>=pdistSE) {
            A5_Write(0);
            A6_Write(0);
            done=1;
        }
    } 
    
}

/* Flashes the PSoC LED (pin 2[1]) X number of times */
void flashXtimes(int rep) {
    
    for (int i=0;i<rep;i++) {
        LED1_Write(1);
        CyDelay(100);
        LED1_Write(0);
        CyDelay(100);
    }
    
}

/* Sound the piezo for X milliseconds */
void soundPiezo(int msec) {
    
    PIEZO_Write(1);
    CyDelay(msec);
    PIEZO_Write(0);
    
}

/* Beep the piezo X times */
void beepXtimes(int rep) {
    
    for (int i=0; i<rep; i++) {
        soundPiezo(100);
        CyDelay(100);
    }
}

int updateusxtimes_r(int rep){
    
    int ccount=0;
    int cdist_r=0;
    
    while(ccount<rep){
  
        updateUS();
        CyDelay(5);
        
        cdist_r=distance_m1+cdist_r;
        ccount++;
        
        UART_1_PutString("\ncdist iteration:");
        printNumUART(cdist_r);
    }
    
    UART_1_PutString("\ncdist total:");
    printNumUART(cdist_r);
    UART_1_PutString("\n");
    
    cdist_r=cdist_r/rep;
    return cdist_r;
}

int updateusxtimes_l(int rep){
    
    int ccount=0;
    int cdist_l=0;
    
    while(ccount<rep){
  
        updateUS();
        CyDelay(5);
        
        cdist_l=distance_m2+cdist_l;
        ccount++;
        
        UART_1_PutString("\ncdist iteration:");
        printNumUART(cdist_l);
    }
    
    UART_1_PutString("\ncdist total:");
    printNumUART(cdist_l);
    UART_1_PutString("\n");
    
    cdist_l=cdist_l/rep;
    return cdist_l;
}

/* Prelim comp tasks */
void task1() {
    // Prelim Task 1
    flashXtimes(1);
    
    driveXdist(60,1); // units = cm
    CyDelay(300);
    
    driveXdist(60,0); //units = cm
    
    flashXtimes(1);
}
void task2() {
    // Prelim Task 2
    flashXtimes(2);
    
    driveXdist(60,1); CyDelay(200);
    turnXdegrees(180,1); CyDelay(200);
    adjust_angle_US(adjspeed); CyDelay(200);
    adjust_angle_US(adjspeed); CyDelay(200);
    adjust_angle_US(adjspeed); CyDelay(200);
    
    //driveXdist(50+15,1);
    
    adjust_dist_US(1,30,fwdspeed);CyDelay(200);
    
    adjust_angle_US(adjspeed); CyDelay(200);
    adjust_angle_US(adjspeed); CyDelay(200);
    adjust_angle_US(adjspeed); CyDelay(200);
    
    adjust_dist_US(1,2,fwdspeed);
    
    flashXtimes(2);
}
void task3() {
    // Prelim Task 3
    flashXtimes(3);
    //straight
    driveXdist(20,1);
    CyDelay(100);
    
    //right
    turnXdegrees(90,1);
    CyDelay(100);
    //stright
    driveXdist(50,1);
    CyDelay(100);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    
    //left
    turnXdegrees(90,0);
    CyDelay(100);
    //stright
    driveXdist(75,1);
    CyDelay(100);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    
    //left
    turnXdegrees(90,0);
    CyDelay(100);
    //stright
    driveXdist(90,1);
    CyDelay(100);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    
    //left 
    turnXdegrees(90,0);
    CyDelay(100);
    //straight
    driveXdist(75,1);
    CyDelay(100);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    
    //left
    turnXdegrees(90,0);
    CyDelay(100);
    //straight
    driveXdist(43,1);
    CyDelay(100);
    
    //right
    turnXdegrees(90,1);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed); //second adjustment
    CyDelay(100);
    //straight
    driveXdist(33,1);
    
    flashXtimes(3);
}
void task3r() {
    
    
    
    //straight
    driveXdist(60,1);
    CyDelay(200);
    
    //right
    turnXdegrees(90,1);
    CyDelay(200);
    
    //straight
    driveXdist(200,1);
    CyDelay(200);
    adjust_dist_US(1,5,fwdspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(2000);
    

    //left1
    turnXdegrees(90,0);
    CyDelay(200);
    
    //straight
    driveXdist(450,1);
    CyDelay(200);
    adjust_dist_US(1,8,fwdspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(2000);
    
    
    //left2
    turnXdegrees(90,0);
    CyDelay(200);
    
    //backup here
    driveXdist(60,0);
    CyDelay(200);
    
     //straight
    driveXdist(150,1);
    CyDelay(200);
    
    //closeservo
    moveServo(0);
    CyDelay(200);
    
    liftClaw(10,1);
    CyDelay(20000);
    
    //reverse
    driveXdist(350,0);
    CyDelay(200);
    
    //going straight before turn
    driveXdist(150,1);
    CyDelay(200);
    
    //first return left
    turnXdegrees(90,0);
    CyDelay(200);
    
    //turn right for homebase
    turnXdegrees(90,1);
    CyDelay(200);
    
    //go straight to consturction zone
    driveXdist(400,1);
    CyDelay(200);
    
    //turn left to drop into consturctionzone
    turnXdegrees(90,1);
    CyDelay(200);
    
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    
    //lower claw
    liftClaw(5,0);
    CyDelay(200);
    
    //openclaw
    moveServo(50);
    CyDelay(200);

}
void task4(int sensor) {
    // Prelim Task 4
    // NOTE: this enters an infinite loop by design; make an exit flag if you want to do something after Task 4
    // NOTE: now used as a generic infinite colour sensing loop; therefore set sensor number as input
    // sensor = 1 for claw colour sensor, 2 for side colour sensor
    
    flashXtimes(4);
    
    S0_Write(0); // 2% scaling?
    S1_Write(1);
    
    int col;
    
    for(;;) {   
        col = getColour(sensor);
        if (!SILENT) {
            UART_1_PutString("\nColour: ");
            if (col==0) {
                UART_1_PutString("None");
               // LEDR_Write(0);
               // LEDG_Write(0);
               // LEDB_Write(0);
            } else if (col==1) {
                UART_1_PutString("R ");
               // LEDR_Write(1);
               // LEDG_Write(0);
               // LEDB_Write(0);
            } else if (col==2) {
                UART_1_PutString("G ");
               // LEDR_Write(0);
               // LEDG_Write(1);
               // LEDB_Write(0);
            } else if (col==3) {
                UART_1_PutString("B ");
               // LEDR_Write(0);
               // LEDG_Write(0);
               // LEDB_Write(1);
            }
        }
        CyDelay(50);
    }
    
}

/**** Final comp tasks ****/
/* Store 5 colours from wall-mounted pucks */
void readWallPucks(void) {
    
    int col; //takes 'best out of 3'
    int cols[3];
        
    /* SCALE HERE */
    S0_Write(0); // 2% scaling?
    S1_Write(1);
    
    // drive into starting position
    driveXdist(30,0);
    CyDelay(300);
    
    
    // drive along wall fetching colours
    for (int i=0; i<5; i++) {
        
        // get colour
        cols[0] = getColour(2);
            CyDelay(20);
        cols[1] = getColour(2);
            CyDelay(20);    
        cols[2] = getColour(2);
            CyDelay(20);
        // this is a pretty hacky solution to take best of 3
        if (cols[0]==cols[1] || cols[0]==cols[2]) {
            col=cols[0];
        } else {
            col=cols[1];
        }
        
        // print colour to UART
        if (!SILENT) {
            UART_1_PutString("\nColour: ");
            if (col==0) {
                UART_1_PutString("None");
               // LEDR_Write(0);
               // LEDG_Write(0);
               // LEDB_Write(0);
            } else if (col==1) {
                UART_1_PutString("R ");
               // LEDR_Write(1);
               // LEDG_Write(0);
               // LEDB_Write(0);
            } else if (col==2) {
                UART_1_PutString("G ");
               // LEDR_Write(0);
               // LEDG_Write(1);
               // LEDB_Write(0);
            } else if (col==3) {
                UART_1_PutString("B ");
               // LEDR_Write(0);
               // LEDG_Write(0);
               // LEDB_Write(1);
            }
        }
        
        // indicate colour with piezo
        beepXtimes(col);
        
        // store colour
        SEQ[4-i]=col; // reads top->bottom so fill SEQ backwards 
        
        // drive to next colour
        driveXdist(56,1);
        
        // wait
        CyDelay(1000);
        
        
        
    }
    
    flashXtimes(3);
    
    
    UART_1_PutString("\n SEQ: ");
    for (int i=0; i<5; i++) {
        printNumUART(SEQ[i]);
        UART_1_PutString("  ");
    }
    
    // turn 180 to get ready for next task
    turnXdegrees(30,1);
    driveXdist(150,0);
    turnXdegrees(150,1);
    CyDelay(500);
    
}

/* Navigate to pucks using ultrasonics and adjustments etc */
void firstNavToPucks(void) {
    
    // drive to corner A
    driveXdist(400,1);
    adjust_dist_US(1,100,100);
    adjust_distances(100,50);
    adjust_angle_US(adjspeed);
    adjust_angle_US(adjspeed);
    
    
    // rotate to drive forwards
    // must be forwards to detect if block is present using front US
    turnXdegrees(90,1);

    // drive forwards, checking US every 5cm
    int i=0;
    int curdist_r=0;
    int curdist_l=0;
    
    while (i<4 && !blockflag) {
        UART_1_PutString("\nloop entered");
        beepXtimes(1);
        CyDelay(1000);
        curdist_r=updateusxtimes_r(10);
        curdist_l=updateusxtimes_l(10);
        CyDelayUs(100);
        if ((curdist_r<=130)||(curdist_l<=130)) { // if block in next 15cm
            UART_1_PutString("\nblock detected");
            beepXtimes(2);
            blockflag=1; // disabled this because it kept detecting block for no reason
            i=4;
        } else {
            driveXdist(50,1);
        }
        CyDelay(400);
        i++;
    }

    // check blockflag; if set, switch to other function
    if (blockflag) {
        beepXtimes(3);
        driveXdist(100,0); //drive backwards
        turnXdegrees(87,1); //might be the wrong direction
        adjust_dist_US(1,250,100); //might need another straight here
        adjust_distances(120,50);
        adjust_angle_US(adjspeed);
        turnXdegrees(87,0); //turn right towards right wall
        driveXdist(200,1);
        adjust_dist_US(1,250,100);
        adjust_distances(110,50);
        adjust_angle_US(adjspeed);
        turnXdegrees(87,0);
    }
    // else; continue on with navigation to pucks i guess?

    if(!blockflag){
    // reach corner B
    driveXdist(200,1);
    adjust_dist_US(1,250,100);
    adjust_distances(110,50);
    adjust_angle_US(adjspeed);
    
    // turn to face row of pucks; must be aligned
    turnXdegrees(87,1);
    }
    // drive back into wall? SLOWLY (then restore bwdspeed to OG value)
    //uint8 tmp=bwdspeed; bwdspeed=60; driveXdist(100,0); bwdspeed=tmp;

}

/* Drive until middle ultrasonic detects puck then pick it up*/
void collectPuck(void) {
    
    int dflag=0;
    uint8 lspeed, rspeed;
    int curdist=0;
    int ccount=0;

    lspeed = 65+2;
    rspeed = 65;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);

    liftClaw(40,1);
    CyDelay(100);

    // set directions
    A3_Write(0); // R
    A4_Write(1);
    A1_Write(0); // L
    A2_Write(1);
    
    // enable middle US
    US_M_EN_Write(1);
    US_SIDEL_EN_Write(0);
    
    while(!dflag) {
        ccount=0;
        curdist=0;
        while (ccount<5){
  
        updateUS();

        curdist=curdist+distance_mid;
        CyDelay(10);
        ccount++;
        }
        
        curdist=curdist/5;
        
        if (!SILENT) {
            UART_1_PutString("\n");
            printNumUART(curdist);
        }
        
        if(curdist<=85) { // when puck is reached; adjust distance value as necessary
            dflag=1;
        }        
             
    }
    
    // stop wheels
    A1_Write(0);
    A2_Write(0);
    A3_Write(0);
    A4_Write(0);            
    PWM_1_WriteCompare1(0);
    PWM_1_WriteCompare2(0);
    CyDelay(500);

    // re-enable sideleft US
    US_M_EN_Write(0);
    US_SIDEL_EN_Write(1);
    
    // open claw, drop then close
    moveServo(sopen);
    liftClaw(40,0);
    CyDelay(100);
    driveXdist(18,1);
    CyDelay(100);
    moveServo(sclose);
    CyDelay(200);
    
    // determine colour
    int col = getColour(1);
    beepXtimes(col);
    
    // lift
    liftClaw(40,1);
        
}

void navToConstruction(void) {
    
    
    if(!blockflag){
    driveXdist(200,0); //reverse into wall
    CyDelay(200);
    
    liftClaw(30,0);
    CyDelay(300);
    
    driveXdist(45,1);
    CyDelay(200);
    
    turnXdegrees(90,1);
    CyDelay(500);

    driveXdist(400,1);
    adjust_dist_US(1,100,100);
    adjust_distances(100,50);
    adjust_angle_US(adjspeed);
    adjust_angle_US(adjspeed);
    }
    else if(blockflag){
        driveXdist(200,0); //reverse into wall
        CyDelay(200);
        
        liftClaw(30,0);
        CyDelay(300);
        
        driveXdist(50,1);
        CyDelay(200);
        
        turnXdegrees(90,0);
        CyDelay(500);
        
        driveXdist(500,1);
        CyDelay(500);
        
        turnXdegrees(90,1);
        CyDelay(500);
        driveXdist(400,1);
        adjust_dist_US(1,100,100);
        adjust_distances(100,50);
        adjust_angle_US(adjspeed);
        adjust_angle_US(adjspeed);
        
        turnXdegrees(90,0);
        CyDelay(500);
        driveXdist(400,1);
        adjust_dist_US(1,100,100);
        adjust_distances(100,50);
        adjust_angle_US(adjspeed);
        adjust_angle_US(adjspeed);
        
    }
}

void stackPuck(void) {}

void navToPucks(void) {
    /* Uses puckcount, prow and pcol to align robot with the next puck to collect */
    // e.g. when puckcount=0; then prow=0, pcol=0; i.e. retrieve rightmost col, top row puck.
    //      when puckcount=1; then prow=1, pcol=0; i.e. retrieve rightmost col, second row puck.
    //      etc.
    //      
    // basically puckcount used to calculate prow and pcol based on hardcoded rules,
    // then together prow and pcol are used by this function to tell robot where to go next
}

void disposePuck(int col) {

    driveXdist(50,0);
    turnXdegrees(20*col,1);
    liftClaw(20,0);
    moveServo(90);
    CyDelay(300);
    liftClaw(20,1);
    moveServo(0);
    turnXdegrees(20*col,0);

}

/* Full task here */
void allTasks(void) {

    readWallPucks();
    firstNavToPucks();

    if (!blockflag) {
        // if path isnt blocked take regular path
        while (!alldone) {

            collectPuck();
            
            int col=getColour(1);
            beepXtimes(col);

            if (col==SEQ[stackcount]) { // if colour matches, stack it then increment stackcount
                navToConstruction();
                stackPuck();
                stackcount++;
                if (stackcount<5) navToPucks();
            } else { // else move it elsewhere and increment puckcount
                disposePuck(col);
                puckcount++;

                // reposition to collect next puck
            }

            if (stackcount==5) alldone=1;

        }

    } else {
        // secondary path
        
        // navToPucks2
        
        // collectPuck()
        
        // etc etc
        
    }


}

/* ===================================================================== */

int main(void)
{   
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    
    // start components and ISRs
    UART_1_Start();
        UART_1_PutString("\nUART started");
        if (SILENT) {
            UART_1_PutString(" but SILENT flag is set (output is suppressed)");
        }    
    COL_COUNTER_Start();
    PWM_1_Start();
    PWM_2_Start();
    SEL_QUAD_Start();
    SER_QUAD_Start();
    SECL_QUAD_Start();
    ISR_QUAD1_StartEx(QUAD1_ISR);
    ISR_QUAD2_StartEx(QUAD2_ISR);
    ISR_COL_StartEx(COL_ISR);
    PWM_SERVO_Start();
    USTimer_R_Start();
    USTimer_L_Start();    
    USTimer_M_Start();
    ISR_US_R_StartEx(Timer_ISR_HandlerR);
    ISR_US_L_StartEx(Timer_ISR_HandlerL);
    ISR_US_M_StartEx(Timer_ISR_HandlerM);
    
    // variables
    int col;
    periodLen = COL_COUNTER_ReadPeriod(); // read length of period register (in clock counts)
    
    // some hardware initialisations 
    S0_Write(0); // 2% colour scaling
    S1_Write(1);
    moveServo(sopen); // open claw
    
    // signal start
    flashXtimes(3);    
    soundPiezo(200);
    CyDelay(2000);
    
    // code here
    firstNavToPucks();
    beepXtimes(3);
    collectPuck();
    navToConstruction();
    beepXtimes(4);
    driveXdist(8,1);
    CyDelay(100);
    moveServo(sopen);
    
    

    
    
    
    for(;;)
    {
        
      
    }
    
}

/* [] END OF FILE */
