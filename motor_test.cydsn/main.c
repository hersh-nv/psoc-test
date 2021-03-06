/* ========================================
DRIVING TESTS

todo:
    - 
 * ========================================
*/

#include "project.h"
#include "stdlib.h"
#include "math.h"

/* Defines for TRUE and FALSE */
#ifndef TRUE
    #define TRUE 1
#endif
#ifndef FALSE
    #define FALSE 0
#endif

#define SILENT FALSE // if set, suppress general UART output
#define DRIVESILENT FALSE // if set, suppress driving UART


// defines for unit-conversion coefficients
#define DIST_COEFF 21
#define ROTATE_COEFF 39
#define PULLEY_DIST_COEFF 48 // TO BE TESTED


/* Defines for puck readings; calibrate when in new environment / lighting */
// sensor 1 : inside claw
uint16 RED[4] = {5000, 5010, 8045, 5560}; // R/N (B sensor) = 1.7
uint16 GRE[4] = {6750, 6770, 6860, 7540}; // G/N (B sensor) = 1.44
uint16 BLU[4] = {7900, 8650, 5650, 7100}; // B/N (B sensor) = 1.19
uint16 NON[4] = {4820, 5300, 4723, 4810};
float NONf[4] = {4820, 5300, 4723, 4810};

// sensor 2 : on wall
uint16 RED2[3] = {1940, 6800, 7040};
uint16 GRE2[3] = {3700, 2300, 4700};
uint16 BLU2[3] = {4000, 4100, 3270};

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
uint8 fwdspeed = 170;
uint8 bwdspeed = 200;
uint8 trnspeed = 120;
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
int SEQ[5]={3,2,1,3,2};
uint16 mindist = 0u;
uint16 periodLen = 0u;   // length of counter period

// flag for block presence (on initial path)
int blockflag=0;

// puck array
uint8 puckcount=0u; // count of how many pucks in arena have been fetched / checked; max of 25?
uint8 prow=0u; // puckrow; this and puckcol are used to program location of next puck to retrieve
uint8 prow2=0u;
uint8 pcol=0u;
uint8 rows[3]={0,52,106};
//uint8 rows2[2]={0,52};

// stacking
int stackcount=0; // count of how many pucks have been stacked; max of 5
int heights[5]={0,24,40,56,71}; // made this global so stackPuck() doesnt have to redefine it each time

// storage count [r,g,b]
int storage[3]={0,0,0};

// servo angles
int16 sopen=75;
int16 sopen2=9;
int16 sclose=-84;

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

/* findMin in array used by getColour(), ripped from
http://www.programmingsimplified.com/c/source-code/c-program-find-minimum-element-in-array*/
int findMin(float a[], int n) {
  int c, min, index;
 
  min = a[0];
  index = 0;
 
  for (c = 1; c < n; c++) {
    if (a[c] < min) {
       index = c;
       min = a[c];
    }
  }
 
  return index;
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

/* Reads the quadrature decoder connected to the shaft encoder and returns the distance as a int32
   Used for all shaft encoders, on wheel motors and pulley motor */
int32 getDistance(int side, int32 startdist) {
    
    int32 distance;
    int32 SE_COUNT = 0u;
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

    uint8 lspeed=0, rspeed=0;
    int32 ldist = 0;
    int32 rdist = 0;
    int32 lsdist=0, rsdist=0;
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
        lspeed = fwdspeed+2;
        rspeed = fwdspeed;
    } else {
        lspeed = bwdspeed+2;
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

        CyDelay(10); // the smaller this value, the more often the SEs are polled and hence the more accurate the distance
        
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
//        if ((ldist-rdist)>10) {
//            lspeed--;
//        } else if ((rdist-ldist)>10) {
//            lspeed++;
//        }
//        PWM_1_WriteCompare1(lspeed); 
//      // ..turned this feature off because it seems to drive straight anyway
        
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

    CyDelay(100);
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
    CyDelay(100);
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
    float distR=0;  //run sum zero
    float distL=0;
    
    //run for 7 iterations summing as you go
    for (int runavg=0;runavg<7;runavg++){
        updateUS();
        CyDelay(5);
        distR=distR+distance_m1;
       // UART_1_PutString("\nd_M1 ");
        //printNumUART(distance_m1);
        distL=distL+distance_m2;
    }
    
    //divide by sum
    distR=distR/7;
    distL=distL/7;
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
    lspeed = speed+2;
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
        for (int runavg=0;runavg<5;runavg++){
            updateUS();
            CyDelay(4);
            distR=distR+distance_m1;
            distL=distL+distance_m2;
            //UART_1_PutString("\nM1  ");
            //printNumUART(distance_m1);
            //UART_1_PutString(" M2  ");
           // printNumUART(distance_m2);
        }
        distR=distR/5;
        distL=distL/5;
        
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

/* Polls colour sensor several times to get a reliable colour reading, and returns a integer value
corresponding with the closest colour reading. Outputs 0 if no colour is strongly detected */
int getColour(int sensor) {
    /* INPUTS */
    // sensor = 1 for claw colour sensor, 2 for side sensor
    
    // initialise
    uint16 rCount=20000;
    uint16 gCount=20000;
    uint16 bCount=20000;
    uint16 nCount=20000;
    uint16 rCountTmp, gCountTmp, bCountTmp, nCountTmp;
    uint16 rDist, gDist, bDist, nDist;
    float dists[4];
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
            CyDelay(3);
           // rCountTmp = (overflowCountCOL * periodLen) + capturedCount;
            rCountTmp = capturedCount;
            rCount = (rCount < rCountTmp) ? rCount : rCountTmp;
        }
        overflowCountCOL = 0u;
    
    S2_Write(1); S3_Write(1); // GREEN
        for (int i=0;i<rep;i++) {
            CyDelay(3);
           // gCountTmp = (overflowCountCOL * periodLen) + capturedCount;
            gCountTmp = capturedCount;
            gCount = (gCount < gCountTmp) ? gCount : gCountTmp;
        }
        overflowCountCOL = 0u;
        
    S2_Write(0); S3_Write(1); // BLUE
        for (int i=0;i<rep;i++) {
            CyDelay(3);
           // bCountTmp = (overflowCountCOL * periodLen) + capturedCount;
            bCountTmp = capturedCount;
            bCount = (bCount < bCountTmp) ? bCount : bCountTmp;
        }
        overflowCountCOL = 0u;
        
    S2_Write(0); S3_Write(0);
        for (int i=0;i<rep;i++) {
            CyDelay(3);
           // bCountTmp = (overflowCountCOL * periodLen) + capturedCount;
            nCountTmp = capturedCount;
            nCount = (nCount < nCountTmp) ? nCount : nCountTmp;
        }
        overflowCountCOL = 0u;
        
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
        UART_1_PutString(" N");
        printNumUART(nCount);
    }
    
    
    // find L1 distance to each puck location
    if (sensor==1) {
        rDist = abs(RED[2]-bCount);// + abs(RED[0]-rCount) + abs(RED[1]-gCount);
            UART_1_PutString("\nDist: R");
            printNumUART(rDist);
        gDist = abs(GRE[2]-bCount);// + abs(GRE[0]-rCount) + abs(GRE[1]-gCount);
            UART_1_PutString(" G");
            printNumUART(gDist);
        bDist = abs(BLU[2]-bCount);// + abs(BLU[0]-rCount) + abs(BLU[1]-gCount);
            UART_1_PutString(" B");
            printNumUART(bDist);
        nDist = abs(NON[2]-bCount);// + abs(NON[0]-rCount) + abs(NON[1]-gCount);
            UART_1_PutString(" N");
            printNumUART(nDist);
        dists[0]=nDist; dists[1]=rDist; dists[2]=gDist, dists[3]=bDist;
    } else if (sensor==2) {
        rDist = abs(RED2[2]-bCount); // + abs(RED2[0]-rCount) + abs(RED2[1]-gCount);
            UART_1_PutString("\nDist: R");
            printNumUART(rDist);
        gDist = abs(GRE2[2]-bCount);// + abs(GRE2[0]-rCount) + abs(GRE2[1]-gCount);
            UART_1_PutString(" G");
            printNumUART(gDist);
        bDist = abs(BLU2[2]-bCount);// + abs(BLU2[0]-rCount) + abs(BLU2[1]-gCount);
            UART_1_PutString(" B");
            printNumUART(bDist);
        dists[0]=(float) rDist+1; dists[1]=(float)rDist; dists[2]=(float)gDist, dists[3]=(float)bDist;
    }
    
    min=findMin(dists, 4);
    
    if (!SILENT) {
        UART_1_PutString("\n");
        printNumUART(dists[min]);
        UART_1_PutString("\n Colour: ");
        if (min==0) {
            UART_1_PutString("None");
        } else if (min==1) {
            UART_1_PutString("R ");
        } else if (min==2) {
            UART_1_PutString("G ");
        } else if (min==3) {
            UART_1_PutString("B ");
        }
    }
    
    return min;
}

/* Rewrote colour sensor algorithm in a way that should be robust to ambient light
    Just make sure calibrateSensor() is used in a new environment over no puck
    calibrateSensor() is only needed/used for getColourv2
    also, getColourv2 is only needed/possible for front sensor; side can still use v1*/
void calibrateSensor(void) {
    
    float bCount=20000;
    uint16 bCountTmp;
    int rep=12;
    
    COL2EN_Write(0);
    COL1EN_Write(1);
    
    moveServo(sclose); CyDelay(500);
    
    S2_Write(0); S3_Write(1); // BLUE
        for (int i=0;i<rep;i++) {
            CyDelay(5);
            bCountTmp = capturedCount;
            bCount = (float) (bCount < bCountTmp) ? bCount : bCountTmp;
        }
        overflowCountCOL = 0u;
        
    NONf[2]=bCount;
    
    UART_1_PutString("\n\nSensor calibration value: ");
    printNumUART(bCount);
    
    // four fast beeps indicate colour calibration
    for (int i=0;i<4;i++) {
        soundPiezo(50);
        CyDelay(50);
    }
    
}
int getColourv2(int sensor) {
    /* INPUTS */
    // sensor = 1 for claw colour sensor, 2 for side sensor
    
    // initialise
    float bCount=20000;
    uint16 bCountTmp;
    float rDist, gDist, bDist, nDist;
    float dists[4];
    int temp, min;
    int rep = 12;
    float ratio;
    
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
    
    
    S2_Write(0); S3_Write(1); // BLUE
        for (int i=0;i<rep;i++) {
            CyDelay(5);
            bCountTmp = capturedCount;
            bCount = (float) (bCount < bCountTmp) ? bCount : bCountTmp;
        }
        overflowCountCOL = 0u;
        
    // disable output
    COL1EN_Write(0);
    COL2EN_Write(0);
    
    // print measures
    if (!SILENT) {
        UART_1_PutString("\n\nSens: B");
        printNumUART(bCount);
    }
    
    // calculate bCount/bCount[NON] ratio
    ratio = (float) (bCount)/(NONf[2]);
    UART_1_PutString("\nRatio ");
    printNumUART(ratio*100);
    
    // find ratio's closest match
    if (sensor==1) {
        rDist = 100.0*fabs(1.68-ratio);
            UART_1_PutString("\nDist: R");
            printNumUART(rDist);
        gDist = 100.0*fabs(1.42-ratio);
            UART_1_PutString(" G");
            printNumUART(gDist);
        bDist = 100.0*fabs(1.10-ratio);
            UART_1_PutString(" B");
            printNumUART(bDist);
        nDist = 100.0*fabs(1-ratio);
            UART_1_PutString(" N");
            printNumUART(nDist);
        dists[0]=nDist; dists[1]=rDist; dists[2]=gDist, dists[3]=bDist;
//    } else if (sensor==2) {
//        rDist = abs(RED2[0]-rCount) + abs(RED2[1]-gCount) + abs(RED2[2]-bCount);
//            UART_1_PutString("\nDist: R");
//            printNumUART(rDist);
//        gDist = abs(GRE2[0]-rCount) + abs(GRE2[1]-gCount) + abs(GRE2[2]-bCount);
//            UART_1_PutString(" G");
//            printNumUART(gDist);
//        bDist = abs(BLU2[0]-rCount) + abs(BLU2[1]-gCount) + abs(BLU2[2]-bCount);
//            UART_1_PutString(" B");
//            printNumUART(bDist);
//        dists[0]=rDist+1; dists[1]=rDist; dists[2]=gDist, dists[3]=bDist;
    }
    
    min=findMin(dists, 4);
    
    if (!SILENT) {
        UART_1_PutString("\n");
        printNumUART(dists[min]);
        UART_1_PutString("\n Colour: ");
        if (min==0) {
            UART_1_PutString("None");
        } else if (min==1) {
            UART_1_PutString("R ");
        } else if (min==2) {
            UART_1_PutString("G ");
        } else if (min==3) {
            UART_1_PutString("B ");
        }
    }
    
    return min;
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

//        CyDelay(5); // the smaller this value, the more often the SEs are polled and hence the more accurate the distance
        
        // get relative pulley travel distance
        pdist = (getDistance(2,psdist)); 
        
        if (!DRIVESILENT) {
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
    
    CyDelay(10);
    
}

/* 'Reset' claw to ground position using middle ultrasonic */
// IMPORTANT: never use resetClaw while there's a puck in front at
// claw distance; this'll pull the claw up indefinitely
void resetClaw(void) {
    
    int done=0;
    int16 dist;
    int thres=100;
    
    // enable middle u/s
    US_SIDEL_EN_Write(0);
    US_M_EN_Write(1);
    
    
    // get initial reading
    dist=0;
    for (int runavg=0; runavg<4; runavg++) {
        updateUS();
        CyDelay(5);
        dist=dist+distance_mid;
    }
    dist=dist/4;
    
    UART_1_PutString("\ninitial dist ");
    printNumUART(distance_mid);
    
    // if less than threshold, lift claw out of the way?
    if (dist<thres) {
        
        UART_1_PutString("\nlifting");
        
        PWM_2_WriteCompare(pllspeed);
        A5_Write(0);
        A6_Write(1);
        
        while(!done) {
        
            dist=0;
            for (int runavg=0; runavg<4; runavg++) {
                updateUS();
                CyDelay(5);
                dist=dist+distance_mid;
            }
            dist=dist/4;
            
            if (dist>thres) {
                CyDelay(500); // wait a bit to get it fully out of the way
                A5_Write(0);
                A6_Write(0);
                done=1;
            }
        }
        
        CyDelay(20);        
    }
    
    // then drop claw
    PWM_2_WriteCompare(37);
    A5_Write(1);
    A6_Write(0);
    
    done=0;
    UART_1_PutString("\ndropping");
    while(!done) {
        
        
        
        dist=0;
        for (int runavg=0; runavg<4; runavg++) {
            updateUS();
            CyDelay(5);
            dist=dist+distance_mid;
        }
        dist=dist/4;
        
        if (dist<thres) {
            A5_Write(0);
            A6_Write(0);
            done=1;
        }
    }

    // drop another 25mm to get to ground position
    liftClaw(15,0);
    
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

int updateusxtimes_m(int rep){
    
    int ccount=0;
    int cdist_m=0;
    
    while(ccount<rep){
  
        updateUS();
        CyDelay(5);
        
        cdist_m=distance_mid+cdist_m;
        ccount++;
        
        UART_1_PutString("\ncdist iteration:");
        printNumUART(cdist_m);
    }
    
    UART_1_PutString("\ncdist total:");
    printNumUART(cdist_m);
    UART_1_PutString("\n");
    
    cdist_m=cdist_m/rep;
    return cdist_m;
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

/* Positions the robot in x/y space in a corner */
void checkCorner(int dist1, int dist2, int dir) {
    // dist1 = distance to wall that the robot is facing
    // dist2 = distance to adjacent wall of corner
    // dir = direction to turn to face dist2 wall; 1=cw, 0=ccw;
    
    int doublecheck=0;
    int checkflag=0;
    
    turnXdegrees(90,dir);
    
    adjust_distances(dist2,85);
    
    while(checkflag==0){
    
        
        adjust_distances(dist2,46);
        
        doublecheck=updateusxtimes_r(20)-105;
        
        UART_1_PutString("\n\n val\n");
        printNumUART(doublecheck);
        
        doublecheck=(doublecheck-dist2)*(doublecheck-dist2);
        UART_1_PutString("\n\n Error\n");
        printNumUART(doublecheck);
        
        if (doublecheck<=5){checkflag=1;}
    }
        
    
    turnXdegrees(90,!dir);
    
    checkflag=0;
    
    adjust_distances(dist1,85);
    
    while(checkflag==0){

        adjust_distances(dist1,46);

        doublecheck=updateusxtimes_r(20)-105;
        
        UART_1_PutString("\n\n val\n");
        printNumUART(doublecheck);
        
        doublecheck=(doublecheck-dist1)*(doublecheck-dist1);
        UART_1_PutString("\n\n Error\n");
        printNumUART(doublecheck);
        
        if (doublecheck<=5){checkflag=1;}
        
    }
    CyDelay(10);
    
}

/* Prelim comp tasks */
void task4(int sensor) {
    // Prelim Task 4
    // NOTE: this enters an infinite loop by design; make an exit flag if you want to do something after Task 4
    // NOTE: now used as a generic infinite colour sensing loop; therefore set sensor number as input
    // sensor = 1 for claw colour sensor, 2 for side colour sensor
    
    flashXtimes(4);
    
//    S0_Write(0); // 2% scaling?
//    S1_Write(1);
    
    int col;
    
    for(;;) {   
        if (sensor==1) {
            col = getColourv2(1);
        } else {
            col = getColour(2);
        }
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
        
//    /* SCALE HERE */
//    S0_Write(0); // 2% scaling?
//    S1_Write(1);
    
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
        
        // sometimes it reads no colour; set to blue as default i guess
        if (col==0) {col=2;}
        
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
//        SEQ[4-i]=col; // reads top->bottom so fill SEQ backwards 
        
        // drive to next colour
        if (i<4) {
            driveXdist(59,1);
        }
        
        // wait
        CyDelay(700);
        
        
        
    }
    
    flashXtimes(3);
    
    
    UART_1_PutString("\n SEQ: ");
    for (int i=0; i<5; i++) {
        printNumUART(SEQ[i]);
        UART_1_PutString("  ");
    }
    
    
}

/* Drive backward along arena checking for puck; sets blockflag if necessary */
void checkForBlock(void) {
    
    // move away from wall
    beepXtimes(2);
    turnXdegrees(30,1);
    driveXdist(350,0);
    turnXdegrees(30,0);
//    turnXdegrees(150,1);
//    CyDelay(500);
    
    // go to side wall and get ready to drive back while checking for block
    
    driveXdist(200,1);
    adjust_distances(20,120);
    adjust_distances(20,50);

    driveXdist(450,0); // drive first half without checking
    beepXtimes(1);     // signal that the blockcheck is about to start
    CyDelay(500);
    
    // then implement driveXdist(300,0) with added side-left US check
    // first enable sideUS
    US_M_EN_Write(0);
    US_SIDEL_EN_Write(1);
    
    int32 Xdist=300;
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
    lspeed = bwdspeed;
    rspeed = bwdspeed;
    PWM_1_WriteCompare1(lspeed);
    PWM_1_WriteCompare2(rspeed);
    
    // set directions
    A3_Write(1); // R
    A4_Write(0);
    // CyDelay(12); // COMPENSATION DELAY; ADJUST AS NECESSARY
    A1_Write(1); // L
    A2_Write(0);

    
    // poll SEs every 20ms, when both shaft encoders read X distance, stop
    while ((!Ldone)||(!Rdone)) {
        
        // get relative distance from both wheels
        ldist = (getDistance(1,lsdist)); //left
        rdist = (getDistance(0,rsdist)); //right
        
        // get side-left US reading
        updateUS();
        CyDelay(5);
        if (distance_mid<90) {     // if block detected in next 20cm then set blockflag?
            LED1_Write(1);
            PIEZO_Write(1);
            //blockflag=1;
        } else {
            LED1_Write(0);
            PIEZO_Write(0);
        }
        
        if (!DRIVESILENT) {
            UART_1_PutString("\nLdist = ");
            printNumUART(ldist);
            UART_1_PutString("  Rdist = ");
            printNumUART(rdist);
        }
        
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

    CyDelay(1000);
    if (blockflag) {
        beepXtimes(2);
    } else {beepXtimes(1);}
}

/* Navigate to pucks using ultrasonics and adjustments etc */
void firstNavToPucks(void) {
    
    // drive to corner A

//    checkForBlock();
//    turnXdegrees(30,1);
//    driveXdist(150,1);
//    turnXdegrees(210,0);
//    driveXdist(250,1);
//    
    
    /* if no checkForBlock()*/
    
    
    adjust_dist_US(1,100,100);
    adjust_distances(100,50);
    adjust_angle_US(adjspeed);
    adjust_angle_US(adjspeed);
    
    
    // rotate to drive forwards
    // must be forwards to detect if block is present using front US
    turnXdegrees(88,1);
    

    // drive forwards, checking US every 5cm
    int i=0;
    int curdist_r=0;
    int curdist_l=0;
    
    while (i<8 && !blockflag) {
        UART_1_PutString("\nloop entered");
        
        CyDelay(400);
        turnXdegrees(15,1);
        
        curdist_r=updateusxtimes_r(10);
        curdist_l=updateusxtimes_l(10);
        CyDelayUs(100);
        turnXdegrees(15,0);
        
        if ((curdist_r<=200)||(curdist_l<=200)) { // if block in next 15cm
            UART_1_PutString("\nblock detected");
            beepXtimes(2);
            blockflag=1; // disabled this because it kept detecting block for no reason
        } else {
            beepXtimes(1);
            driveXdist(25,1);
        }
        
        CyDelay(400);
        i++;
    }

    // check blockflag; if set, switch to other function
    if (blockflag) {
        beepXtimes(3);
        driveXdist(100,0); //drive backwards
        turnXdegrees(87,1); //might be the wrong direction
        driveXdist(400,1);
        adjust_dist_US(1,250,100); //might need another straight here
        adjust_angle_US(adjspeed);
        adjust_distances(120,50);
        turnXdegrees(87,0); //turn right towards right wall
        driveXdist(200,1);
//        adjust_dist_US(1,250,100);
//        adjust_angle_US(adjspeed);
//        adjust_distances(110,50);
        checkCorner(122-rows[2-prow2],60,0);
        checkCorner(122-rows[2-prow2],60,0);
        turnXdegrees(87,0);
    }
    // else; continue on with navigation to pucks i guess?

    if(!blockflag){
        // reach corner B
        driveXdist(200,1);
        checkCorner(115-rows[prow],60,0); // third row first then two then one
        turnXdegrees(88,1);
    }
    // drive back into wall? SLOWLY (then restore bwdspeed to OG value)
    //uint8 tmp=bwdspeed; bwdspeed=60; driveXdist(100,0); bwdspeed=tmp;

}

/* Drive until middle ultrasonic detects puck then pick it up
   Returns the colour of the puck [1/2/3]=[r/g/b] as int */
int collectPuck(void) {
    
    int dflag=0;
    uint8 lspeed, rspeed;
    int curdist=0;
    int ccount=0;
    
    if (!blockflag) {

        lspeed = 70+1;
        rspeed = 70;
        PWM_1_WriteCompare1(lspeed);
        PWM_1_WriteCompare2(rspeed);
        
        resetClaw();
        moveServo(sclose);
        calibrateSensor();
        liftClaw(55,1);
        CyDelay(100);

        // set directions
        A3_Write(0); // R
        A4_Write(1);
        A1_Write(0); // L
        A2_Write(1);
        
        beepXtimes(prow);
        CyDelay(800);
        beepXtimes(pcol);
        
    //    driveXdist(60,0);
        
        // enable middle US
        US_M_EN_Write(1);
        US_SIDEL_EN_Write(0);
        
        while(!dflag) {
            ccount=0;
            curdist=0;
            while (ccount<5){
                updateUS();
                CyDelay(5);
                curdist=curdist+distance_mid;
                CyDelay(10);
                ccount++;
            }
            
            curdist=curdist/5;
            
            if (!SILENT) {
                UART_1_PutString("\n");
                printNumUART(curdist);
            }
            
            if(curdist<=130) { // when puck is reached; adjust distance value as necessary
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
        CyDelay(700);
        
        curdist=updateusxtimes_m(20);

        // re-enable sideleft US
        US_M_EN_Write(0);
        US_SIDEL_EN_Write(1);
        
        // open claw, drop, drive forward to puck then close
        moveServo(sopen);
        liftClaw(55,0);
        driveXdist(curdist-70,1);
        moveServo(sclose);
        CyDelay(200);
        
        // determine colour
        int col = getColourv2(1);
        beepXtimes(col);
        CyDelay(400);
        
        // lift a little bit so puck doesn't drag
        //liftClaw(5,1);
        
        return col;
    }
    else {
        lspeed = 70+1;
        rspeed = 70;
        PWM_1_WriteCompare1(lspeed);
        PWM_1_WriteCompare2(rspeed);
        
        resetClaw();
        moveServo(sclose);
        calibrateSensor();
        liftClaw(55,1);
        CyDelay(100);

        // set directions
        A3_Write(0); // R
        A4_Write(1);
        A1_Write(0); // L
        A2_Write(1);
        
        beepXtimes(prow);
        CyDelay(800);
        beepXtimes(pcol);
        
    //    driveXdist(60,0);
        
        // enable middle US
        US_M_EN_Write(1);
        US_SIDEL_EN_Write(0);
        
        while(!dflag) {
            ccount=0;
            curdist=0;
            while (ccount<5){
                updateUS();
                CyDelay(5);
                curdist=curdist+distance_mid;
                CyDelay(10);
                ccount++;
            }
            
            curdist=curdist/5;
            
            if (!SILENT) {
                UART_1_PutString("\n");
                printNumUART(curdist);
            }
            
            if(curdist<=150) { // when puck is reached; adjust distance value as necessary
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
        CyDelay(700);
        
        curdist=updateusxtimes_m(20);

        // re-enable sideleft US
        US_M_EN_Write(0);
        US_SIDEL_EN_Write(1);
        
        // open claw, drop, drive forward to puck then close
        moveServo(sopen2);
        liftClaw(55,0);
        driveXdist(curdist-70,1);
        moveServo(sclose);
        CyDelay(200);
        
        // determine colour
        int col = getColourv2(1);
        beepXtimes(col);
        CyDelay(400);
        
        // lift a little bit so puck doesn't drag
        //liftClaw(5,1);
        
        return col;
    }         
}

void navToConstruction(void) {
    
    
    if(!blockflag){
    driveXdist(150+55*pcol,0); //reverse into wall
    CyDelay(500);
    
//    liftClaw(30,0);
//    CyDelay(300);
//    
//    driveXdist(45,1);
//    CyDelay(200);
//    
    
    if (prow<2) {
        turnXdegrees(90,1);
    } else {
        turnXdegrees(90,0);
        driveXdist(100,0);
        turnXdegrees(180,1);
    }
    CyDelay(500);

    driveXdist(480+60*prow,1);
//    adjust_dist_US(1,100,100);
//    adjust_angle_US(adjspeed);
//    adjust_angle_US(adjspeed);
//    adjust_distances(100,50);
    
    } else {
        
        driveXdist(150+55*pcol,0); //reverse into wall
        
        if (prow<2) {
            turnXdegrees(90,0);
        } else {
            turnXdegrees(90,1);
            driveXdist(100,0);
            turnXdegrees(180,0);
        }

        driveXdist(500,1);

        turnXdegrees(90,1);
        CyDelay(500);
        driveXdist(400,1);
        adjust_dist_US(1,60,fwdspeed);
        adjust_distances(60,50);
        
        turnXdegrees(90,0);
        CyDelay(500);
        
    }
    
    checkCorner(130,50,1);
    CyDelay(500);
    checkCorner(130,50,1);
    
    resetClaw();
    
}

void stackPuck(void) {
    
    // lift claw to correct height based on number of 
    // currently stacked pucks
    liftClaw(heights[stackcount],1);
    
    CyDelay(100);
    uint8 temp=fwdspeed; fwdspeed=90; driveXdist(100,1); fwdspeed=temp;
    CyDelay(500);
    
    // open
    moveServo(sopen);
    CyDelay(500);
    
    // move back out
    driveXdist(100,0);
    moveServo(sclose);
    resetClaw();

}

void navToPucks(void)   {
    /* Uses puckcount, prow and pcol to align robot with the next puck to collect */
    // e.g. when puckcount=0; then prow=0, pcol=0; i.e. retrieve rightmost col, top row puck.
    //      when puckcount=1; then prow=1, pcol=0; i.e. retrieve rightmost col, second row puck.
    //      etc.
    //      
    // basically puckcount used to calculate prow and pcol based on hardcoded rules,
    // then together prow and pcol are used by this function to tell robot where to go next
    
    // calculate puckrow (and puckcol?)
    
    
    if (!blockflag) {
        
        prow=puckcount%3; // collect from the first three rows
        pcol=puckcount/3;
        
        turnXdegrees(180,1);
        driveXdist(500,1);
        beepXtimes(1);
        adjust_dist_US(1,250,100);
        beepXtimes(2);
        checkCorner(115-rows[prow],60,0);
        checkCorner(115-rows[prow],60,0);
        turnXdegrees(87,1);
    } else {
        
        prow=puckcount%2; // collect from the first three rows
        pcol=puckcount/2;
        
        turnXdegrees(89,0);
        driveXdist(500,1);
        beepXtimes(1);
        adjust_dist_US(1,250,100);
        adjust_distances(60,50);
        
        turnXdegrees(89,0);
        driveXdist(500,1);
        beepXtimes(1);
        adjust_dist_US(1,120,0);
        checkCorner(122-rows[2-prow],60,1);
        checkCorner(122-rows[2-prow],60,1);
        turnXdegrees(87,0);
    }
}

void storePuck(int col) {

    if (col!=0) {
        
        
        turnXdegrees(30+col*30,0);
        
        liftClaw(heights[storage[col-1]],1);
        driveXdist(160,1);

        moveServo(sopen);
        storage[col-1]++;
        CyDelay(100);
        
        driveXdist(160,0);
        moveServo(sclose);
        resetClaw();
        turnXdegrees(30+col*30,1);
    }

}

// retrieve puck from storage
void unstorePuck(int col) {
    
    turnXdegrees(33+col*30,0);
    moveServo(sopen);
    CyDelay(100);
    
    liftClaw(heights[storage[col-1]-1],1);
    driveXdist(160,1);
    moveServo(sclose);
    CyDelay(400);
    liftClaw(6,1);
    
    driveXdist(160,0);
    resetClaw();
    turnXdegrees(33+col*30,1);
    
    storage[col-1]--;
}

void disposePuck() {
    
    moveServo(sopen); CyDelay(500);
    
    if (!blockflag) {
        prow=puckcount%3; // collect from the first three rows
        pcol=puckcount/3;
        
        driveXdist(150+55*pcol,0);
        turnXdegrees(90,0);
        checkCorner(115-rows[prow],60,0);
        turnXdegrees(87,1);
    } else {
        prow=puckcount%2; // collect from the first three rows
        pcol=puckcount/2;
        
        driveXdist(150+55*pcol,0);
        turnXdegrees(90,1);
        checkCorner(122-52*rows[prow],60,1);
        turnXdegrees(87,0);
    }
}

/* Full task here */
void allTasks(void) {

    int col;
    int exit=0;
    int gotocon=0; // flag for going to construction

    
    resetClaw();
    
//    liftClaw(10,1);
//    readWallPucks();
//    
//    beepXtimes(2);
//    turnXdegrees(30,1);
//    driveXdist(250,0);
//    turnXdegrees(210,0);
    
    firstNavToPucks();

//    if (!blockflag) {
        
        // if path isnt blocked take regular path
        
        
        // stack pucks, then retrieve more either from array or storage;
        // this loop continues for the next four pucks
        while (!alldone) {
            
            while (!gotocon) {
                gotocon=0;
                col=collectPuck();
                puckcount++;
                for (int i=stackcount;i<5;i++) {
                    if (col==SEQ[i]) {
                        gotocon=1;
                    }
                }
                if (gotocon) {
                    navToConstruction();
                } else {
                    disposePuck();
                }
            }
            gotocon=0;
            
        
//          /* Comment above block and uncomment this block to test un/storage */
//            moveServo(sopen); CyDelay(3000);
//            moveServo(sclose); CyDelay(500);
//            col=getColourv2(1);
//            beepXtimes(col);
//            checkCorner(130,45,1);
//          /* ***************************************** */
            
            while (!exit) {
                
                if (col==SEQ[stackcount]) { // if colour matches, stack it then increment stackcount
                    stackPuck();
                    stackcount++;
                    CyDelay(400);
                } else { // else move it elsewhere and increment puckcount
                    storePuck(col);
                }
                
                if (storage[SEQ[stackcount]-1]>0) { // if next colour is already stored, retrieve it
                    unstorePuck(SEQ[stackcount]);
                    checkCorner(130,50,1);
                    checkCorner(130,50,1);
                    col=SEQ[stackcount];
                } else {
                    exit=1;
                }
            }
            exit=0;

            if (stackcount==5) { // if finished, exit loop
                alldone=1;
            } else {
                navToPucks();
            }
        }

//    } else {
        // secondary path
        
        // navToPucks2
        
        // collectPuck()
        
        // etc etc
        
//    }


}

/* ===================================================================== */

int main(void) {   
    
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
    moveServo(sclose); // open claw
    
    // signal start
    flashXtimes(3);    
    soundPiezo(200);
    CyDelay(2000);
    
    //** CODE HERE **//
    allTasks();
//    resetClaw();
    
//    turnXdegrees(30,1);
//    for (;;) {
//        driveXdist(100,1);
//        driveXdist(100,0);
//        turnXdegrees(60,0);
//        driveXdist(100,1);
//        driveXdist(100,0);
//        turnXdegrees(60,1);
//    }
    
//   
//    resetClaw();
//    for (;;) {
//        checkCorner(110-rows[2-prow],60,1);
//        turnXdegrees(87,0);
//        CyDelay(2000);
//        col=collectPuck();
//        puckcount++;
//        prow=puckcount%3;
//        
//        CyDelay(2000);
//        moveServo(sopen);
//        CyDelay(6000);
//        moveServo(sclose);
//        CyDelay(2000);
//
//    }
// 
        
    
//    resetClaw(); CyDelay(1000);
//    calibrateSensor();
//    checkCorner(150,80,1); CyDelay(500);
//    unstorePuck(1); stackPuck();
//    unstorePuck(2); stackPuck();
    
//    for (int i=0;i<6;i++) {
//        moveServo(sopen); CyDelay(4000);
//        moveServo(sclose); CyDelay(200);
//        col=getColourv2(1);
//        beepXtimes(col);
//        checkCorner(150,80,1); CyDelay(1000);
//        storePuck(col);
//    }
    
    
//    
//    for (int i=0;i<5;i++) {
//        moveServo(sopen);
//        CyDelay(5000);
//        
//        moveServo(sclose);
//        CyDelay(2000);
//        
//        checkCorner(130,50,1);
//    
//        resetClaw();
//        
//        stackPuck();
//        stackcount++;
//        
//        resetClaw();
//    }
//    
//    resetClaw();
//    calibrateSensor();
//    moveServo(sopen); CyDelay(4000);
//      
//    US_SIDEL_EN_Write(1);
//    US_M_EN_Write(0);
//    
//    
    for(;;)
    {
//        updateusxtimes_m(7);
//        CyDelay(10);
//        
////                
//        turnXdegrees(90,1);
//        CyDelay(1000);
    }
    
}

/* [] END OF FILE */
