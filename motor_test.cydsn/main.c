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
uint16 RED[3] = {6400, 6400, 9300};
uint16 GRE[3] = {8600, 7300, 7300};
uint16 BLU[3] = {10000, 9200, 5500};

uint16 redc=250;
uint16 greenc=60;
uint16 bluec=70;


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
uint8 adjspeed=36;

// US sensors
uint16 uscount1=0;
float distance_m1=0;
uint16 uscount2=0;
float distance_m2=0;

// for storing colour sequence
int8 SEQ[5];



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

        CyDelay(100); // the smaller this value, the more often the SEs are polled and hence the more accurate the distance
        
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

/* Write to all ultrasonic TRIG pins, which should update all their distance values through respective ISRs. */
void updateUS(void) {
    
    //right
    TRIG_R_Write(1); TRIG_L_Write(0);
    CyDelayUs(10);
    TRIG_R_Write(0); TRIG_L_Write(0);
    
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
    int runavg=0;
    float dist1a=0;
    float dist2a=0;
    
    // read US
    while(runavg<=10){
    TRIG_R_Write(1); 
    TRIG_L_Write(1);
    CyDelayUs(10);
    TRIG_R_Write(0); TRIG_L_Write(0);
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
    
        runavg=0;
        dist1a=0;
        dist2a=0;

    // poll USs, stop when both wheels have rotated enough
    while (!done) {
    
        while(runavg<8){    
        // set trigger so distances update
        TRIG_R_Write(1); TRIG_L_Write(1);
        CyDelayUs(10);
        TRIG_R_Write(0); TRIG_L_Write(0);
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
        
//        UART_1_PutString("\nR:  ");
//        printNumUART(dist1a);
//        UART_1_PutString("  L:  ");
//        printNumUART(dist2a);
//        UART_1_PutString("\n");
        
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
};

/* Polls colour sensor several times at 100us intervals to get a reliable colour reading, and returns a integer value
corresponding with the closest colour reading. Outputs 0 if no colour is strongly detected */
int getColour(uint16* periodLen, uint16 dist) {
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
            CyDelay(2);
            rCountTmp = (overflowCountCOL * *periodLen) + capturedCount;
//            rCountTmp = capturedCount;
            rCount = (rCount < rCountTmp) ? rCount : rCountTmp;
        }
        overflowCountCOL = 0u;
    
    S2_Write(1); S3_Write(1); // GREEN
        for (int i=0;i<rep;i++) {
            CyDelay(2);
            gCountTmp = (overflowCountCOL * *periodLen) + capturedCount;
//            gCountTmp = capturedCount;
            gCount = (gCount < gCountTmp) ? gCount : gCountTmp;
        }
        overflowCountCOL = 0u;
        
    S2_Write(0); S3_Write(1); // BLUE
        for (int i=0;i<rep;i++) {
            CyDelay(2);
            bCountTmp = (overflowCountCOL * *periodLen) + capturedCount;
//            bCountTmp = capturedCount;
            bCount = (bCount < bCountTmp) ? bCount : bCountTmp;
        }
        overflowCountCOL = 0u;
        
    S2_Write(0); S3_Write(0); 
    
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
    dist = (bDist < temp) ? bDist : temp;
    min = (bDist < temp) ? 3 : 2;
    if (min==2) {
        min = (rDist < gDist) ? 1 : 2;
    }
    UART_1_PutString("\n");
    printNumUART(dist);
    if (dist<3500) {
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
    
    uint16 duty, oldduty;
    
    oldduty = PWM_SERVO_ReadCompare();
    
    duty = 4500 + 16*angle; // range from 3k to 6k duty cycle
                            // pwm period = 60k, 20ms so this corresponds 
                            // 1ms to 2ms range
//    for (int i=0; i<8; i++) {
//        PWM_SERVO_WriteCompare(oldduty+i*((duty-oldduty)>>3));
//        CyDelay(100);
//    }
    PWM_SERVO_WriteCompare(duty);
    
    UART_1_PutString("\nduty  ");
    printNumUART(duty);
}

/* Flashes the PSoC LED (pin 2[1]) X number of times with a 400ms period, useful for very basic signalling without UART */
void flashXtimes(int rep) {
    
    for (int i=0;i<rep;i++) {
        LED1_Write(1);
        CyDelay(100);
        LED1_Write(0);
        CyDelay(100);
    }
    
}

/* Sound the piezo for X seconds */
void soundPiezo(int sec) {
    
    PIEZO_Write(1);
    CyDelay(sec*1000);
    PIEZO_Write(0);
    
}

/* Prelim comp tasks */
void task1() {
    // Prelim Task 1
    flashXtimes(1);
    
    driveXdist(60,1,fwdspeed); // units = cm
    CyDelay(300);
    
    driveXdist(60,0,bwdspeed); //units = cm
    
    flashXtimes(1);
}
void task2() {
    // Prelim Task 2
    flashXtimes(2);
    
    driveXdist(60,1,fwdspeed); CyDelay(200);
    turnXdegrees(180,1,fwdspeed); CyDelay(200);
    adjust_angle_US(adjspeed); CyDelay(200);
    adjust_angle_US(adjspeed); CyDelay(200);
    adjust_angle_US(adjspeed); CyDelay(200);
    
    //driveXdist(50+15,1,fwdspeed);
    
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
    driveXdist(20,1,fwdspeed);
    CyDelay(100);
    
    //right
    turnXdegrees(90,1,trnspeed);
    CyDelay(100);
    //stright
    driveXdist(50,1,fwdspeed);
    CyDelay(100);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    
    //left
    turnXdegrees(90,0,trnspeed);
    CyDelay(100);
    //stright
    driveXdist(75,1,fwdspeed);
    CyDelay(100);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    
    //left
    turnXdegrees(90,0,trnspeed);
    CyDelay(100);
    //stright
    driveXdist(90,1,fwdspeed);
    CyDelay(100);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    
    //left 
    turnXdegrees(90,0,trnspeed);
    CyDelay(100);
    //straight
    driveXdist(75,1,fwdspeed);
    CyDelay(100);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    
    //left
    turnXdegrees(90,0,trnspeed);
    CyDelay(100);
    //straight
    driveXdist(43,1,fwdspeed);
    CyDelay(100);
    
    //right
    turnXdegrees(90,1,trnspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed);
    CyDelay(100);
    adjust_angle_US(adjspeed); //second adjustment
    CyDelay(100);
    //straight
    driveXdist(33,1,fwdspeed);
    
    flashXtimes(3);
}
void task4(uint16 periodLen) {
    // Prelim Task 4
    // NOTE: this enters an infinite loop by design; make an exit flag if you want to do something after Task 4
    flashXtimes(4);
    
    S0_Write(0); // 2% scaling?
    S1_Write(1);
    
    int col;
    uint16 dist;
    
    for(;;) {   
        col = getColour(&periodLen, dist);
        if (!SILENT) {
            UART_1_PutString("\nColour: ");
            if (col==0) {
                UART_1_PutString("None");
                LEDR_Write(0);
                LEDG_Write(0);
                LEDB_Write(0);
            } else if (col==1) {
                UART_1_PutString("R ");
                LEDR_Write(1);
                LEDG_Write(0);
                LEDB_Write(0);
            } else if (col==2) {
                UART_1_PutString("G ");
                LEDR_Write(0);
                LEDG_Write(1);
                LEDB_Write(0);
            } else if (col==3) {
                UART_1_PutString("B ");
                LEDR_Write(0);
                LEDG_Write(0);
                LEDB_Write(1);
            }
        }
        CyDelay(50);
    }
    
}

void task3g(){
flashXtimes(3);
    //straight
    adjust_dist_US(1,12,fwdspeed);
    CyDelay(200);
    
    //adjust?
    /*adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);*/
    
    //right
    turnXdegrees(90,1,trnspeed);
    CyDelay(200);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    
    //straight
    adjust_dist_US(1,17,fwdspeed);
    CyDelay(200);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);


    //left1
    turnXdegrees(95,0,trnspeed);
    CyDelay(200);
    
    //adjust    
    /*adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);*/
    
    //straight
    adjust_dist_US(1,17,fwdspeed);
    CyDelay(200);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    
    //left2
    turnXdegrees(98,0,trnspeed);
    CyDelay(200);
    
    //adjust
    /*adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);*/
    
    //straight
    adjust_dist_US(1,15,fwdspeed);
    CyDelay(200);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);


    //left3
    turnXdegrees(97,0,trnspeed);
    CyDelay(200);
    
    
    //adjust
    /*adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);*/
    
    //straight
    adjust_dist_US(1,20,fwdspeed);
    CyDelay(200);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);

    
    //left 4
    turnXdegrees(95,0,trnspeed);
    CyDelay(200);
    
    //adjust
    /*adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);*/
    
    //back
    
    driveXdist(35,0,bwdspeed);
    CyDelay(200);
    
    //straight
    adjust_dist_US(1,54,fwdspeed);
    CyDelay(200);
    
    
    //right
    turnXdegrees(90,1,trnspeed);
    CyDelay(200);
    
    //adjust
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    adjust_angle_US(adjspeed);
    CyDelay(200);
    

    
    //straight
    adjust_dist_US(1,2,fwdspeed);
    CyDelay(200);
    
    

}


/* Store 5 colours from wall-mounted pucks */
void readWallPucks(uint16 periodLen) {
    
    int col;
    uint16 dist;
    
    flashXtimes(3);
    CyDelay(2000);
    
    /* SCALE HERE */
    S0_Write(0); // 2% scaling?
    S1_Write(1);
    
    for (int i=0; i<5; i++) {
        // get colour
        col = getColour(&periodLen, dist);
        
        if (!SILENT) {
            UART_1_PutString("\nColour: ");
            if (col==0) {
                UART_1_PutString("None");
                LEDR_Write(0);
                LEDG_Write(0);
                LEDB_Write(0);
            } else if (col==1) {
                UART_1_PutString("R ");
                LEDR_Write(1);
                LEDG_Write(0);
                LEDB_Write(0);
            } else if (col==2) {
                UART_1_PutString("G ");
                LEDR_Write(0);
                LEDG_Write(1);
                LEDB_Write(0);
            } else if (col==3) {
                UART_1_PutString("B ");
                LEDR_Write(0);
                LEDG_Write(0);
                LEDB_Write(1);
            }
        }
        
        // store colour
        SEQ[i]=col;
        
        // wait
        flashXtimes(1);
        CyDelay(1000);
    }
    
    flashXtimes(3);
    
    
    UART_1_PutString("\n SEQ: ");
    for (int i=0; i<5; i++) {
        printNumUART(SEQ[i]);
        UART_1_PutString("  ");
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
    SEL_QUAD_Start();
    SER_QUAD_Start();
    ISR_QUAD1_StartEx(QUAD1_ISR);
    ISR_QUAD2_StartEx(QUAD2_ISR);
    ISR_COL_StartEx(COL_ISR);
    PWM_SERVO_Start();
    USTimer_1_Start();
    USTimer_2_Start();    
    isr_1_StartEx(Timer_ISR_Handler1);
    isr_2_StartEx(Timer_ISR_Handler2);
    
    // variables
    int col;
    uint16 periodLen = 0u;   // length of counter period
    periodLen = COL_COUNTER_ReadPeriod(); // read length of period register (in clock counts)
    
    // some hardware initialisations 
    S0_Write(0); // 2% colour scaling
    S1_Write(1);
    moveServo(90); // open claw
    
    ////////
    
    flashXtimes(3);
//    CyDelay(1000);
//    
//    col = getColour(&periodLen, 0);
//    UART_1_PutString("\nCol  ");
//    printNumUART(col);
//    
//    if (col==1) {
//        moveServo(0);
//    }
//    
//    flashXtimes(3);
    
    for(;;)
    {
        
    }
    
}

/* [] END OF FILE */
