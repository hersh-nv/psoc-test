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

CY_ISR_PROTO(SE_ISR) {
    // isr every time rising edge of (A XOR B) occurs
    

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* DRIVE FOR 10s WHILE MEASURING DISTANCE */
    /* ======================= */
    
    PWM_1_Start();
    
    // enable
    PWM_1_Enable();
    
    // 50% speed
    PWM_1_WriteCompare(127);
    
    // direction A
    A1_Write(1);
    A2_Write(0);
    CyDelay(10000); // drive for 10 seconds
    
    
    
    for(;;)
    {

    }
}

/* [] END OF FILE */
