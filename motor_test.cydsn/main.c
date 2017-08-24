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

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* DRIVE AT VARIOUS SPEEDS */
    /* ======================= */
    
    PWM_1_Start();
    
    // enable
    PWM_1_Enable();
    
    // 25% speed
    PWM_1_WriteCompare(63);
    
    // direction A
    A1_Write(1);
    A2_Write(0);
    CyDelay(2000);
        
    // 50% speed
    PWM_1_WriteCompare(127);
    CyDelay(2000);
    
    // 75% speed
    PWM_1_WriteCompare(191);
    CyDelay(2000);
    
    // 100% speed
    PWM_1_WriteCompare(255);
    CyDelay(3000);
    
    // stop
    A1_Write(0);
    A2_Write(0);
    
    for(;;)
    {

    }
}

/* [] END OF FILE */
