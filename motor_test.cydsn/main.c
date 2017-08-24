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

    /* DRIVE IN TWO DIRECTIONS */
    /* ======================= */
    
    PWM_1_Start();
    
    // enable
    PWM_1_Enable();
    
    // direction A
    A1_Write(1);
    A2_Write(0);
    CyDelay(5000);
    
    // stop
    A1_Write(0);
    A2_Write(0);
    CyDelay(1000);
    
    // direction B
    A1_Write(0);
    A2_Write(1);
    CyDelay(5000);
    
    // stop
    A1_Write(0);
    A2_Write(0);
    for(;;)
    {

    }
}

/* [] END OF FILE */
