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

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_1_Start();
    
    for(;;)
    {
        /* Place your application code here. */
        UART_1_PutString("Hello world!\n");
        TEST_PIN_Write(1);
        CyDelay(1000);
        
        TEST_PIN_Write(0);
        CyDelay(1000);
        
    }
}

/* [] END OF FILE */
