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
volatile uint8 cap1, cap2 = 0;
volatile uint8 freq = 1; // volatile bc it can be 'unknowingly' modified by isr
volatile int flag = 0;

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_1_Start();
    Timer_1_Start();
    isr_1_Start();
           
    
    S0_Write(1); // 20% scaling?
    S1_Write(0);
    
    S2_Write(0);
    S2_Write(0);
    
    for(;;)
    {
        /* Place your application code here. */
        UART_1_Start();
        UART_1_WriteTxData('s');
        
        //myVar = COLOUT_Read();
        
        // start timer, capture t
                
//        UART_1_WriteTxData(freq);
//        UART_1_WriteTxData(flag);
//        UART_1_WriteTxData(0x0A);
        
        UART_1_WriteTxData(cap1);
        UART_1_WriteTxData(cap2);
        UART_1_WriteTxData(0x0A);
        CyDelay(1000);    
        
    }
}


/* [] END OF FILE */
