#include "ti/devices/msp432p4xx/inc/msp.h"  //Importing the MSP432 libraries
volatile unsigned int i;                   // Iterate through the characters to be transmitted
int fl = 0;                               //Flag variable to check if the direction has reversed
char TXData [16]= {""};                  //Transmit data



int main(void) {

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // Stop WDT


    // GPIO Setup

    P4->OUT &= ~BIT6;                            //Direction of Motor(Port 4.6)
    P4->DIR |= BIT6;


    P1->SEL0 |= BIT3;                           // Configure UART pins

    P1->REN = BIT4;                            // Enable pull resistor
    P1->OUT = BIT4;                           // Enable pull-up (P1.4 output high)
    P1->IFG = 0;                             // Clear all P1 interrupt flags
    P1->IE = BIT4;                          // Enable interrupt for P1.4
    P1->IES = BIT4;                        // Interrupt on high-to-low transition


    P5->SEL1 |= BIT4;                       // Configure P5.4 for ADC
    P5->SEL0 |= BIT4;


    P2->SEL0 |= 0x80;
    P2->SEL1 &= ~0x80;
    P2->DIR |= 0x80;                    // Configuring Pin 2.7 as PWM output


    TIMER_A0->CCTL[4] = 0xE0;        // CCR4 reset/set mode
    TIMER_A0->CTL = 0x0214;         // use SMCLK, count up, clear TA0R register */



    //UART config
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SSEL__SMCLK;  // Remain eUSCI in reset and Configure eUSCI clock source for SMCLK
    EUSCI_A0->BRW = 313; // 3Mhz/(9600) 
    EUSCI_A0->MCTLW = 0*EUSCI_A_MCTLW_OS16;// Turn off modulation
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI



    // Sampling time, S&H=16, ADC14 on
    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_ON;//,Pulse sample mode is selected,core is ready to power up  when a valid conversion is triggered.
    ADC14->CTL1 = ADC14_CTL1_RES_2;         // Use sampling timer, 12-bit conversion results

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1;   // A1 ADC input select; Vref= 3.3V

    // Enable global interrupt
    __enable_irq();    // Enable ADC interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);

    ADC14->IER0 |= ADC14_IER0_IE0;          // Enable ADC conv complete interrupt

    ADC14->IER0 = ADC14_IER0_IE0;           // Enable ADC14IFG.0
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Wake up on exit from ISR

    // Ensures SLEEPONEXIT takes effect immediately
    __DSB();

    while (1)
    {


         //Start sampling/conversion
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
        __sleep();//Enter Low power mode 0

        __no_operation();                   // For debugger
    }
}

// ADC14 interrupt service routine
void ADC14_IRQHandler(void) {

    if (ADC14->MEM[0] > 0x00 & ADC14->MEM[0] <= 0xFF) { //check if the ADc count is between 0x00 and 0XFF
        TIMER_A0->CCR[0] = 50000 - 1; //Decreasing the speed of the motor by increasing the number of ticks
        TIMER_A0->CCR[4] = 50000 / 4; // 25% duty cycle for PWM wave
        if (P1->IFG & BIT4) { //Button Interrupt
            P4->OUT ^= BIT6;//Changing the direction of rotation of motor
            fl ^= 1; //Changing the flag to indicate the direction of rotation of motor
            if(fl == 0){//If P4.6 signal is low then fl==0
                strcpy(TXData, "Anticlockwise ");//Copying the message onto TXData
            }
            else if(fl == 1){//If P4.6 signal is low then fl==1
                strcpy(TXData, "Clockwise ");//Copying the message onto TXData
            }
            while(TXData[i]){ //Once the data to be transmitted data is obtained

                while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));//Check if there is no interrupt
                EUSCI_A0->TXBUF = TXData[i]; //Transmits each character over the serial port through TXBuffer
                i++;//increments the character count
            }
            i=0;//Clear character count
            __delay_cycles(200000);//Delay for avoiding de-bouncing
            P1->IFG &= ~BIT4;//Clear button interrupt flag
        }

    }
    if (ADC14->MEM[0] > 0xFF & ADC14->MEM[0] <= 0xFFF) {//check if the ADc count is between 0x00 and 0XFF
        TIMER_A0->CCR[0] = 10000 - 1;//Increasing the speed of the motor by reducing the number of ticks
        TIMER_A0->CCR[4] = 10000 / 2;//50% duty cycle for PWM wave
        if (P1->IFG & BIT4) { //Button Interrupt

            P4->OUT ^= BIT6;//Changing the direction of rotation of motor
            fl ^= 1;    //Changing the flag to indicate the direction of rotation of motor
            if(fl == 0){//If P4.6 signal is low then fl==0
                strcpy(TXData, "Anticlockwise ");//Copying the message onto TXData
            }
            else if(fl == 1){//If P4.6 signal is low then fl==1
                strcpy(TXData, "Clockwise ");//Copying the message onto TXData
            }
            while(TXData[i]){//Once the data to be transmitted data is obtained

                while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));//Check if there is no interrupt
                EUSCI_A0->TXBUF = TXData[i];//Transmits each character over the serial port through TXBuffer
                i++;//increments the character count
            }
            i=0;//Clear character count
            __delay_cycles(200000);//Delay for avoiding de-bouncing
            P1->IFG &= ~BIT4;//Clear button interrupt flag
        }




    }

}
