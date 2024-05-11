/**********************************************************************************************
 *
 * Module: GPIO
 *
 * File Name: GPIO.c
 *
 * Description: Source file for the TM4C123GH6PM DIO driver for TivaC Built-in Buttons and LEDs
 *
 * Author: Abdelrhman Khaled Sobhi
 *
 ***********************************************************************************************/
#include "gpio.h"
#include "tm4c123gh6pm_registers.h"


void GPIO_BuiltinButtonsLedsInit(void)
{
    /*
     * PF0 --> SW2
     * PF1 --> Red LED
     * PF2 --> Blue LED
     * PF3 --> Green LED
     * PF4 --> SW1
     * PB2   --> Extra Switch
     */

    /* Enable clock for PORTF and wait for clock to start */
    SYSCTL_RCGCGPIO_REG |= 0x20;
    while(!(SYSCTL_PRGPIO_REG & 0x20));

    GPIO_PORTF_LOCK_REG   = 0x4C4F434B;                       /* Unlock the GPIO_PORTF_CR_REG */
    GPIO_PORTF_CR_REG    |= (1<<0);                           /* Enable changes on PF0 */
    GPIO_PORTF_AMSEL_REG &= 0xE0;                             /* Disable Analog on PF0, PF1, PF2, PF3 and PF4 */
    GPIO_PORTF_PCTL_REG  &= 0xFFF00000;                       /* Clear PMCx bits for PF0, PF1, PF2, PF3 and PF4 to use it as GPIO pins */
    GPIO_PORTF_DIR_REG   &= ~(1<<0) & ~(1<<4);                /* Configure PF0 & PF4 as input pins */
    GPIO_PORTF_DIR_REG   |= ((1<<1) | (1<<2) | (1<<3));       /* Configure PF1, PF2 & PF3 as output pins */
    GPIO_PORTF_AFSEL_REG &= 0xE0;                             /* Disable alternative function on PF0, PF1, PF2, PF3 and PF4 */
    GPIO_PORTF_PUR_REG   |= ((1<<0)|(1<<4));                  /* Enable pull-up on PF0 & PF4 */
    GPIO_PORTF_DEN_REG   |= 0x1F;                             /* Enable Digital I/O on PF0, PF1, PF2, PF3 and PF4 */
    GPIO_PORTF_DATA_REG  &= ~(1<<1) & ~(1<<2) & ~(1<<3);      /* Clear bits 1, 2 & 3 in Data register to turn off the LEDs */

    /*Enable Extra Switch on Port B Pin 2*/
    /* Enable clock for PORTB and wait for clock to start */
    SYSCTL_RCGCGPIO_REG |= 0x02;
    while(!(SYSCTL_PRGPIO_REG & 0x02));

    GPIO_PORTB_DEN_REG |= (1<<2);    /* Enable Digital I/O on PB2 */
    GPIO_PORTF_CR_REG |= (1<<2);     /* Enable changes on PB2 */
    GPIO_PORTB_DIR_REG &= ~(1<<2);   /* Configure PB2 as input pin */
    GPIO_PORTF_AFSEL_REG &= ~(1<<2); /* Disable alternative function for PB2 */
    GPIO_PORTB_PUR_REG |= (1<<2);    /* Enable pull-up on PB2 */
    GPIO_PORTF_AMSEL_REG &= ~(1<<2); /* Disable Analog on PB2 */
    GPIO_PORTF_DATA_REG &= ~(1<<2);  /* Clear Data Register for PB2 */
}

void GPIO_RedLedOn(void)
{
    GPIO_PORTF_DATA_REG |= (1<<1);  /* Red LED ON */
}

void GPIO_BlueLedOn(void)
{
    GPIO_PORTF_DATA_REG |= (1<<2);  /* Blue LED ON */
}

void GPIO_GreenLedOn(void)
{
    GPIO_PORTF_DATA_REG |= (1<<3);  /* Green LED ON */
}

void GPIO_RedLedOff(void)
{
    GPIO_PORTF_DATA_REG &= ~(1<<1);  /* Red LED OFF */
}

void GPIO_BlueLedOff(void)
{
    GPIO_PORTF_DATA_REG &= ~(1<<2);  /* Blue LED OFF */
}

void GPIO_GreenLedOff(void)
{
    GPIO_PORTF_DATA_REG &= ~(1<<3);  /* Green LED OFF */
}

void GPIO_RedLedToggle(void)
{
    GPIO_PORTF_DATA_REG ^= (1<<1);  /* Red LED is toggled */
}

void GPIO_BlueLedToggle(void)
{
    GPIO_PORTF_DATA_REG ^= (1<<2);  /* Blue LED is toggled */
}

void GPIO_GreenLedToggle(void)
{
    GPIO_PORTF_DATA_REG ^= (1<<3);  /* Green LED is toggled */
}

uint8 GPIO_SW1GetState(void)
{
    return ((GPIO_PORTF_DATA_REG >> 4) & 0x01);
}

uint8 GPIO_SW2GetState(void)
{
    return ((GPIO_PORTF_DATA_REG >> 0) & 0x01);
}

uint8 GPIO_ExtraSW_GetState(void)
{
    return (GPIO_PORTB_DATA_REG & (1<<2));
}

void GPIO_SW1EdgeTriggeredInterruptInit(void)
{
    GPIO_PORTF_IS_REG    &= ~(1<<4);      /* PF4 detect edges */
    GPIO_PORTF_IBE_REG   &= ~(1<<4);      /* PF4 will detect a certain edge */
    GPIO_PORTF_IEV_REG   &= ~(1<<4);      /* PF4 will detect a falling edge */
    GPIO_PORTF_ICR_REG   |= (1<<4);       /* Clear Trigger flag for PF4 (Interrupt Flag) */
    GPIO_PORTF_IM_REG    |= (1<<4);       /* Enable Interrupt on PF4 pin */
    /* Set GPIO PORTF priority as 5 by set Bit number 21, 22 and 23 with value 2 */
    NVIC_PRI7_REG = (NVIC_PRI7_REG & GPIO_PORTF_PRIORITY_MASK) | (GPIO_PORTF_INTERRUPT_PRIORITY<<GPIO_PORTF_PRIORITY_BITS_POS);
    NVIC_EN0_REG         |= 0x40000000;   /* Enable NVIC Interrupt for GPIO PORTF by set bit number 30 in EN0 Register */
}

void GPIO_SW2EdgeTriggeredInterruptInit(void)
{
    GPIO_PORTF_IS_REG    &= ~(1<<0);      /* PF0 detect edges */
    GPIO_PORTF_IBE_REG   &= ~(1<<0);      /* PF0 will detect a certain edge */
    GPIO_PORTF_IEV_REG   &= ~(1<<0);      /* PF0 will detect a falling edge */
    GPIO_PORTF_ICR_REG   |= (1<<0);       /* Clear Trigger flag for PF0 (Interrupt Flag) */
    GPIO_PORTF_IM_REG    |= (1<<0);       /* Enable Interrupt on PF0 pin */
    /* Set GPIO PORTF priority as 5 by set Bit number 21, 22 and 23 with value 2 */
    NVIC_PRI7_REG = (NVIC_PRI7_REG & GPIO_PORTF_PRIORITY_MASK) | (GPIO_PORTF_INTERRUPT_PRIORITY<<GPIO_PORTF_PRIORITY_BITS_POS);
    NVIC_EN0_REG         |= 0x40000000;   /* Enable NVIC Interrupt for GPIO PORTF by set bit number 30 in EN0 Register */
}

/*
 * initialize this function to work with:
 * 1- ADC0
 * 2- Using Pin 3 on Port E
 * 3- the temperature sensor was connected to AN0 that PE3
 *
 */
void Driver_TempSensor_Init(void)
{
    /*Enable the Clock for Port E for avoiding any fault*/
    SYSCTL_RCGCGPIO_REG |= 0x10;
    while(!(SYSCTL_PRGPIO_REG & 0x10));
    /*Enable the Clock for ADC0 for avoiding any fault*/
    SYSCTL_RCGCADC_REG |= 0x01;
    while(!(SYSCTL_RCGCADC_REG & 0x01));

    /*
     * Initialize the GPIO pin (PE3):
     * 1- Disable Digital Mode
     * 2- Enable Alternative Function
     * 3- Enable Analog Mode
     *
     */
    GPIO_PORTE_DEN_REG   &= ~(1<<3);
    GPIO_PORTE_AMSEL_REG |= (1<<3);
    GPIO_PORTE_AFSEL_REG |= (1<<3);
    //GPIO_PORTE_ADCCTL_REG |= (1<<3); /*Enable Channel 3 on Port E as ADC pin*/

    ADC0_ADCACTSS_REG  &= ~(1<<3);
    ADC0_ADCEMUX_REG   &= ~(0xF000);
    ADC0_ADCSSMUX3_REG  = 0;
    ADC0_ADCSSCTL3_REG |= (1<<1) | (1<<2);
    ADC0_ADCACTSS_REG  |= (1<<3);

}


uint32 Driver_TempSensor_readChannel(void)
{
    uint32  ADC0_read;

    ADC0_ADCPSSI_REG |= (1<<3);

    while((ADC0_ADCRIS_REG & 8) == 0 );

    ADC0_read = ADC0_ADCSSFIFO3_REG;

    ADC0_ADCISC_REG |= (1<<8);

    return ADC0_read;

}

/*
 * initialize this function to work with:
 * 1- ADC1
 * 2- Using Pin 2 on Port E
 * 3- the passenger temperature sensor was connected to AN1 that PE2
 *
 */
void Passenegr_TempSensor_Init(void)
{
    /*Enable the Clock for Port E for avoiding any fault*/
    SYSCTL_RCGCGPIO_REG |= 0x10;
    while(!(SYSCTL_PRGPIO_REG & 0x10));
    /*Enable the Clock for ADC1 for avoiding any fault*/
    SYSCTL_RCGCADC_REG |= (1<<1);
    while(!(SYSCTL_RCGCADC_REG & 0x02));
    /*Enable the Clock for Timer 0 for avoiding any fault*/
    SYSCTL_RCGCTIMER_REG |= 0x01;
    while(!(SYSCTL_RCGCTIMER_REG & 0x01));

    /*
     * Initialize the GPIO pin (PE2):
     * 1- Disable Digital Mode
     * 2- Enable Alternative Function
     * 3- Enable Analog Mode
     *
     */
    GPIO_PORTE_DEN_REG   &= ~(1<<2);
    GPIO_PORTE_AMSEL_REG |= (1<<2);
    GPIO_PORTE_AFSEL_REG |= (1<<2);
    GPIO_PORTE_ADCCTL_REG |= (1<<2); /* Enable Channel 2 on Port E as ADC pin */

    /*
     * Initialize the ADC Module 1 (with interrupt):
     * 1- using sample sequencer 3
     * 2- use a sample sequencer 3 with Always (continuously sample).
     * 3- configure the input sample pin that is PIN 2 on PORT E
     * 4- configure the end and IE bits for sampling
     * 5- configure the mask interrupt corresponding to Sample sequencer 3
     * 6- Enable the sample sample sequencer 3
     * 7- VDDA and GNDA are the voltage references for all ADC modules
     * 8- The temperature sensor is read during the eighth sample of the sample sequence.
     * 9- Enable the sample sequencer 0
     *
     */

    ADC1_ADCACTSS_REG  &= ~(1<<3);
    ADC1_ADCEMUX_REG   &= ~(0xF000);
    ADC1_ADCSSMUX3_REG  = 0;
    ADC1_ADCSSCTL3_REG |= (1<<1) | (1<<2);
    ADC1_ADCACTSS_REG  |= (1<<3);
}

uint32 Passenger_TempSensor_readChannel(void)
{
    uint32  ADC1_read;

    ADC1_ADCPSSI_REG |= (1<<3);//Enable SS3 to Convert

    while((ADC1_ADCRIS_REG & 8) == 0 );

    ADC1_read = ADC1_ADCSSFIFO3_REG;

    ADC1_ADCISC_REG |= (1<<8);

    return ADC1_read;
}
