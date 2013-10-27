/* **********************************************************************
 * Author: Jks Liu
 * Date:   21-Oct-2013
 * **********************************************************************/
#include "lisp-style-repl/lisp-style-repl.h"
#include "nrf24l01+.h"
#include <stdbool.h>
#include <stdint.h>
#include "delay.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"


#define APP_INPUT_BUF_SIZE 128

#ifndef NULL
#define NULL (void *)0
#endif

//*****************************************************************************
//
// Input buffer for the command line interpreter.
//
//*****************************************************************************
static char g_cInput[APP_INPUT_BUF_SIZE];



//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

int copyright(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    UARTprintf ("Lisp style repl: A small command line interpreter.\n");
    UARTprintf ("Last version: https://github.com/jks-liu/lisp-style-repl\n");
    UARTprintf ("Copyright 2013. Jks Liu, http://jks-liu.github.io/\n");
    UARTprintf ("It's open source which is licensed under BSD.\n");
    return 0;
}

static int put_str(const char *str) {
    UARTprintf ("%s", str);
	return 0;
}

void InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);    
}


#ifdef REMOTE_BOARD
//*****************************************************************************
//
// Main function performs init and manages system.
//
// Called automatically after the system and compiler pre-init sequences.
// Performs system init calls, restores state from hibernate if needed and 
// then manages the application context duties of the system.
//
//*****************************************************************************
int main(void) {
    unsigned char IRQ_Source;
    /* Red LED */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlDelay(1);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    /* For UART */
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    InitConsole();

    struct Lsr_command Lsr_commands[] =
            {{"copyright", copyright,
              (const char *[]){"Show copyright",
                               "Show Information like author and license.",
                               NULL}},
             {NULL, NULL, NULL}};
    char *args[5];
    struct Lsr_setting lsr = {Lsr_commands, put_str, args, true};


    if (nrf24l01p_init()) {
        UARTprintf("NRF OK\n");
    } else {
        UARTprintf("NRF fail\n");
    }
    nrf24l01p_tx_mode();

    lsr_init(&lsr);
    //
    // spin forever and wait for carriage returns or state changes.
    //
    
    UARTprintf("\x1b[32;1mjks@LX4F120H5QR:\x1b[34;1m~\n\x1b[0m# ");
    while(1)
    {
        
        if (UARTPeek('\r') != -1) {
          UARTgets(g_cInput,sizeof(g_cInput));
          lsr_execute(g_cInput);
          UARTprintf("\x1b[32;1mjks@LX4F120H5QR:\x1b[34;1m~\n\x1b[0m# ");
        }
        GPIOPinWrite(CE_BASE, CE_PIN, CE_PIN);
        delay_us(10);

        if (GPIOPinRead(IRQ_BASE, IRQ_PIN) == 0) {          
            nRF24L01_IRQ(&IRQ_Source);
            UARTprintf("IRQ: ");
            if (IRQ_Source & MAX_RT) {
                UARTprintf("Max_RT.\n");
            }
            if (IRQ_Source & TX_DS) {
                UARTprintf("TX_DS.\n");
            }
            if (IRQ_Source & RX_DR) {
                UARTprintf("RX_DR\n");
            }
            UARTprintf("^^^^%x^^^^", IRQ_Source);
        }
      
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        //ROM_SysCtlDelay(1000000);
        delay_ms(100);
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
        delay_ms(100);
        //ROM_SysCtlDelay(1000000);
        
        
    }
}
#endif /* REMOTE_BOARD */


#ifdef MOTHER_BOARD
//*****************************************************************************
//
// Main function performs init and manages system.
//
// Called automatically after the system and compiler pre-init sequences.
// Performs system init calls, restores state from hibernate if needed and 
// then manages the application context duties of the system.
//
//*****************************************************************************
int main(void) {
    unsigned char IRQ_Source;
    /* Red LED */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlDelay(1);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    /* For UART */
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    InitConsole();

    struct Lsr_command Lsr_commands[] =
            {{"copyright", copyright,
              (const char *[]){"Show copyright",
                               "Show Information like author and license.",
                               NULL}},
             {NULL, NULL, NULL}};
    char *args[5];
    struct Lsr_setting lsr = {Lsr_commands, put_str, args, true};


    if (nrf24l01p_init()) {
        UARTprintf("NRF OK\n");
    } else {
        UARTprintf("NRF fail\n");
    }
    nrf24l01p_rx_mode();

    lsr_init(&lsr);
    //
    // spin forever and wait for carriage returns or state changes.
    //
    
    UARTprintf("\x1b[32;1mjks@LX4F120H5QR:\x1b[34;1m~\n\x1b[0m# ");
    while(1)
    {
        
        if (UARTPeek('\r') != -1) {
          UARTgets(g_cInput,sizeof(g_cInput));
          lsr_execute(g_cInput);
          UARTprintf("\x1b[32;1mjks@LX4F120H5QR:\x1b[34;1m~\n\x1b[0m# ");
        }
        GPIOPinWrite(CE_BASE, CE_PIN, CE_PIN);
        delay_us(10);

        if (GPIOPinRead(IRQ_BASE, IRQ_PIN) == 0) {          
            nRF24L01_IRQ(&IRQ_Source);
            UARTprintf("IRQ: ");
            if (IRQ_Source & MAX_RT) {
                UARTprintf("Max_RT.\n");
            }
            if (IRQ_Source & TX_DS) {
                UARTprintf("TX_DS.\n");
            }
            if (IRQ_Source & RX_DR) {
                UARTprintf("RX_DR\n");
            }
            UARTprintf("^^^^%x^^^^", IRQ_Source);
        }
      
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        //ROM_SysCtlDelay(1000000);
        delay_ms(100);
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
        delay_ms(100);
        //ROM_SysCtlDelay(1000000);
        
        
    }
}
#endif /* MOTHER_BOARD */
