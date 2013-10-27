#include "nrf24l01+.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
				  
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "delay.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include <stdbool.h>
#include <stdint.h>
#include "delay.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"


#define NRF24L01P_NOP 0xff

static unsigned char spi_swap(unsigned char c) {
    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    unsigned long tmp;
    while(SSIDataGetNonBlocking(SSI0_BASE, &tmp))
    {
    }

    SSIDataPut(SSI0_BASE, c);
    SSIDataGet(SSI0_BASE, &tmp);
    UARTprintf("-%x- ", tmp);
    return (unsigned char)tmp;
}

bool nrf24l01p_init(void) {
    //
    // The SSI0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //
    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
//    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | // GPIO_PIN_3 |
                   GPIO_PIN_2);

    //
    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    //
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);

    //
    // Enable the SSI0 module.
    //
    SSIEnable(SSI0_BASE);

    unsigned char b;

    GPIOPinTypeGPIOOutput(CE_BASE, CE_PIN);
    GPIOPinTypeGPIOOutput(CS_BASE, CS_PIN);
    GPIOPinTypeGPIOInput(IRQ_BASE, IRQ_PIN);
    GPIOPinWrite(CE_BASE, CE_PIN, 0x00);    
    GPIOPinWrite(CS_BASE, CS_PIN, CS_PIN);

	
  nrf24l01p_write(FLUSH_TX, NULL, 0);	  
  nrf24l01p_write(FLUSH_RX, NULL, 0);
    
    b = 0x70;
    nrf24l01p_write(WRITE_REG + STATUS, &b, 1);    // Read STATUS byte and clear IRQ flag's(nRF24L01)
    b = 0x08;
    nrf24l01p_write(WRITE_REG + 0x00, &b, 1);
    nrf24l01p_read(0x00, &b, 1);
    return b == 0x08 ? true : false;
}

unsigned char nrf24l01p_write(unsigned char command,
                              const unsigned char *buffer,
                              char length) {
  unsigned char status;
  GPIOPinWrite(CS_BASE, CS_PIN, 0x00);
  status = spi_swap(command);
  while (length--) {
    spi_swap(*buffer++);
  }
  GPIOPinWrite(CS_BASE, CS_PIN, CS_PIN);
  return status;
}

static unsigned char nrf24l01p_write_register(unsigned char command, unsigned char b) {
  return nrf24l01p_write(command, &b, 1);
}

unsigned char nrf24l01p_read(unsigned char command,
                             unsigned char *buffer,
                             char length) {
  unsigned char status;
  GPIOPinWrite(CS_BASE, CS_PIN, 0x00);
  status = spi_swap(command);
  while (length--) {
    *buffer++ = spi_swap(NRF24L01P_NOP);
  }
  GPIOPinWrite(CS_BASE, CS_PIN, CS_PIN);
  return status;
}

#define TX_ADR_LENGTH   5                         // 5 bytes TX(RX) address width
#define TX_PLOAD_WIDTH  16                        // 16 bytes TX payload

typedef unsigned char BYTE;
// Predefine a static TX address
BYTE const TX_ADDRESS[TX_ADR_LENGTH]  = {0x34,0x43,0x10,0x10,0x01}; 
// Predefine TX payload packet..
BYTE const TX_PAYLOAD[TX_PLOAD_WIDTH] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
                                         0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};  
#define SPI_Write_Buf(x, y, z) nrf24l01p_write(x, y, z)
#define SPI_RW_Reg(x, y) nrf24l01p_write_register(x, y)
//#define SPI_Read(x) nrf24l01p_read(x, (void *)0, 0)

// **********************************************************
//
//  Function: nrf24l01p_tx_mode
//
//  Description:
//  This function initializes one nRF24L01 device to
//  TX mode, set TX address, set RX address for auto.ack,
//  fill TX payload, select RF channel, datarate & TX pwr.
//  PWR_UP is set, CRC(2 bytes) is enabled, & PRIM:TX.
//
//  ToDo: One high pulse(>10탎) on CE will now send this
//  packet and expext an acknowledgment from the RX device.
//
//
//  Author: RSK   Date: 28.11.05
// **********************************************************
void nrf24l01p_tx_mode(void)
{
  SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_LENGTH);    // Writes TX_Address to nRF24L01
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_LENGTH); // RX_Addr0 same as TX_Adr for Auto.Ack
  SPI_Write_Buf(WR_TX_PLOAD, TX_PAYLOAD, TX_PLOAD_WIDTH); // Writes data to TX payload

  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);            // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);        // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a);       // 500탎 + 86탎, 10 retrans...
  SPI_RW_Reg(WRITE_REG + RF_CH, 40);              // Select RF channel 40
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x0f);         // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);           // Set PWR_UP bit, enable CRC(2 bytes) & Prim:TX. MAX_RT & TX_DS enabled..

  //  This device is now ready to transmit one packet of 16 bytes payload to a RX device at address
  //  '3443101001', with auto acknowledgment, retransmit count of 10(retransmit delay of 500탎+86탎)
  //  RF channel 40, datarate = 2Mbps with TX power = 0dBm.
}


// **********************************************************
//
//  Function: nrf24l01p_rx_mode
//
//  Description:
//  This function initializes one nRF24L01 device to
//  RX Mode, set RX address, writes RX payload width,
//  select RF channel, datarate & LNA HCURR.
//  After init, CE is toggled high, which means that
//  this device is now ready to receive a datapacket.
//
//  Author: RSK   Date: 28.11.05
// **********************************************************
void nrf24l01p_rx_mode(void)
{
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_LENGTH); // Use the same address on the RX device as the TX device

  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);            // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);        // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + RF_CH, 40);              // Select RF channel 40
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x0f);         // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);           // Set PWR_UP bit, enable CRC(2 bytes) & Prim:RX. RX_DR enabled..

  GPIOPinWrite(CE_BASE, CE_PIN, CE_PIN); // Set CE pin high to enable RX device

  //  This device is now ready to receive one packet of 16 bytes payload from a TX device sending to address
  //  '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 2Mbps.

}
void nRF24L01_IRQ(unsigned char *IRQ_Source)// interrupt EXT_INT0
{
  BYTE temp;
  
  GPIOPinWrite(CE_BASE, CE_PIN, 0);
  temp = SPI_RW_Reg(WRITE_REG + STATUS, 0x70);    // Read STATUS byte and clear IRQ flag's(nRF24L01)
  UARTprintf("****%x****", temp);

  if(temp & MAX_RT) *IRQ_Source |= MAX_RT;          // Indicates max #of retransmit interrupt
  if(temp & TX_DS)  *IRQ_Source |= TX_DS;           // Indicates TX data succsessfully sent
  
  if(temp & RX_DR)  // In RX mode, check for data received
  {
    // Data received, so find out which datapipe the data was received on:
//    temp = (0x07 & (temp > 1)); // Shift bits in status byte one bit to LSB and mask 'Data Pipe Number'
//    rx_pw = SPI_Read(READ_REG + RX_PW_P0 + temp); // Read current RX_PW_Pn register, where Pn is the pipe the data was received on..
//    SPI_Read_Buf(RD_RX_PLOAD, SPI_Buffer, rx_pw); // Data from RX Payload register is now copied to SPI_Buffer[].
	unsigned char tmp_d[16];
	nrf24l01p_read(RD_RX_PLOAD, tmp_d, 16);
	for (int i = 0; i < 16; ++i) {
		UARTprintf("%x ", tmp_d[i]);
	}
    *IRQ_Source |= RX_DR; // Indicates RX data received
  }
  nrf24l01p_write(FLUSH_TX, NULL, 0);	  
  nrf24l01p_write(FLUSH_RX, NULL, 0);


//  EA = 1; // enable global interrupt again
}


