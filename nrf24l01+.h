/* **********************************************************************
 * Author: Jks Liu(chinatianma@gmail.com)
 * Date:
 * ***********************************************************************/

#ifndef NRF24L01P_NRF24L01P_H_
#define NRF24L01P_NRF24L01P_H_



#define CE_BASE GPIO_PORTA_BASE
#define CE_PIN  GPIO_PIN_6
#define CS_BASE GPIO_PORTA_BASE
#define CS_PIN  GPIO_PIN_7

#define IRQ_BASE GPIO_PORTA_BASE
#define IRQ_PIN GPIO_PIN_3

/* Custom include and MACRO here */
// Define nRF24L01 interrupt flag's
#define IDLE            0x00  // Idle, no interrupt pending
#define MAX_RT          0x10  // Max #of TX retrans interrupt
#define TX_DS           0x20  // TX data sent interrupt
#define RX_DR           0x40  // RX data received

//********************************************************************************************************************//
// SPI(nRF24L01) commands
#define READ_REG        0x00  // Define read command to register
#define WRITE_REG       0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
//#define NOP             0xFF  // Define No Operation, might be used to read status register

//********************************************************************************************************************//
// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address

#ifdef __SDCC
  /* http://sdcc.sourceforge.net/ */
  #ifndef code
    #define code __code
  #endif
  #include <mcs51/at89x52.h>
  #include <stdint.h>
  #include <stdbool.h>
#elif defined(__C51__) 
  /* Keil C51 */
  #include <atmel/at89x52.h>
  /* Keil C51 does not support c99 */
  #ifndef STDINT_AND_STDBOOL_DEFINED_
  #define STDINT_AND_STDBOOL_DEFINED_
    typedef unsigned char uint8_t;
    typedef unsigned short uint16_t;
    typedef unsigned long uint32_t;
    typedef bit bool;
    #define true 1
    #define false 0
  #endif
#else
  #include <stdint.h>
  #include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif
#if 0
}
#endif 

/* Custom function synopsis here */
bool nrf24l01p_init(void);
unsigned char nrf24l01p_read(unsigned char command,
                             unsigned char *buffer,
                             char length);
void nrf24l01p_tx_mode(void);
void nRF24L01_IRQ(unsigned char *IRQ_Source);
void nrf24l01p_rx_mode(void);
unsigned char nrf24l01p_write(unsigned char command,
                              const unsigned char *buffer,
                              char length);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif  /* NRF24L01P_NRF24L01P_H_ */

