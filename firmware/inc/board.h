#ifndef __BOARD_H_
#define __BOARD_H_

#include "chip.h"
#include "util.h"
#include <string.h>

// -------------------------------------------------------------
// Global Variables

extern const uint32_t OscRateIn; 				/** @brief Board Oscillator Frequency (Hz) **/
volatile uint32_t msTicks; 						/** @brief System Time (ms) **/

// -------------------------------------------------------------
// Configuration Macros


// -------------------------------------------------------------
// Pin Descriptions

#define LED0_PORT 2
#define LED0_PIN 5

#define LED1_PORT 0
#define LED1_PIN 6

#define UART_RX_PORT 1
#define UART_RX_PIN 6
#define UART_RX_IOCON IOCON_PIO1_6

#define UART_TX_PORT 1
#define UART_TX_PIN 7
#define UART_TX_IOCON IOCON_PIO1_7

#define CONTACTOR_PRECHARGE_CTRL_PORT 1
#define CONTACTOR_PRECHARGE_CTRL_PIN 9

#define CONTACTOR_LOW_CTRL_PORT 2
#define CONTACTOR_LOW_CTRL_PIN 4

#define CONTACTOR_PRECHARGE_SWITCH_PORT 0
#define CONTACTOR_PRECHARGE_SWITCH_PIN 4

#define CONTACTOR_LOW_SWITCH_PORT 0
#define CONTACTOR_LOW_SWITCH_PIN 5


// -------------------------------------------------------------
// DI States
#define KEY_IGNITION_BITS 				3
#define KEY_IGNITION_RUN 				1
#define KEY_IGNITION_START 				2
#define KEY_IGNITION_OFF 				0

#define DRIVE_STATUS_BITS 				((0xF)<<2)
#define DRIVE_STATUS_PARKED 			0
#define DRIVE_STATUS_FORWARD 			(1<<2)
#define DRIVE_STATUS_REVERSE 			(2<<2)
#define DRIVE_STATUS_SHUTDOWN_IMPENDING (3<<2)
#define DRIVE_STATUS_INIT 				(4<<2)
#define DRIVE_STATUS_CHARGE 			(5<<2)
#define DRIVE_STATUS_OFF 				(6<<2)

#define CONTACTOR_CTRL_BITS					(3<<6)
#define CONTACTOR_PRECHARGE_CTRL_BIT 		(1<<6)
#define CONTACTOR_LOW_CTRL_BIT	 			(1<<7)


// -------------------------------------------------------------
// Computed Macros

#define LED0 LED0_PORT, LED0_PIN
#define LED1 LED1_PORT, LED1_PIN

#define UART_RX UART_RX_PORT, UART_RX_PIN
#define UART_TX UART_TX_PORT, UART_TX_PIN

#define CONTACTOR_PRECHARGE_CTRL 	CONTACTOR_PRECHARGE_CTRL_PORT, CONTACTOR_PRECHARGE_CTRL_PIN
#define CONTACTOR_LOW_CTRL 			CONTACTOR_LOW_CTRL_PORT, CONTACTOR_LOW_CTRL_PIN
#define CONTACTOR_PRECHARGE_SWITCH 	CONTACTOR_PRECHARGE_SWITCH_PORT, CONTACTOR_PRECHARGE_SWITCH_PIN
#define CONTACTOR_LOW_SWITCH 		CONTACTOR_LOW_SWITCH_PORT, CONTACTOR_LOW_SWITCH_PIN

#define Board_LED_On(led) {Chip_GPIO_SetPinState(LPC_GPIO, led, true);}
#define Board_LED_Off(led) {Chip_GPIO_SetPinState(LPC_GPIO, led, false);}
 
// -------------------------------------------------------------
// Board Level Function Prototypes
/**
 * Initialize the Core Systick Timer
 * 
 * @return true if error
 */
int8_t Board_SysTick_Init(void);

void Board_LEDs_Init(void);

void Board_UART_Init(uint32_t baudrate);

/**
 * Transmit the given string through the UART peripheral (blocking)
 * 
 * @param str pointer to string to transmit
 * @note	This function will send or place all bytes into the transmit
 *			FIFO. This function will block until the last bytes are in the FIFO.
 */
void Board_UART_Print(const char *str);

/**
 * Transmit a string through the UART peripheral and append a newline and a linefeed character (blocking)
 * 
 * @param str pointer to string to transmit
 * @note	This function will send or place all bytes into the transmit
 *			FIFO. This function will block until the last bytes are in the FIFO.
 */
void Board_UART_Println(const char *str);

/**
 * Transmit a string containing a number through the UART peripheral (blocking)
 * 
 * @param num number to print
 * @param base number base
 * @param crlf append carraige return and line feed
 */
void Board_UART_PrintNum(const int num, uint8_t base, bool crlf);

/**
 * Transmit a byte array through the UART peripheral (blocking)
 * 
 * @param	data		: Pointer to data to transmit
 * @param	num_bytes	: Number of bytes to transmit
 * @note	This function will send or place all bytes into the transmit
 *			FIFO. This function will block until the last bytes are in the FIFO.
 */
void Board_UART_SendBlocking(const void *data, uint8_t num_bytes);

/**
 * Read data through the UART peripheral (non-blocking)
 * 
 * @param	data		: Pointer to bytes array to fill
 * @param	num_bytes	: Size of the passed data array
 * @return	The actual number of bytes read
 * @note	This function reads data from the receive FIFO until either
 *			all the data has been read or the passed buffer is completely full.
 *			This function will not block. This function ignores errors.
 */
int8_t Board_UART_Read(void *data, uint8_t num_bytes);

void Board_CAN_Init(uint32_t baudrate, void (*rx_callback)(uint8_t), void (*tx_callback)(uint8_t), void (*error_callback)(uint32_t));

void Board_Contactors_Init(void);

void Board_Contactor_Controls_Precharge_Closed(void);

void Board_Contactor_Controls_Low_Closed(void);

void Board_Contactor_Controls_Low_Open(void);

void Board_Contactor_Controls_Precharge_Open(void);

void Board_State_Contactor_Update(uint8_t *state);

#endif
