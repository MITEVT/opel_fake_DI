#include "board.h"


// -------------------------------------------------------------
// Board ISRs

/**
 * SysTick Timer Interrupt Handler. Counts milliseconds since start
 */
void SysTick_Handler(void) {
	msTicks++;
}

// -------------------------------------------------------------
// Public Functions and Members

const uint32_t OscRateIn = 0;

int8_t Board_SysTick_Init(void) {
	msTicks = 0;

	// Update the value of SystemCoreClock to the clock speed in hz
	SystemCoreClockUpdate();

	// Initialize SysTick Timer to fire interrupt at 1kHz
	return (SysTick_Config (SystemCoreClock / 1000));
}

void Board_LEDs_Init(void) {
	Chip_GPIO_Init(LPC_GPIO);
	Chip_GPIO_WriteDirBit(LPC_GPIO,2,10, true);
}

void LED_On(int port,int pin) {
	Chip_GPIO_SetPinState(LPC_GPIO,port,pin,true);
}

void Board_UART_Init(uint32_t baudrate) {
	Chip_IOCON_PinMuxSet(LPC_IOCON, UART_RX_IOCON, (IOCON_FUNC1 | IOCON_MODE_INACT));	// Rx pin
	Chip_IOCON_PinMuxSet(LPC_IOCON, UART_TX_IOCON, (IOCON_FUNC1 | IOCON_MODE_INACT));	// Tx Pin

	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, baudrate);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);
}

void Board_UART_Print(const char *str) {
	Chip_UART_SendBlocking(LPC_USART, str, strlen(str));
}

void Board_UART_Println(const char *str) {
	Board_UART_Print(str);
	Board_UART_Print("\r\n");
}

void Board_UART_PrintNum(const int num, uint8_t base, bool crlf) {
	static char str[32];
	itoa(num, str, base);
	Board_UART_Print(str);
	if (crlf) Board_UART_Print("\r\n");
}

void Board_UART_SendBlocking(const void *data, uint8_t num_bytes) {
	Chip_UART_SendBlocking(LPC_USART, data, num_bytes);
}

int8_t Board_UART_Read(void *data, uint8_t num_bytes) {
	return Chip_UART_Read(LPC_USART, data, num_bytes);
}

void Board_Contactors_Init(void){
	Chip_GPIO_WriteDirBit(LPC_GPIO, CONTACTOR_PRECHARGE_SWITCH, false);
	Chip_GPIO_WriteDirBit(LPC_GPIO, CONTACTOR_LOW_SWITCH, false);
	Chip_GPIO_WriteDirBit(LPC_GPIO, CONTACTOR_PRECHARGE_CTRL, true);
	Chip_GPIO_WriteDirBit(LPC_GPIO, CONTACTOR_LOW_CTRL, true);
}

void Board_Contactor_Controls_Precharge_Open(void){
	Chip_GPIO_SetPinState(LPC_GPIO,CONTACTOR_PRECHARGE_CTRL,false);
}

void Board_Contactor_Controls_Low_Open(void){
	Chip_GPIO_SetPinState(LPC_GPIO,CONTACTOR_LOW_CTRL,false);
}

void Board_Contactor_Controls_Precharge_Closed(void){
	Chip_GPIO_SetPinState(LPC_GPIO,CONTACTOR_PRECHARGE_CTRL,true);
}

void Board_Contactor_Controls_Low_Closed(void){
	Chip_GPIO_SetPinState(LPC_GPIO,CONTACTOR_LOW_CTRL,true);
}

void Board_State_Contactor_Update(uint8_t *state){
	uint8_t contactors = 0;
	if(Chip_GPIO_GetPinState(LPC_GPIO,CONTACTOR_PRECHARGE_SWITCH)){
		contactors |= CONTACTOR_PRECHARGE_CTRL_BIT;
	}
	if(Chip_GPIO_GetPinState(LPC_GPIO,CONTACTOR_LOW_SWITCH)){
		contactors |= CONTACTOR_LOW_CTRL_BIT;
	}
	*state = ((*state & (~CONTACTOR_CTRL_BITS)) | contactors);
}

