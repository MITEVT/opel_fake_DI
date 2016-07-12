#include "board.h"

// -------------------------------------------------------------
// Macro Definitions

#define CCAN_BAUD_RATE 500000 					// Desired CAN Baud Rate
#define UART_BAUD_RATE 57600 					// Desired UART Baud Rate

#define BUFFER_SIZE 8

// -------------------------------------------------------------
// Static Variable Declaration

extern volatile uint32_t msTicks;

static CCAN_MSG_OBJ_T msg_obj; 					// Message Object data structure for manipulating CAN messages
static RINGBUFF_T can_rx_buffer;				// Ring Buffer for storing received CAN messages
static CCAN_MSG_OBJ_T _rx_buffer[BUFFER_SIZE]; 	// Underlying array used in ring buffer

static char str[100];							// Used for composing UART messages
static uint8_t uart_rx_buffer[BUFFER_SIZE]; 	// UART received message buffer

static bool can_error_flag;
static uint32_t can_error_info;
static uint32_t last_message;

static uint8_t DI_CTRL;
static uint32_t last_update;

static uint8_t motor_state;
static int16_t motor_current;
static int16_t motor_speed;
static uint16_t HV_Voltage;
static int16_t motor_torque;
static bool motor_shutdown_ok;

// -----------------------------------------
// Helper Functions

/**
 * Delay the processor for a given number of milliseconds
 * @param ms Number of milliseconds to delay
 */
void _delay(uint32_t ms) {
	uint32_t curTicks = msTicks;
	while ((msTicks - curTicks) < ms);
}

inline static void displayData(void){
	Board_UART_Print("[Motor current:");
	Board_UART_PrintNum(motor_current,10,false);
	Board_UART_Println("]");
	Board_UART_Print("[Motor Speed:");
	Board_UART_PrintNum(motor_speed,10,false);
	Board_UART_Println("]");
	Board_UART_Print("[High Voltage:");
	Board_UART_PrintNum(HV_Voltage,10,false);
	Board_UART_Println("]");
	Board_UART_Print("[Motor Torque:");
	Board_UART_PrintNum(motor_torque,10,false);
	Board_UART_Println("]");
	Board_UART_Print("[Motor Shutdown OK:");
	if(motor_shutdown_ok){
		Board_UART_Println("True]");
	}
	else{
		Board_UART_Println("False]");
	}
	Board_UART_Print("[Motor State:");
	switch(motor_state){
		case (0):
			Board_UART_Println("Powerup]");
			break;
		case (1):
			Board_UART_Println("Disabled]");
			break;
		case (3):
			Board_UART_Println("Enabled]");
			break;
		case (2):
			Board_UART_Println("Standby]");
			break;
		case (4):
			Board_UART_Println("Powerdown]");
			break;
		case (5):
			Board_UART_Println("Fault]");
			break;
		case (6):
			Board_UART_Println("Critical Fault]");
			break;
		case (7):
			Board_UART_Println("Off]");
			break;
		default:
			Board_UART_Println("Unknown]");
	}
}

inline static void sendDIMessage(void){
	msg_obj.msgobj = 2;
	msg_obj.mode_id = 0x505;
	msg_obj.dlc = 4;
	if((DI_CTRL&KEY_IGNITION_BITS)==KEY_IGNITION_OFF){
		msg_obj.data_16[0] = 0;
	}
	else if ((DI_CTRL&KEY_IGNITION_BITS)==KEY_IGNITION_RUN){
		msg_obj.data_16[0] = 0x020;
	}
	else if ((DI_CTRL&KEY_IGNITION_BITS)==KEY_IGNITION_START){
		msg_obj.data_16[0] = 0x040;
	}
	else{
		DI_CTRL = 0xFF;
	}
	if((DI_CTRL&DRIVE_STATUS_BITS)==DRIVE_STATUS_PARKED){
		msg_obj.data_16[1] = 0;
	}	
	else if((DI_CTRL&DRIVE_STATUS_BITS)==DRIVE_STATUS_FORWARD){
		msg_obj.data_16[1] = 0x00F0;
	}
	else if((DI_CTRL&DRIVE_STATUS_BITS)==DRIVE_STATUS_REVERSE){
		msg_obj.data_16[1] = 0x0030;
	}
	else if((DI_CTRL&DRIVE_STATUS_BITS)==DRIVE_STATUS_SHUTDOWN_IMPENDING){
		msg_obj.data_16[1] = 0x0F00;
	}
	else if((DI_CTRL&DRIVE_STATUS_BITS)==DRIVE_STATUS_INIT){
		msg_obj.data_16[1] = 0x0300;
	}
	else if((DI_CTRL&DRIVE_STATUS_BITS)==DRIVE_STATUS_INIT){
		msg_obj.data_16[1] = 0xF000;
	}
	else if((DI_CTRL&DRIVE_STATUS_BITS)==DRIVE_STATUS_INIT){
		msg_obj.data_16[1] = 0x3000;
	}
	else{
		DI_CTRL = 0xFF;
	}
	if(DI_CTRL == 0xFF){
		Board_UART_Println("SEVERE ERROR : Unknown State!");
		msg_obj.data_16[0]=1;
		msg_obj.data_16[1] = 0x0F00;
	}
	LPC_CCAN_API->can_transmit(&msg_obj);			
}	

// -------------------------------------------------------------
// CAN Driver Callback Functions

/*	CAN receive callback */
/*	Function is executed by the Callback handler after
    a CAN message has been received */
void CAN_rx(uint8_t msg_obj_num) {
	// LED_On();
	/* Determine which CAN message has been received */
	msg_obj.msgobj = msg_obj_num;
	/* Now load up the msg_obj structure with the CAN message */
	LPC_CCAN_API->can_receive(&msg_obj);
	if (msg_obj_num == 1) {
		RingBuffer_Insert(&can_rx_buffer, &msg_obj);
	}
}

/*	CAN transmit callback */
/*	Function is executed by the Callback handler after
    a CAN message has been transmitted */
void CAN_tx(uint8_t msg_obj_num) {
	msg_obj_num = msg_obj_num;
}

/*	CAN error callback */
/*	Function is executed by the Callback handler after
    an error has occurred on the CAN bus */
void CAN_error(uint32_t error_info) {
	can_error_info = error_info;
	can_error_flag = true;
}

// -------------------------------------------------------------
// Interrupt Service Routines


// -------------------------------------------------------------
// Main Program Loop

int main(void)
{
	DI_CTRL=0;

	//---------------
	// Initialize UART Communication
	Board_UART_Init(UART_BAUD_RATE);
	Board_UART_Println("Started up");

	//---------------
	// Initialize SysTick Timer to generate millisecond count
	if (Board_SysTick_Init()) {
		Board_UART_Println("Failed to Initialize SysTick. ");
		// Unrecoverable Error. Hang.
		while(1);
	}

	//---------------
	// Initialize GPIO and LED as output
	Board_LEDs_Init();
	Board_LED_On(LED0);
	Board_Contactors_Init();

	//---------------
	// Initialize CAN  and CAN Ring Buffer

	RingBuffer_Init(&can_rx_buffer, _rx_buffer, sizeof(CCAN_MSG_OBJ_T), BUFFER_SIZE);
	RingBuffer_Flush(&can_rx_buffer);

	Board_CAN_Init(CCAN_BAUD_RATE, CAN_rx, CAN_tx, CAN_error);
	msg_obj.msgobj = 1;
	msg_obj.mode_id = 0x000;
	msg_obj.mask = 0x000;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	can_error_flag = false;
	can_error_info = 0;
	last_message = msTicks;
	last_update = msTicks;
	
	while (1) {
		if(last_message<msTicks-100){
			last_message = msTicks;
			sendDIMessage();
			displayData();
		}
		if(last_update<msTicks-10){
			last_update = msTicks;
			Board_State_Contactor_Update(&DI_CTRL);	
			if((DI_CTRL & CONTACTOR_PRECHARGE_CTRL_BIT)==0){
				Board_Contactor_Controls_Precharge_Closed();
			}
			else{
				Board_Contactor_Controls_Precharge_Open();
			}
			if((DI_CTRL & CONTACTOR_LOW_CTRL_BIT)==0){
				Board_Contactor_Controls_Low_Closed();
			}
			else{
				Board_Contactor_Controls_Low_Open();
			}
		}
		if (!RingBuffer_IsEmpty(&can_rx_buffer)) {
			CCAN_MSG_OBJ_T temp_msg;
			RingBuffer_Pop(&can_rx_buffer, &temp_msg);
			if(temp_msg.mode_id == 0x705){
				motor_shutdown_ok = (temp_msg.data[0] & 0x80 > 0);
				motor_state = ((temp_msg.data[0] >>4) & 7);
				motor_current = ((temp_msg.data[1] << 8) & 0xFF00) | (temp_msg.data[2]);
				motor_speed = ((temp_msg.data[3] << 4) & 0xFF0) | ((temp_msg.data[4] >> 4) & 0xF);
				HV_Voltage = ((temp_msg.data[4] << 8) & 0xF00) | (temp_msg.data[5]);
				motor_torque = ((temp_msg.data[6]<<8) & 0xFF00) | temp_msg.data[7];
			}
		}	

		if (can_error_flag) {
			can_error_flag = false;
			Board_UART_Print("[CAN Error: 0b");
			itoa(can_error_info, str, 2);
			Board_UART_Print(str);
			Board_UART_Println("]");
		}

		uint8_t count;
		if ((count = Board_UART_Read(uart_rx_buffer, BUFFER_SIZE)) != 0) {
			Board_UART_SendBlocking(uart_rx_buffer, count); // Echo user input
			switch (uart_rx_buffer[0]) {
				case '1':
					DI_CTRL = (DI_CTRL & ~KEY_IGNITION_BITS)|KEY_IGNITION_RUN;
					break;
				case '2':
					DI_CTRL = (DI_CTRL & ~KEY_IGNITION_BITS)|KEY_IGNITION_START;
					break;
				case '0':
					DI_CTRL = (DI_CTRL & ~KEY_IGNITION_BITS)|KEY_IGNITION_OFF;
					break;
				case 'p':
					DI_CTRL = (DI_CTRL & ~DRIVE_STATUS_BITS)|DRIVE_STATUS_PARKED;
					break;
				case 'f':
					DI_CTRL = (DI_CTRL & ~DRIVE_STATUS_BITS)|DRIVE_STATUS_FORWARD;
					break;
				case 'r':
					DI_CTRL = (DI_CTRL & ~DRIVE_STATUS_BITS)|DRIVE_STATUS_REVERSE;
					break;
				case 's':
					DI_CTRL = (DI_CTRL & ~DRIVE_STATUS_BITS)|DRIVE_STATUS_SHUTDOWN_IMPENDING;
					break;
				case 'i':
					DI_CTRL = (DI_CTRL & ~DRIVE_STATUS_BITS)|DRIVE_STATUS_INIT;
					break;
				case 'c':
					DI_CTRL = (DI_CTRL & ~DRIVE_STATUS_BITS)|DRIVE_STATUS_CHARGE;
					break;
				case 'o':
					DI_CTRL = (DI_CTRL & ~DRIVE_STATUS_BITS)|DRIVE_STATUS_OFF;
					break;
				default:
					Board_UART_Println("Invalid Command");
					break;
			}
		}
	}
}
