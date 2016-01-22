#include "asf.h"
#include "bastian_sercom.h"

	// Motor Parameters To Program
#define MOTOR_OPERATION_MODE_1	( (uint8_t) 0x09 )
#define MOTOR_OPERATION_MODE_2	( (uint8_t) 0x01 )

#define MOTOR_DEFAULT_RAMP_UP_H	( (uint8_t) 0x00 )
#define MOTOR_DEFAULT_RAMP_UP_L	( (uint8_t) 0xFF )
#define MOTOR_DEFAULT_RAMP_DN_H	( (uint8_t) 0x00 )
#define MOTOR_DEFAULT_RAMP_DN_L	( (uint8_t) 0xFF )

	// State Definition
#define MOTOR_STATE_START_UP_CHECK_HEALTH		((uint8_t) 0x01 )
#define MOTOR_STATE_START_UP_HEALTH_RESPONSE	((uint8_t) 0x02 )
#define MOTOR_STATE_GET_PARAMETER_ACCESS		((uint8_t) 0x03 )
#define MOTOR_STATE_START_UP_CHECK_MODE_1		((uint8_t) 0x04 )
#define MOTOR_STATE_START_UP_CHECK_MODE_2		((uint8_t) 0x05 )
#define MOTOR_STATE_START_UP_CHANGE_MODE_1		((uint8_t) 0x06 )
#define MOTOR_STATE_START_UP_CHANGE_MODE_2		((uint8_t) 0x07 )
#define MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP		((uint8_t) 0x08 )
#define MOTOR_STATE_START_UP_CHANGE_CW_RAMP_DOWN	((uint8_t) 0x09 )
#define MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_UP		((uint8_t) 0x0A )
#define MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_DOWN	((uint8_t) 0x0B )

#define MOTOR_STATE_SAVE_EEPROM			((uint8_t) 0x0E )

#define MOTOR_STATE_CHECK_HEALTH		((uint8_t) 0x10 )
#define MOTOR_STATE_HEALTH_RESPONSE		((uint8_t) 0x11 )
#define MOTOR_STATE_CHECK_HEALTH_BUSY	((uint8_t) 0x1E )

#define MOTOR_STATE_CHECK_MODE_1		((uint8_t) 0x20 )
#define MOTOR_STATE_CHECK_MODE_1_BUSY	((uint8_t) 0x2E )

#define MOTOR_STATE_CHECK_MODE_2		((uint8_t) 0x30 )
#define MOTOR_STATE_CHECK_MODE_2_BUSY	((uint8_t) 0x3E )

#define MOTOR_COMM_ERROR			((uint8_t) 0xEE )
#define MOTOR_COMM_START_UP_ERROR	((uint8_t) 0xED )
#define MOTOR_COMM_GET_MOTOR_ACCESS	((uint8_t) 0x99 )
#define MOTOR_COMM_GET_MOTOR_ACCESS_START_UP	((uint8_t) 0x9A )

// This is the declaration for tasks
volatile struct usart_module motor_serial;
TaskHandle_t motor_task_handler;
volatile struct ebm_papst_motor motor;

// Define the handler for the timer
TimerHandle_t timer_motor_comm;

//uint8_t motor_tx_array[10] = { 0 };
//uint8_t motor_rx_array[10] = { 0 };

void motor_task( void );
void motor_comm_ping_callback(TimerHandle_t pxTimer);

void ebp_papst_motor_setup( void ) {
	// Set the configuration of the pin that is going to enable the
	// transceiver
	struct port_config mot_enable_output;
	port_get_config_defaults(&mot_enable_output);
	
	mot_enable_output.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(MOTOR_TRANSCEIVER_ENABLE_PIN, &mot_enable_output);
	port_pin_set_config(MOTOR_A_SIGNAL, &mot_enable_output);
	
	// Set A signal Low Now!
	motor_disable_A();
	
	// Set the actual port
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	
	config_usart.transfer_mode = USART_TRANSFER_ASYNCHRONOUSLY;
	config_usart.generator_source = GCLK_GENERATOR_0;
	config_usart.baudrate = 115200;
	config_usart.character_size = USART_CHARACTER_SIZE_8BIT;
	config_usart.stopbits = USART_STOPBITS_1;
	config_usart.parity = USART_PARITY_EVEN;
	
	config_usart.mux_setting = USART_RX_1_TX_0_XCK_1;
	config_usart.pinmux_pad0 = PINMUX_PA08C_SERCOM0_PAD0;
	config_usart.pinmux_pad1 = PINMUX_PA09C_SERCOM0_PAD1;
	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;
	
	while ( usart_init((struct usart_module*) &motor_serial, SERCOM0, &config_usart) != STATUS_OK )
	{
	}

	usart_enable((struct usart_module*)  &motor_serial );
	
	// Set the callbacks for the serial communication protocol
	usart_register_callback((struct usart_module*) &motor_serial, (usart_callback_t) motor_callback_transmitted, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback((struct usart_module*) &motor_serial, (usart_callback_t) motor_callback_received, USART_CALLBACK_BUFFER_RECEIVED);
	
	usart_enable_callback((struct usart_module*) &motor_serial, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback((struct usart_module*) &motor_serial, USART_CALLBACK_BUFFER_RECEIVED);
}

// This always needs to be a lower priority than the motor task
void ebp_papst_motor_task_setup( void ) {
	xTaskCreate(motor_task,
		(const char *)"MOT",
		configMINIMAL_STACK_SIZE*2,
		NULL,
		2,		
		&motor_task_handler );
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
////////////////////// MOTOR CALLBACK FUNCTIONS ////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Motor Rx Callback Function
static void motor_callback_received(const struct usart_module *const module) {
	BaseType_t xYieldRequired;

	switch ( motor.motor_state ) {
		case MOTOR_STATE_CHECK_HEALTH:	// We are about to check the health of the motor
				// Check if the CRC is good
			if ( crc_check( &motor.rx_buffer, 4 ) ) {
				xTimerResetFromISR( timer_motor_comm, 0 );
				
					// Check if the motor sent successful response
				if ( motor.rx_buffer[3] == 0x00 ) {
					motor.health = motor.rx_buffer[2];
					motor.motor_state = MOTOR_STATE_HEALTH_RESPONSE;
				} else {
					motor.motor_state = MOTOR_COMM_ERROR;	// Mark motor as having reported an error
				}
				
				
				// Get back to the task, and respond accordingly
				xYieldRequired = xTaskResumeFromISR( motor_task_handler );
				if( xYieldRequired == pdTRUE )
				{
					// We should switch context so the ISR returns to a different task.
					// NOTE:  How this is done depends on the port you are using.  Check
					// the documentation and examples for your port.
					portYIELD_FROM_ISR(motor_task_handler);
				}
			}
		break;
		case MOTOR_STATE_START_UP_CHECK_HEALTH:	// This is the first health check that is performed
				// Check if the CRC is good
			if ( crc_check( &motor.rx_buffer, 4 ) ) {
				xTimerResetFromISR( timer_motor_comm, 0 );
				
					// Check if the motor sent successful response, if so, check the first mode 1
				if ( motor.rx_buffer[3] == 0x00 ) {
					motor.health = motor.rx_buffer[2];
					motor.motor_state = MOTOR_STATE_START_UP_CHECK_MODE_1;
				} else {
					motor.motor_state = MOTOR_COMM_GET_MOTOR_ACCESS_START_UP;	// Mark motor as having reported an error
				}
				
				// Get back to the task, and respond accordingly
				xYieldRequired = xTaskResumeFromISR( motor_task_handler );
				if( xYieldRequired == pdTRUE )
				{
					// We should switch context so the ISR returns to a different task.
					// NOTE:  How this is done depends on the port you are using.  Check
					// the documentation and examples for your port.
					portYIELD_FROM_ISR(motor_task_handler);
				}
			}	
		break;
		case MOTOR_STATE_START_UP_CHECK_MODE_1:		// We have received the mode 1 parameter
				// Check if the CRC is good
			if ( crc_check( &motor.rx_buffer, 7 ) ) {	// Check CRC
				xTimerResetFromISR( timer_motor_comm, 0 );
					
				// Check if the motor sent successful response, if so, check the first mode 1
					// If this is the correct mode, continue and check the next mode
				if ( motor.rx_buffer[6] == 0x00 ) {	// If we are fine and got access
					if ( motor.rx_buffer[5] == MOTOR_OPERATION_MODE_1 ) {
						// if motor is ready, continue by checking the operation mode
						motor.motor_state = MOTOR_STATE_START_UP_CHECK_MODE_2;
					} else {
						// If this is not the right Mode, Continue by setting all necessary parameters
						// Start by getting access
						// ************** At this point in time, we go into ERROR for the time being,
						// ***************** however, we must start re-parameterization at this point
						motor.motor_state = MOTOR_STATE_START_UP_CHANGE_MODE_1;
					}	
				} else {	// If this is a packet that reported an error in access
					// Change to access routine
					motor.motor_state = MOTOR_COMM_GET_MOTOR_ACCESS_START_UP;
				}
				
				// Get back to the task, and respond accordingly
				xYieldRequired = xTaskResumeFromISR( motor_task_handler );
				if( xYieldRequired == pdTRUE )
				{
					// We should switch context so the ISR returns to a different task.
					// NOTE:  How this is done depends on the port you are using.  Check
					// the documentation and examples for your port.
					portYIELD_FROM_ISR(motor_task_handler);
				}
			}
		break;
		case MOTOR_STATE_START_UP_CHECK_MODE_2:		// We have received the mode 1 parameter
				// Check if the CRC is good
			if ( crc_check( &motor.rx_buffer, 7 ) ) {	// Check CRC
				xTimerResetFromISR( timer_motor_comm, 0 );
			
				// Check if the motor sent successful response, if so, check the first mode 1
				// If this is the correct mode, continue and check the next mode
				if ( motor.rx_buffer[6] == 0x00 ) {
					if ( motor.rx_buffer[5] == MOTOR_OPERATION_MODE_2 ) {
						// if motor is ready, continue by checking the operation mode
						// At this point, this is
						motor.motor_state = MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP;
					} else {
						// If this is not the right Mode, Continue by setting all necessary parameters
						// Start by getting access
						// ************** At this point in time, we go into ERROR for the time being,
						// ***************** however, we must start re-parameterization at this point
						motor.motor_state = MOTOR_STATE_START_UP_CHANGE_MODE_1;
					}
				} else {	// If this is a packet that reported an error in access
					// Change to access routine
					motor.motor_state = MOTOR_COMM_GET_MOTOR_ACCESS_START_UP;
				}
				
				// Get back to the task, and respond accordingly
				xYieldRequired = xTaskResumeFromISR( motor_task_handler );
				if( xYieldRequired == pdTRUE )
				{
					// We should switch context so the ISR returns to a different task.
					// NOTE:  How this is done depends on the port you are using.  Check
					// the documentation and examples for your port.
					portYIELD_FROM_ISR(motor_task_handler);
				}
			}
		break;
		case MOTOR_COMM_GET_MOTOR_ACCESS_START_UP:	// This is the result of getting the motor start-up
		case MOTOR_STATE_SAVE_EEPROM:	//	This is the state where we save the parameters selected
				// Check if the CRC is good
			if ( crc_check( &motor.rx_buffer, 3 ) ) {
				xTimerResetFromISR( timer_motor_comm, 0 );
					
				// Check if the motor sent successful response, if so, check the first mode 1
				if ( motor.rx_buffer[2] == 0x00 ) {
					
					if ( motor.motor_state == MOTOR_COMM_GET_MOTOR_ACCESS_START_UP ) {
						motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;	
					} else {
						motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
					}
					
					// Get back to the task, and respond accordingly
					xYieldRequired = xTaskResumeFromISR( motor_task_handler );
					if( xYieldRequired == pdTRUE )
					{
						// We should switch context so the ISR returns to a different task.
						// NOTE:  How this is done depends on the port you are using.  Check
						// the documentation and examples for your port.
						portYIELD_FROM_ISR(motor_task_handler);
					}
				} else {
					motor.motor_state = MOTOR_COMM_GET_MOTOR_ACCESS_START_UP;	// Mark motor as having reported an error
				}
			}
		break;
		case MOTOR_STATE_START_UP_CHANGE_MODE_1:	// We will be changing mode 1 at this point
		case MOTOR_STATE_START_UP_CHANGE_MODE_2:	// We will be changing mode 2 at this point
		case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP:		// Modes for changing the ramps on the motor
		case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_DOWN:
		case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_UP:
		case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_DOWN:
				// Check if the CRC byte is OK
			if ( crc_check( &motor.rx_buffer, 9) ) {
				xTimerResetFromISR( timer_motor_comm, 0 );	// Resetting the timer value 
				
				switch ( motor.motor_state ) {
					case MOTOR_STATE_START_UP_CHANGE_MODE_1:
							// If this got entered as the correct mode, continue
						if ( motor.rx_buffer[5] == MOTOR_OPERATION_MODE_1 ) motor.motor_state = MOTOR_STATE_START_UP_CHANGE_MODE_2;
						else motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
					break;
					case MOTOR_STATE_START_UP_CHANGE_MODE_2:
							// If this got entered as the correct mode, continue
						if ( motor.rx_buffer[5] == MOTOR_OPERATION_MODE_2 ) motor.motor_state = MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP;
						else motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
					break;
					case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP:
							// If this got entered as the correct mode, continue
						if ( motor.rx_buffer[4] == MOTOR_DEFAULT_RAMP_UP_H &&
							 motor.rx_buffer[5] == MOTOR_DEFAULT_RAMP_UP_L ) motor.motor_state = MOTOR_STATE_START_UP_CHANGE_CW_RAMP_DOWN;
						else motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
					break;
					case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_DOWN:
							// If this got entered as the correct mode, continue
						if ( motor.rx_buffer[4] == MOTOR_DEFAULT_RAMP_DN_H &&
							 motor.rx_buffer[5] == MOTOR_DEFAULT_RAMP_DN_L ) motor.motor_state = MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_UP;
						else motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
					break;
					case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_UP:
							// If this got entered as the correct mode, continue
						if ( motor.rx_buffer[4] == MOTOR_DEFAULT_RAMP_UP_H &&
							 motor.rx_buffer[5] == MOTOR_DEFAULT_RAMP_UP_L ) motor.motor_state = MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_DOWN;
						else motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
					break;
					case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_DOWN:
							// If this got entered as the correct mode, continue
						if ( motor.rx_buffer[4] == MOTOR_DEFAULT_RAMP_DN_H && 
							 motor.rx_buffer[5] == MOTOR_DEFAULT_RAMP_DN_L ) motor.motor_state = MOTOR_STATE_SAVE_EEPROM;	// Send to EEPROM Save
						else motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
					break;
				}
				
				// Get back to the task, and respond accordingly
				xYieldRequired = xTaskResumeFromISR( motor_task_handler );
				if( xYieldRequired == pdTRUE )
				{
					// We should switch context so the ISR returns to a different task.
					// NOTE:  How this is done depends on the port you are using.  Check
					// the documentation and examples for your port.
					portYIELD_FROM_ISR(motor_task_handler);
				}
			}
		break;
	}
}
// Motor Tx Callback Function
static void motor_callback_transmitted(const struct usart_module *const module) {
	motor_enable_RX();	// Enable the RS485 chip to receive
	
	// Enable the Receiver back up
	usart_enable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
	
	switch ( motor.motor_state ) {
		case MOTOR_STATE_CHECK_HEALTH:
		case MOTOR_STATE_START_UP_CHECK_HEALTH:
			usart_read_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.rx_buffer, 5);
		break;
		case MOTOR_STATE_START_UP_CHECK_MODE_1:	// Check Motor-Mode 1
		case MOTOR_STATE_START_UP_CHECK_MODE_2:	// Check Motor-Mode 1
			usart_read_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.rx_buffer, 8);
		break;
		case MOTOR_COMM_GET_MOTOR_ACCESS_START_UP:
		case MOTOR_STATE_SAVE_EEPROM:
			usart_read_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.rx_buffer, 4);
		break;
		case MOTOR_STATE_START_UP_CHANGE_MODE_1:	// We will be changing mode 1 at this point
		case MOTOR_STATE_START_UP_CHANGE_MODE_2:	// We will be changing mode 2 at this point
		case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP:		// Modes for changing the ramps on the motor
		case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_DOWN:
		case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_UP:
		case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_DOWN:
			usart_read_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.rx_buffer, 10);
		break;
	}
}

void motor_enable_TX(void) {
	port_pin_set_output_level(MOTOR_TRANSCEIVER_ENABLE_PIN, pdTRUE);
}

void motor_enable_RX(void) {
	port_pin_set_output_level(MOTOR_TRANSCEIVER_ENABLE_PIN, pdFALSE);
}

void motor_enable_A(void) {
	port_pin_set_output_level(MOTOR_A_SIGNAL, pdFALSE);
}

void motor_disable_A(void) {
	port_pin_set_output_level(MOTOR_A_SIGNAL, pdTRUE);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
////////////////////// MOTOR TASK FUNCTIONS //////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void motor_task( void ) {
	// Initialize the task and motor status
	motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;	// Start with the Start Up health status check
	TickType_t xLastWakeTime;
	// This is the variable that the task is going to use to time itself at 15ms
	xLastWakeTime = xTaskGetTickCount();
	
	timer_motor_comm = xTimerCreate("Motr", 20, pdFALSE, 0, motor_comm_ping_callback);
	
		// Delay this task initially by about 200ms
	vTaskDelay(200);
	xTimerStart(timer_motor_comm, 0);	// Start timer that keeps track of Linking
	
	while(1) {	// Run this task forever
		switch ( motor.motor_state ) {
			case MOTOR_STATE_CHECK_HEALTH:	// We are about to check the health of the motor
			case MOTOR_STATE_START_UP_CHECK_HEALTH:	// This is the first health check and preceeds
														// any necessary change in mode
				motor.tx_buffer[0] = 0x05;
				motor.tx_buffer[1] = 0x01;
				crc_generate(motor.tx_buffer, 2);	// CRC generation for this packet
				
				xTimerReset( timer_motor_comm, 0 );
				motor_enable_TX();
				usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
				usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 3);
			break;
			case MOTOR_STATE_START_UP_CHECK_MODE_1:	// We will be checking mode 1 at this point
			case MOTOR_STATE_START_UP_CHECK_MODE_2:	// We will be checking mode 2 at this point
				motor.tx_buffer[0] = 0x04;
				motor.tx_buffer[1] = 0x01;
				motor.tx_buffer[2] = 0x00;
				
				if ( motor.motor_state == MOTOR_STATE_START_UP_CHECK_MODE_1 ) {		// Change request queue
					motor.tx_buffer[3] = 0x01;
				} else {
					motor.tx_buffer[3] = 0x02;
				}
				
				crc_generate(motor.tx_buffer, 4);	// CRC generation for this packet
				
				xTimerReset( timer_motor_comm, 0 );
				motor_enable_TX();
				usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
				usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 5);
			break;
			case MOTOR_COMM_GET_MOTOR_ACCESS_START_UP:	// We will be checking mode 2 at this point
				motor.tx_buffer[0] = 0x09;
				motor.tx_buffer[1] = 0x01;
				motor.tx_buffer[2] = 0x00;
				motor.tx_buffer[3] = 0x00;
				motor.tx_buffer[4] = 0x00;
				motor.tx_buffer[5] = 0x00;
			
				crc_generate(motor.tx_buffer, 6);	// CRC generation for this packet
			
				xTimerReset( timer_motor_comm, 0 );
				motor_enable_TX();
				usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
				usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 7);
			break;
			case MOTOR_STATE_START_UP_CHANGE_MODE_1:	// We will be changing mode 1 at this point
			case MOTOR_STATE_START_UP_CHANGE_MODE_2:	// We will be changing mode 2 at this point
			case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP:		// Modes for changing the ramps on the motor
			case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_DOWN:
			case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_UP:
			case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_DOWN:
				motor.tx_buffer[0] = 0x03;
				motor.tx_buffer[1] = 0x01;
				motor.tx_buffer[2] = 0x00;
				motor.tx_buffer[4] = 0x00;
				
				switch ( motor.motor_state ) {
					case MOTOR_STATE_START_UP_CHANGE_MODE_1:
						motor.tx_buffer[3] = 0x01;
						motor.tx_buffer[5] = MOTOR_OPERATION_MODE_1;
					break;
					case MOTOR_STATE_START_UP_CHANGE_MODE_2:
						motor.tx_buffer[3] = 0x02;
						motor.tx_buffer[5] = MOTOR_OPERATION_MODE_2;
					break;
					case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP:
						motor.tx_buffer[3] = 0x1A;
						motor.tx_buffer[4] = MOTOR_DEFAULT_RAMP_UP_H;
						motor.tx_buffer[5] = MOTOR_DEFAULT_RAMP_UP_L;
					break;
					case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_DOWN:
						motor.tx_buffer[3] = 0x1B;
						motor.tx_buffer[4] = MOTOR_DEFAULT_RAMP_DN_H;
						motor.tx_buffer[5] = MOTOR_DEFAULT_RAMP_DN_L;
					break;
					case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_UP:
						motor.tx_buffer[3] = 0x1C;
						motor.tx_buffer[4] = MOTOR_DEFAULT_RAMP_UP_H;
						motor.tx_buffer[5] = MOTOR_DEFAULT_RAMP_UP_L;
					break;
					case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_DOWN:
						motor.tx_buffer[3] = 0x1D;
						motor.tx_buffer[4] = MOTOR_DEFAULT_RAMP_DN_H;
						motor.tx_buffer[5] = MOTOR_DEFAULT_RAMP_DN_L;
					break;
				}
			
				crc_generate(motor.tx_buffer, 6);	// CRC generation for this packet
			
				xTimerReset( timer_motor_comm, 0 );
				motor_enable_TX();
				usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
				usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 7);
			break;
			case MOTOR_STATE_SAVE_EEPROM:
				motor.tx_buffer[0] = 0x02;
				motor.tx_buffer[1] = 0x01;
				motor.tx_buffer[2] = 0x00;
				motor.tx_buffer[3] = 0x00;
				motor.tx_buffer[4] = 0x00;
				motor.tx_buffer[5] = 0x00;
				
				crc_generate(motor.tx_buffer, 6);	// CRC generation for this packet
				
				xTimerReset( timer_motor_comm, 0 );
				motor_enable_TX();
				usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
				usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 7);
			break;
			case MOTOR_STATE_HEALTH_RESPONSE:
				xTimerReset( timer_motor_comm, 0 );
				
				if ( motor.health == 0x80 ) {
					port_pin_set_output_level(LED_ERROR, pdFALSE);	
				} else if ( motor.health == 0x40 ) {
					port_pin_set_output_level(LED_ERROR, pdTRUE);					
				}
				
				//motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
			break;
			case MOTOR_COMM_ERROR:
				xTimerReset( timer_motor_comm, 0 );
				//motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
				port_pin_set_output_level(LED_ERROR, pdTRUE);
			break;
		}
		
			// Timeout for about 15ms
		//vTaskDelayUntil(&xLastWakeTime, (TickType_t)15);
		vTaskSuspend(NULL);
	}
}

void motor_comm_ping_callback(TimerHandle_t pxTimer)
{
	system_interrupt_disable_global();
	
	configASSERT( pxTimer );
	
	
	switch ( motor.motor_state ) {
		case MOTOR_STATE_CHECK_HEALTH:
			port_pin_set_output_level(LED_ERROR, pdTRUE);
		case MOTOR_COMM_ERROR:
		case MOTOR_STATE_HEALTH_RESPONSE:
		
			if ( motor.motor_state == MOTOR_STATE_CHECK_HEALTH ) {
				motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
			} else {
				motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
			}
			
			usart_abort_job( &irda_master, USART_TRANSCEIVER_RX );
			
				// The IrDA task is now to reset and ping again
			vTaskResume( motor_task_handler );
		break;
		case MOTOR_COMM_START_UP_ERROR:
		case MOTOR_STATE_START_UP_CHECK_MODE_1:
		case MOTOR_STATE_START_UP_CHECK_MODE_2:
		case MOTOR_STATE_START_UP_CHANGE_MODE_1:	// We tried to change the mode, we must go back to step 1
		case MOTOR_STATE_START_UP_CHANGE_MODE_2:
		case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP:		// Modes for changing the ramps on the motor
		case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_DOWN:
		case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_UP:
		case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_DOWN:
		case MOTOR_STATE_SAVE_EEPROM:
			motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
		case MOTOR_STATE_START_UP_CHECK_HEALTH:
			usart_abort_job( &irda_master, USART_TRANSCEIVER_RX );	
			
			if ( motor.motor_state == MOTOR_COMM_START_UP_ERROR ||
				 motor.motor_state == MOTOR_STATE_START_UP_CHECK_HEALTH ) {
				//port_pin_set_output_level(LED_ERROR, pdTRUE);
			}
			
			// The IrDA task is now to reset and ping again
			vTaskResume( motor_task_handler );
		break;
	}
	
	system_interrupt_disable_global();
}
