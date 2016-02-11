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
#define MOTOR_RESET_ACKNOWLEDGE			((uint8_t) 0x0C )
#define MOTOR_RESET_UNDERVOLTAGE		((uint8_t) 0x0D )
#define MOTOR_RESET_TIMEOUT				((uint8_t) 0x0F )

#define MOTOR_STATE_CHECK_HEALTH		((uint8_t) 0x10 )
#define MOTOR_STATE_HEALTH_RESPONSE		((uint8_t) 0x11 )
#define MOTOR_STATE_CHECK_HEALTH_BUSY	((uint8_t) 0x1E )

#define MOTOR_STATE_CHECK_MODE_1		((uint8_t) 0x20 )
#define MOTOR_STATE_CHECK_MODE_1_BUSY	((uint8_t) 0x2E )

#define MOTOR_STATE_CHECK_MODE_2		((uint8_t) 0x30 )
#define MOTOR_STATE_CHECK_MODE_2_BUSY	((uint8_t) 0x3E )

#define MOTOR_STATE_CHANGE_CW_RAMP_UP		((uint8_t) 0x50 )
#define MOTOR_STATE_CHANGE_CW_RAMP_DOWN		((uint8_t) 0x51 )
#define MOTOR_STATE_CHANGE_CCW_RAMP_UP		((uint8_t) 0x52 )
#define MOTOR_STATE_CHANGE_CCW_RAMP_DOWN	((uint8_t) 0x53 )

#define MOTOR_STATE_RUNNING		((uint8_t) 0x70 )
#define MOTOR_STATE_DEAD_TIME	((uint8_t) 0x71 )
#define MOTOR_STATE_END_RUN		((uint8_t) 0x72 )
#define MOTOR_STATE_RUNNING_TO	((uint8_t) 0x7A )
#define MOTOR_STATE_DEAD_TO 	((uint8_t) 0x7B )

#define MOTOR_COMM_ERROR			((uint8_t) 0xEE )
#define MOTOR_COMM_START_UP_ERROR	((uint8_t) 0xED )
#define MOTOR_COMM_GET_MOTOR_ACCESS	((uint8_t) 0x99 )
#define MOTOR_COMM_GET_MOTOR_ACCESS_START_UP	((uint8_t) 0x9A )
#define MOTOR_COMM_GET_MOTOR_ACCESS				((uint8_t) 0x9B )

// Default Run constants
// We want to run constants where 
// RUN-RAMP = 500ms
// SPEED = 800 RPM
// DURATION = 1000ms
#define MOTOR_POWER_LIMIT			( 120 )	// Watts of power
#define MOTOR_POWER_LIMITER			( uint16_t ) ( ( MOTOR_POWER_LIMIT / 24 ) * 10 )	// In mA
#define MOTOR_RUN_DELAY_TIME		((TickType_t) 0 )		// Perform the action at no delay
#define MOTOR_RUN_DEAD_TIME			((TickType_t) 500 )		// Wait about 500ms between runs
//#define MOTOR_RUN_RAMP_DEFAULT		((uint16_t) 0x01F4 )	// This is the desired ramp up as specified by a data packet
#define MOTOR_RUN_RAMP_DEFAULT		((uint16_t) 200 )	// This is the desired ramp up as specified by a data packet
#define MOTOR_RUN_SPEED_DEFAULT		((uint16_t) 700 )	// This is the desired speed as specified by the data packet
//#define MOTOR_RUN_DURATION_DEFAULT	((uint16_t) 0x03E8 )	// This is the desired duration as specified by the data packet
#define MOTOR_RUN_DURATION_DEFAULT	((uint16_t) 500 )	// This is the desired duration as specified by the data packet
#define MOTOR_DEFAULT_CURRENT	( (uint8_t) 12 )	//% Percentage
#define MOTOR_DEFAULT_UNDERVOLTAGE ((uint16_t) 1650 )	// x 10mV

// This is the declaration for tasks
volatile struct usart_module motor_serial;
TaskHandle_t motor_task_handler;
volatile struct ebm_papst_motor motor;

// Define the handler for the timer
TimerHandle_t timer_motor_comm;

volatile int16_t current_consumption;

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
	//int16_t temp_s16bit_integer;

	switch ( motor.motor_state ) {
		case MOTOR_STATE_CHECK_HEALTH:	// We are about to check the health of the motor
				// Check if the CRC is good
			if ( crc_check( &motor.rx_buffer, 4 ) ) {
				xTimerResetFromISR( timer_motor_comm, 0 );
				
					// Check if the motor sent successful response
				if ( motor.rx_buffer[3] == 0x00 ) {
					motor.health = motor.rx_buffer[2];
					if ( motor.health & 0x08 ) {
							// Motor needs acknowledgment
						motor_disable_A();
						motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
						port_pin_set_output_level(LED_ERROR, pdTRUE);
						motor.is_motor_error = true;
						//motor.motor_state = MOTOR_RESET_UNDERVOLTAGE;
					} else if ( motor.health & 0x40 ) {
							// Motor needs acknowledgment
						motor_disable_A();
						motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
						port_pin_set_output_level(LED_ERROR, pdTRUE);
						motor.is_motor_error = true;
						//motor.motor_state = MOTOR_RESET_ACKNOWLEDGE;	
					} else {
						 motor.motor_state = MOTOR_STATE_HEALTH_RESPONSE;
						 motor.is_motor_error = false;
					}
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
					if ( motor.health & 0x08 ) {
							// Motor needs acknowledgment
						motor_disable_A();
						motor.motor_state_callback = MOTOR_STATE_START_UP_CHECK_HEALTH;
						motor.motor_state = MOTOR_RESET_UNDERVOLTAGE;
						port_pin_set_output_level(LED_ERROR, pdTRUE);
						motor.is_motor_error = true;
					} else if ( motor.health & 0x40 ) {
							// Motor needs acknowledgment
						motor_disable_A();
						motor.motor_state_callback = MOTOR_STATE_START_UP_CHECK_HEALTH;
						motor.motor_state = MOTOR_RESET_ACKNOWLEDGE;
						port_pin_set_output_level(LED_ERROR, pdTRUE);	
						motor.is_motor_error = true;
					} else {
						motor.motor_state = MOTOR_STATE_START_UP_CHECK_MODE_1;
						motor.is_motor_error = false;
					}
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
		case MOTOR_STATE_RUNNING:
		case MOTOR_STATE_DEAD_TIME:
			if ( crc_check( &motor.rx_buffer, 12) ) {
				xTimerResetFromISR( timer_motor_comm, 0 );
				
				motor.health = motor.rx_buffer[10];
				
				if ( motor.health & 0x48 ) {	// If there is an undervoltage error
					motor.motor_state = MOTOR_STATE_END_RUN;
				} else {	// Everything is fine
					current_consumption = 0x00;
					current_consumption |= (int16_t) (motor.rx_buffer[4] << 8);
					current_consumption |= (int16_t) (motor.rx_buffer[5]);
					current_consumption = current_consumption * 10;	// Current usage in mA
					// ############## We have to handle errors in this area
					
					if ( current_consumption < 0 ) {
						current_consumption = current_consumption * (-1);
					}
					
					if ( motor.motor_state == MOTOR_STATE_RUNNING &&
					motor.rx_buffer[2] == 0 && motor.rx_buffer[3] == 0 && current_consumption > motor.allocated_current ) {
						port_pin_set_output_level(LED_ERROR, pdTRUE);
						motor.rx_buffer[2] = 0xEE;
						
						motor.speed_high_nibble = 0;
						motor.speed_low_nibble = 0;
						
						if ( !motor.motor_job_report ) motor.motor_job_report = 0xCC;	// Job did not complete due to over-current
						
						motor_disable_A();
						
					}
					
					// Continue with case logic for the time being
					if ( motor.motor_state == MOTOR_STATE_RUNNING ) {
						motor.motor_state = MOTOR_STATE_RUNNING_TO;
					} else {
						motor.motor_state = MOTOR_STATE_DEAD_TO;
					}
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
		case MOTOR_STATE_CHANGE_CW_RAMP_UP:		// Modes for changing the ramps on the motor
		case MOTOR_STATE_CHANGE_CW_RAMP_DOWN:
		case MOTOR_STATE_CHANGE_CCW_RAMP_UP:
		case MOTOR_STATE_CHANGE_CCW_RAMP_DOWN:
			// Check if the CRC byte is OK
			if ( crc_check( &motor.rx_buffer, 9) ) {
				xTimerResetFromISR( timer_motor_comm, 0 );	// Resetting the timer value
				
				if ( motor.rx_buffer[8] == 0x00 ) {
					if ( motor.rx_buffer[4] == motor.ramp_high_nibble && motor.rx_buffer[5] == motor.ramp_low_nibble )
					{
							switch ( motor.motor_state ) {
								case MOTOR_STATE_CHANGE_CW_RAMP_UP:
									motor.motor_state = MOTOR_STATE_CHANGE_CW_RAMP_DOWN;
								break;
								case MOTOR_STATE_CHANGE_CW_RAMP_DOWN:
									motor.motor_state = MOTOR_STATE_CHANGE_CCW_RAMP_UP;
								break;
								case MOTOR_STATE_CHANGE_CCW_RAMP_UP:
									motor.motor_state = MOTOR_STATE_CHANGE_CCW_RAMP_DOWN;
								break;
								case MOTOR_STATE_CHANGE_CCW_RAMP_DOWN:
									if ( motor.is_motor_error ) motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
									else motor.motor_state = MOTOR_STATE_RUNNING;	// Ramps saved, get to running the motor
								break;
							}
						} else {
						motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
					}
				} else if ( motor.rx_buffer[8] == 0x10 ) {
					motor.motor_state_callback = motor.motor_state;
					motor.motor_state = MOTOR_COMM_GET_MOTOR_ACCESS;
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
						motor.motor_state_callback = MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP;
						motor.motor_state = MOTOR_RESET_UNDERVOLTAGE;
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
		case MOTOR_COMM_GET_MOTOR_ACCESS:
		case MOTOR_COMM_GET_MOTOR_ACCESS_START_UP:	// This is the result of getting the motor start-up
		case MOTOR_STATE_SAVE_EEPROM:	//	This is the state where we save the parameters selected
				// Check if the CRC is good
			if ( crc_check( &motor.rx_buffer, 3 ) ) {
				xTimerResetFromISR( timer_motor_comm, 0 );
					
				// Check if the motor sent successful response, if so, check the first mode 1
				if ( motor.rx_buffer[2] == 0x00 || motor.rx_buffer[2] == 0x08 ) {
					
					if ( motor.motor_state == MOTOR_COMM_GET_MOTOR_ACCESS_START_UP ) {
						motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;	
					} else if ( motor.motor_state == MOTOR_COMM_GET_MOTOR_ACCESS ) {
						motor.motor_state = motor.motor_state_callback;
					} else {
						motor.is_motor_ready = true;	// Motor is now ready to run
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
		case MOTOR_RESET_ACKNOWLEDGE:
		case MOTOR_RESET_UNDERVOLTAGE:
				// Check if the CRC byte is OK
			if ( crc_check( &motor.rx_buffer, 9) ) {
				xTimerResetFromISR( timer_motor_comm, 0 );	// Resetting the timer value 
				
				if ( motor.rx_buffer[8] == 0x10 ) motor.motor_state = MOTOR_COMM_GET_MOTOR_ACCESS;
				else motor.motor_state = motor.motor_state_callback;	// Go back to where we came from
				
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
		case MOTOR_STATE_RUNNING:
		case MOTOR_STATE_DEAD_TIME:
			usart_read_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.rx_buffer, 13);
		break;
		case MOTOR_STATE_START_UP_CHECK_MODE_1:	// Check Motor-Mode 1
		case MOTOR_STATE_START_UP_CHECK_MODE_2:	// Check Motor-Mode 1
			usart_read_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.rx_buffer, 8);
		break;
		case MOTOR_COMM_GET_MOTOR_ACCESS_START_UP:
		case MOTOR_COMM_GET_MOTOR_ACCESS:
		case MOTOR_STATE_SAVE_EEPROM:
			usart_read_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.rx_buffer, 4);
		break;
		case MOTOR_STATE_START_UP_CHANGE_MODE_1:	// We will be changing mode 1 at this point
		case MOTOR_STATE_START_UP_CHANGE_MODE_2:	// We will be changing mode 2 at this point
		case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_UP:		// Modes for changing the ramps on the motor
		case MOTOR_STATE_START_UP_CHANGE_CW_RAMP_DOWN:
		case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_UP:
		case MOTOR_STATE_START_UP_CHANGE_CCW_RAMP_DOWN:
		case MOTOR_STATE_CHANGE_CW_RAMP_UP:		// Modes for changing the ramps on the motor
		case MOTOR_STATE_CHANGE_CW_RAMP_DOWN:
		case MOTOR_STATE_CHANGE_CCW_RAMP_UP:
		case MOTOR_STATE_CHANGE_CCW_RAMP_DOWN:
		case MOTOR_RESET_ACKNOWLEDGE:
		case MOTOR_RESET_UNDERVOLTAGE:
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
	uint32_t temp_32bit_variable;
	uint16_t temp_16bit_variable;
	uint8_t temp_8bit_variable;
	TickType_t currentTickCount;
	
	// Initialize the task and motor status
	motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;	// Start with the Start Up health status check
	motor.is_motor_ready = false;		// The motor is NOT ready to run
	motor.is_motor_running = false;
	motor.is_motor_queued = false;
	motor.is_motor_error = false;
	motor.job_is_incoming = false;
	motor.motor_report_past_job = true;
	
	timer_motor_comm = xTimerCreate("Motr", 20, pdFALSE, 0, motor_comm_ping_callback);
	
	
	
		// Delay this task initially by about 200ms
	vTaskDelay(200);
	xTimerStart(timer_motor_comm, 0);	// Start timer that keeps track of Linking
	
	while(1) {	// Run this task forever
		switch ( motor.motor_state ) {
			case MOTOR_STATE_CHECK_HEALTH:	// We are about to check the health of the motor
			case MOTOR_STATE_START_UP_CHECK_HEALTH:	// This is the first health check and preceeds
														// any necessary change in mode
					// Check if the motor is ready to run
				if ( motor.is_motor_queued && motor.is_motor_ready && !motor.is_motor_error ) {	// There is a job waiting
						// At this point, we set all the data necessary for the motor to run
					motor.allocated_power = motor.rx_data_B[3];
					motor.motor_job_number = motor.rx_data_A[0];
						// Getting the Ramp
					motor.rx_ramp_value = ( uint16_t ) motor.rx_data_A[3];
					motor.rx_ramp_value = motor.rx_ramp_value << 4;
					temp_8bit_variable = motor.rx_data_A[2];
					temp_8bit_variable = temp_8bit_variable & 0x0F;
					temp_16bit_variable = ( uint16_t ) temp_8bit_variable;
					temp_16bit_variable = temp_16bit_variable << 8;
					motor.rx_ramp_value = motor.rx_ramp_value | temp_16bit_variable; // Ramp Value Obtained!
						// Getting the duration
					motor.rx_duration_value = ( uint16_t ) motor.rx_data_B[2];
					motor.rx_duration_value = motor.rx_duration_value << 4;
					temp_8bit_variable = motor.rx_data_B[1];
					temp_8bit_variable = temp_8bit_variable & 0x0F;
					temp_16bit_variable = ( uint16_t ) temp_8bit_variable;
					temp_16bit_variable = temp_16bit_variable << 8;
					motor.rx_duration_value = motor.rx_duration_value | temp_16bit_variable; // Ramp Value Obtained!
						// Getting the Speed
					temp_16bit_variable = 0;
					temp_16bit_variable = ( uint16_t ) motor.rx_data_B[0];
					motor.rx_speed_value = ( int16_t ) temp_16bit_variable << 8;
					temp_8bit_variable = ( uint8_t ) motor.rx_data_B[1];
					temp_8bit_variable = temp_8bit_variable & 0xF0;
					motor.rx_speed_value = motor.rx_speed_value | (int16_t)temp_8bit_variable; // Speed value obtained!
						// Getting the delay
					temp_16bit_variable = 0;
					temp_16bit_variable = (uint16_t) motor.rx_data_A[1];
					motor.rx_delay_value = (uint16_t) temp_16bit_variable << 4;
					temp_8bit_variable = (uint8_t) motor.rx_data_A[2] & 0xF0;
					motor.rx_delay_value = motor.rx_delay_value | (uint16_t) temp_8bit_variable;	// Delay value obtained!
					
					
						// We populate the delay duration at this point
					motor.run_delay_duration = xTaskGetTickCount();
					motor.run_delay_duration += (TickType_t) motor.rx_delay_value;
					
						// we will change the operating mode and start by doing the necessary math 
					motor.run_total_duration = ( TickType_t ) ( (motor.rx_ramp_value * 2) + motor.rx_duration_value );
					motor.run_total_duration += motor.run_delay_duration;	// Add to the total duration with a dead_time included
					motor.motor_dead_time = motor.run_total_duration + MOTOR_RUN_DEAD_TIME;	// Adding the dead-time
					
						// We now set the time for the duration of non-zero data to the motor
					motor.run_nzdata_duration = ( TickType_t ) ( motor.rx_ramp_value + motor.rx_duration_value );
					motor.run_nzdata_duration += motor.run_delay_duration;
					
						// Then we set the adjusted ramp time that is to be sent to the motor
					temp_32bit_variable = (uint32_t) ( motor.rx_ramp_value * 1000 );
					if ( motor.rx_speed_value < 0 ) {	// If speed is a negative value, we need to get the positive
						temp_16bit_variable = motor.rx_speed_value * (-1);	// Change the sign
						temp_32bit_variable = temp_32bit_variable / temp_16bit_variable;
					} else {
						temp_32bit_variable = temp_32bit_variable / motor.rx_speed_value;
					}
					motor.motor_ramp_ms = ( uint16_t ) temp_32bit_variable;	// We truncate the calculated value and store
					
						// We store and save the motor run speed
					motor.motor_speed_rpm = motor.rx_speed_value;	// Raw, no need to convert
					
						// At this point, the variables to prep the motor have been set, we proceed to send the necessary
							// variables to make the motor run as desired
					motor.is_motor_running = true;
					
						// We now set the correct data variables;
					temp_16bit_variable = motor.motor_ramp_ms >> 8;	// bit-shift the ramp and save it to the utility
					motor.ramp_high_nibble = ( uint8_t ) temp_16bit_variable;
					temp_16bit_variable = motor.motor_ramp_ms & 0x00FF;	// Mask to obtain lower nibble 
					motor.ramp_low_nibble = ( uint8_t ) temp_16bit_variable;
					
					temp_16bit_variable = motor.motor_speed_rpm >> 8;	// bit-shift the ramp and save it to the utility
					motor.speed_high_nibble = ( uint8_t ) temp_16bit_variable;
					temp_16bit_variable = motor.motor_speed_rpm & 0x00FF;	// Mask to obtain lower nibble
					motor.speed_low_nibble = ( uint8_t ) temp_16bit_variable;
					
						// Determine the allocated power for this slat
					motor.allocated_current = motor.allocated_power * MOTOR_POWER_LIMITER;
					
						// At this point, we must send all the ramps to the motor
							// Change the state
					motor.motor_state = MOTOR_STATE_CHANGE_CW_RAMP_UP;
					
					motor.motor_job_report = 0x00;	// Start with a clean job report
					
						// Populate the buffer
					motor.tx_buffer[0] = 0x03;
					motor.tx_buffer[1] = 0x01;
					motor.tx_buffer[2] = 0x00;
					motor.tx_buffer[3] = 0x1A;
					motor.tx_buffer[4] = motor.ramp_high_nibble;
					motor.tx_buffer[5] = motor.ramp_low_nibble;
					motor_enable_A();	// Enable the motor to start running any commands
					port_pin_set_output_level(LED_ERROR, pdFALSE);
					
						// Calculate the CRC
					crc_generate(motor.tx_buffer, 6);	// CRC generation for this packet
					
					// Send the data to the motor
					xTimerReset( timer_motor_comm, 0 );
					motor_enable_TX();
					usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
					usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 7);
				} else {
					if ( motor.motor_state == MOTOR_STATE_START_UP_CHECK_HEALTH ) {
 						motor.motor_state_callback = motor.motor_state;
 						motor.motor_state = MOTOR_RESET_TIMEOUT;
 						vTaskDelay(500);
 						motor.motor_state = motor.motor_state_callback;
 					}
					
					if ( motor.health & 0x80 ) { 
						//motor.is_motor_queued = true;
						port_pin_set_output_level(LED_ERROR, pdFALSE);
					}
					
					motor.tx_buffer[0] = 0x05;
					motor.tx_buffer[1] = 0x01;
					crc_generate(motor.tx_buffer, 2);	// CRC generation for this packet
					
					xTimerReset( timer_motor_comm, 0 );
					motor_enable_TX();
					usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
					usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 3);	
				}
			break;
			case MOTOR_STATE_RUNNING:	// The Motor is running at this stage
			case MOTOR_STATE_DEAD_TIME:
				motor.tx_buffer[0] = 0x00;
				motor.tx_buffer[1] = 0x01;
				
				currentTickCount = xTaskGetTickCount();
					// At this point, we have to check the speed that we have to send
				if ( currentTickCount < motor.run_delay_duration ) {
					//port_pin_set_output_level(LED_ERROR, pdTRUE);
					motor.tx_buffer[2] = 0x00;
					motor.tx_buffer[3] = 0x00;	// Set the run speed to ZERO
				} else if ( currentTickCount <= motor.run_nzdata_duration ) {	// If we are still within the
					//port_pin_set_output_level(LED_ERROR, pdFALSE);
					motor.tx_buffer[2] = motor.speed_high_nibble;
					motor.tx_buffer[3] = motor.speed_low_nibble;	
				} else if ( currentTickCount <= motor.run_total_duration ) {	// If we are to ramp down
					motor.tx_buffer[2] = 0x00;
					motor.tx_buffer[3] = 0x00;	// Set the run speed to ZERO
				} else if ( currentTickCount <= motor.motor_dead_time) {
					motor.tx_buffer[2] = 0x00;
					motor.tx_buffer[3] = 0x00;	// Set the run speed to ZERO
					motor.motor_state = MOTOR_STATE_DEAD_TIME;
					
					motor_disable_A();	// Disable the pin on the motor
				} else {	// This is the moment that we go into dead time
					
					motor.tx_buffer[2] = 0x00;
					motor.tx_buffer[3] = 0x00;	// Set the run speed to ZERO
					motor.motor_state = MOTOR_STATE_END_RUN;
				}
				
				motor.tx_buffer[4] = 0x00;
				motor.tx_buffer[5] = motor.allocated_power;	// 20% current allocation, constant for now
				
				crc_generate(motor.tx_buffer, 6);	// CRC generation for this packet
				
				xTimerReset( timer_motor_comm, 0 );
				motor_enable_TX();
				usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
				usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 7);
			break;
			case MOTOR_STATE_CHANGE_CW_RAMP_UP:		// Modes for changing the ramps on the motor
			case MOTOR_STATE_CHANGE_CW_RAMP_DOWN:
			case MOTOR_STATE_CHANGE_CCW_RAMP_UP:
			case MOTOR_STATE_CHANGE_CCW_RAMP_DOWN:
				motor.tx_buffer[0] = 0x03;
				motor.tx_buffer[1] = 0x01;
				motor.tx_buffer[2] = 0x00;
			
				switch ( motor.motor_state ) {
					case MOTOR_STATE_CHANGE_CW_RAMP_UP:
						motor.tx_buffer[3] = 0x1A;
					break;
					case MOTOR_STATE_CHANGE_CW_RAMP_DOWN:
						motor.tx_buffer[3] = 0x1B;
					break;
					case MOTOR_STATE_CHANGE_CCW_RAMP_UP:
						motor.tx_buffer[3] = 0x1C;
					break;
					case MOTOR_STATE_CHANGE_CCW_RAMP_DOWN:
						motor.tx_buffer[3] = 0x1D;
					break;
				}
				
				motor.tx_buffer[4] = motor.ramp_high_nibble;
				motor.tx_buffer[5] = motor.ramp_low_nibble;
			
				crc_generate(motor.tx_buffer, 6);	// CRC generation for this packet
			
				xTimerReset( timer_motor_comm, 0 );
				motor_enable_TX();
				usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
				usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 7);
			break;
			case MOTOR_STATE_START_UP_CHECK_MODE_1:	// We will be checking mode 1 at this point
			case MOTOR_STATE_START_UP_CHECK_MODE_2:	// We will be checking mode 2 at this point
				motor.tx_buffer[0] = 0x04;
				motor.tx_buffer[1] = 0x01;
				motor.tx_buffer[2] = 0x00;
				
				if ( motor.motor_state == MOTOR_STATE_START_UP_CHECK_MODE_1 ) {		// Change request queue
					motor.tx_buffer[3] = 0x01; // THIS WAS JUST CHANGED
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
			case MOTOR_COMM_GET_MOTOR_ACCESS:	// We will be checking mode 2 at this point
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
			case MOTOR_STATE_RUNNING_TO:
			case MOTOR_STATE_DEAD_TO:
				xTimerReset( timer_motor_comm, 0 );
				
				if ( motor.health == 0x80 ) {
					//port_pin_set_output_level(LED_ERROR, pdFALSE);
				} else if ( motor.health & 0x08 ) {
					port_pin_set_output_level(LED_ERROR, pdTRUE);
				}
			break;
			case MOTOR_STATE_HEALTH_RESPONSE:
				xTimerReset( timer_motor_comm, 0 );
				
				if ( motor.health == 0x80 ) {
					//port_pin_set_output_level(LED_ERROR, pdFALSE);	
				} else if ( motor.health & 0x08 ) {
					port_pin_set_output_level(LED_ERROR, pdTRUE);					
				}
				
				//motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
			break;
			case MOTOR_COMM_ERROR:
				xTimerReset( timer_motor_comm, 0 );
				//motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
				port_pin_set_output_level(LED_ERROR, pdTRUE);
			break;
			case MOTOR_RESET_ACKNOWLEDGE:
			case MOTOR_RESET_UNDERVOLTAGE:
				motor.tx_buffer[0] = 0x03;
				motor.tx_buffer[1] = 0x01;
				motor.tx_buffer[2] = 0x00;
				if ( motor.motor_state == MOTOR_RESET_ACKNOWLEDGE ) {
					motor.tx_buffer[3] = 0x06;
					motor.tx_buffer[4] = 0x00;
					motor.tx_buffer[5] = 0x00;
				} else {
						// We now set the correct data variables;
					temp_16bit_variable = MOTOR_DEFAULT_UNDERVOLTAGE >> 8;	// bit-shift the ramp and save it to the utility
					motor.tx_buffer[4] = ( uint8_t ) temp_16bit_variable;
					temp_16bit_variable = MOTOR_DEFAULT_UNDERVOLTAGE & 0x00FF;	// Mask to obtain lower nibble 
					motor.tx_buffer[5] = ( uint8_t ) temp_16bit_variable;
					
					motor.tx_buffer[3] = 0x4B;
				}
				
				motor_enable_A();
				
				crc_generate(motor.tx_buffer, 6);	// CRC generation for this packet
				
				xTimerReset( timer_motor_comm, 0 );
				motor_enable_TX();
				usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
				usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 7);
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
		case MOTOR_STATE_RUNNING_TO:
			motor.motor_state = MOTOR_STATE_RUNNING;
				// The IrDA task is now to reset and ping again
			vTaskResume( motor_task_handler );
		break;
		case MOTOR_STATE_DEAD_TO:
			motor.motor_state = MOTOR_STATE_DEAD_TIME;	
				// The IrDA task is now to reset and ping again
			vTaskResume( motor_task_handler );
		break;
		case MOTOR_STATE_CHECK_HEALTH:
		case MOTOR_STATE_RUNNING:	// Motor did not respond at the time when it was supposed to run
		case MOTOR_STATE_DEAD_TIME:
			motor.is_motor_running = false;
			motor.is_motor_ready = false;
			motor.is_motor_queued = false;
			motor.health = 0x00;	// Motor is no longer there....
			port_pin_set_output_level(LED_ERROR, pdTRUE);
		case MOTOR_COMM_ERROR:
		case MOTOR_STATE_HEALTH_RESPONSE:
		
			if ( motor.motor_state == MOTOR_STATE_CHECK_HEALTH ) {
				motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
			} else {
				motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
			}
			
			usart_abort_job( &motor_serial, USART_TRANSCEIVER_RX );
			
				// The IrDA task is now to reset and ping again
			vTaskResume( motor_task_handler );
		break;
		case MOTOR_STATE_START_UP_CHECK_HEALTH:
			motor.health = 0x00;	// Motor is no longer there....
			port_pin_set_output_level(LED_ERROR, pdTRUE);
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
		case MOTOR_RESET_ACKNOWLEDGE:
		case MOTOR_RESET_UNDERVOLTAGE:
		case MOTOR_STATE_CHANGE_CW_RAMP_UP:		// Modes for changing the ramps on the motor
		case MOTOR_STATE_CHANGE_CW_RAMP_DOWN:
		case MOTOR_STATE_CHANGE_CCW_RAMP_UP:
		case MOTOR_STATE_CHANGE_CCW_RAMP_DOWN:
		case MOTOR_COMM_GET_MOTOR_ACCESS_START_UP:
		case MOTOR_COMM_GET_MOTOR_ACCESS:
			motor.motor_state = MOTOR_STATE_START_UP_CHECK_HEALTH;
			usart_abort_job( &motor_serial, USART_TRANSCEIVER_RX );	
			
			if ( motor.motor_state == MOTOR_COMM_START_UP_ERROR ||
				 motor.motor_state == MOTOR_STATE_START_UP_CHECK_HEALTH ) {
				//port_pin_set_output_level(LED_ERROR, pdTRUE);
			}
			
			// The IrDA task is now to reset and ping again
			vTaskResume( motor_task_handler );
		break;
		case MOTOR_STATE_END_RUN:	// This is the case when the run has been ended
			motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
			
			motor.is_motor_running = false;
			motor.is_motor_queued = false;
			
				// Since we have finished a run, go ahead and report the job
			motor.motor_report_past_job = true;
			if ( !motor.motor_job_report ) motor.motor_job_report = 0xFF;	// Job Completed fine
			
			// The IrDA task is now to reset and ping again
			vTaskResume( motor_task_handler );
		break;
	}
	
	system_interrupt_disable_global();
}
