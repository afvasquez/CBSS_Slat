#include "asf.h"
#include "bastian_sercom.h"

	// State Definition
#define MOTOR_STATE_CHECK_HEALTH		((uint8_t) 0x10 )
#define MOTOR_STATE_HEALTH_RESPONSE		((uint8_t) 0x11 )
#define MOTOR_STATE_CHECK_HEALTH_BUSY	((uint8_t) 0x1E )

#define MOTOR_STATE_CHECK_MODE_1		((uint8_t) 0x20 )
#define MOTOR_STATE_CHECK_MODE_1_BUSY	((uint8_t) 0x2E )

#define MOTOR_STATE_CHECK_MODE_2		((uint8_t) 0x30 )
#define MOTOR_STATE_CHECK_MODE_2_BUSY	((uint8_t) 0x3E )

#define MOTOR_COMM_ERROR	((uint8_t) 0xEE )

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
				
					// Check if the motor sent successful reponse
				if ( motor.rx_buffer[3] == 0x00 ) {
					motor.health = motor.rx_buffer[2];
					motor.motor_state = MOTOR_STATE_HEALTH_RESPONSE;
				} else {
					motor.motor_state = MOTOR_COMM_ERROR;	// Mark motor as having reported an error
				}
			}
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
// Motor Tx Callback Function
static void motor_callback_transmitted(const struct usart_module *const module) {
	motor_enable_RX();	// Enable the RS485 chip to receive
	
	// Enable the Receiver back up
	usart_enable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
	
	switch ( motor.motor_state ) {
		case MOTOR_STATE_CHECK_HEALTH:
			usart_read_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.rx_buffer, 5);
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
	motor.motor_state = MOTOR_STATE_CHECK_HEALTH;	// Set the state to check the state
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
				motor.tx_buffer[0] = 0x05;
				motor.tx_buffer[1] = 0x01;
				crc_generate(motor.tx_buffer, 2);	// CRC generation for this packet
				
				xTimerReset( timer_motor_comm, 0 );
				motor_enable_TX();
				usart_disable_transceiver((struct usart_module*) &motor_serial, USART_TRANSCEIVER_RX);
				usart_write_buffer_job((struct usart_module*) &motor_serial, (uint8_t*)motor.tx_buffer, 3);
			break;
			case MOTOR_STATE_HEALTH_RESPONSE:
				xTimerReset( timer_motor_comm, 0 );
				motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
			break;
			case MOTOR_COMM_ERROR:
				xTimerReset( timer_motor_comm, 0 );
				motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
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
		case MOTOR_STATE_HEALTH_RESPONSE:
		case MOTOR_COMM_ERROR:
			motor.motor_state = MOTOR_STATE_CHECK_HEALTH;
			usart_abort_job( &irda_master, USART_TRANSCEIVER_RX );
		
				// The IrDA task is now to reset and ping again
			vTaskResume( motor_task_handler );
		break;
	}
	
	system_interrupt_disable_global();
}
