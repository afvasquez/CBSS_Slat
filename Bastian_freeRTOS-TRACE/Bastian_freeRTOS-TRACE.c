/*
 * First-FreeRTOS_Trace.c
 *
 * Created: 11/19/2015 10:56:40 AM
 * Author : avasquez
 */ 



#include "asf.h"
#include "bastian_sercom.h"

void irda_communication_task(void);
void timer_irda_ping_callback(TimerHandle_t pxTimer);
void timer_irda_sync_callback(TimerHandle_t pxTimer);

// Define the handler for the timer
// Define the handler for the timer
TimerHandle_t timer_IrDA_Ping;
TimerHandle_t timer_IrDA_Sync;

TaskHandle_t irda_task_handler;
traceLabel event_channel;

//struct tc_module tc_instance;

int main(void)
{
    /* Initialize the SAM system */
	system_init();
	
	
	//////////////////////////////////////////////////////////////////////////
	// Set the LED outputs for this board.
	struct port_config led_out;
	port_get_config_defaults(&led_out);
	
	led_out.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_BUSY, &led_out);
	port_pin_set_config(LED_ERROR, &led_out);
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	// Start the IrDA communication port
	bastian_IrDA_configuration();
	
	// Start the trace logger
	vTraceInitTraceData();
	
	
	
	/* Initialization code - create the channel label for a VTracePrintF*/
	event_channel = xTraceOpenLabel("Debug");

	
	
	// Create the task
	xTaskCreate(irda_communication_task,
					(const char *)"IrDA",
					configMINIMAL_STACK_SIZE*3,
					NULL,
					2,
					&irda_task_handler );
					
	
	// Enable global interrupts
	system_interrupt_enable_global();
	
	// Create the necessary timer
	timer_IrDA_Ping = xTimerCreate("Ping", 2, pdFALSE, 0, timer_irda_ping_callback);
	timer_IrDA_Sync = xTimerCreate("Sync", 1, pdFALSE, 1, timer_irda_sync_callback );
	xTimerStart(timer_IrDA_Ping, 0);	// Start timer that keeps track of Linking
	//xTimerStart(timer_IrDA_Sync, 0);	// Start ping timer
	
	// ..and let FreeRTOS run tasks!
	vTaskStartScheduler();

    /* Replace with your application code */
    while (1) 
    {
		
    }
}

//#define IRDA_BEACON_PING	(( uint8_t ) 0x01 )		// This is the 
uint8_t irda_comm_state;
uint8_t irda_tx_array[6] = { 0 };
uint8_t irda_rx_array[6] = { 0 };
void irda_communication_task(void) {
	
	// Start this task by pinging out
	irda_comm_state = IRDA_SLAT_PING;
	
	while (1) {
		//port_pin_toggle_output_level(LED_BUSY);
		//vTracePrintF(event_channel, "IRDA: %d", irda_comm_state);
		switch( irda_comm_state )
		{
			case IRDA_SLAT_PING:
				irda_timed_out = pdFALSE;
				
				// Start the necessary timers 
				//vTracePrintF(event_channel, "Rx Request.");
				xTimerReset( timer_IrDA_Ping, 0 );
				
				port_pin_set_output_level(LED_ERROR, pdTRUE);
				
				usart_enable_transceiver( &irda_master, USART_TRANSCEIVER_RX );	// Enable Receiving Transceiver
				usart_read_buffer_job( &irda_master, irda_rx_array, 3 );	// Try to get the 3-Byte ping
			break;
			case IRDA_SLAT_FIRST:  // Send the response back and reset
				// Send out the ping and wait
				irda_tx_array[0] = 0xBB;
				irda_tx_array[1] = 0xBB;
				irda_tx_array[2] = 0xBB;
				irda_tx_array[3] = 0xBB;
				irda_tx_array[4] = 0xBB;
				
				vTracePrintF(event_channel, "Send Resp.");
				
				// Send this data now
				usart_write_buffer_job(&irda_master, irda_tx_array, 5);
			break;
			case IRDA_SLAT_FIRST_RESPONSE:	// This is the action taken for
				// Reset the response timer
				xTimerReset( timer_IrDA_Ping, 0 );
				
				// Set the ERROR LED to indicate the start of an Rx, sampling sequence
				port_pin_set_output_level(LED_BUSY, pdTRUE);
				
				
				//port_pin_toggle_output_level(LED_BUSY);
				
				
			break;
			case IRDA_SLAT_RESET:
				
			break;	
		}
		
		system_interrupt_enable_global();
		vTaskSuspend( NULL );
		system_interrupt_disable_global();
		//xTimerStart(timer_IrDA_link, 0);
	}
}


// vTracePrintF(event_channel, "IrDA Reset!");
void timer_irda_ping_callback(TimerHandle_t pxTimer) 
{
	system_interrupt_disable_global();
	
	configASSERT( pxTimer );
	
	switch ( irda_comm_state ) {
		case IRDA_SLAT_FIRST_RESPONSE:
			irda_comm_state = IRDA_SLAT_PING;	// Go back to the Ping Mode
		case IRDA_SLAT_PING:
			irda_timed_out = pdTRUE;
			
			port_pin_set_output_level(LED_BUSY, pdFALSE);
			//vTracePrintF(event_channel, "Ping TO!");
			//port_pin_set_output_level(LED_ERROR, pdFALSE);
			// There was no significant response to the ping,
			// Reset accordingly
			usart_abort_job( &irda_master, USART_TRANSCEIVER_RX );
			
			// The IrDA task is now to reset and ping again
			vTaskResume( irda_task_handler );
		break;
	}	
	
	system_interrupt_disable_global();
}

void timer_irda_sync_callback(TimerHandle_t pxTimer) 
{
	configASSERT( pxTimer );
	
	// This is the timeout for no response from the Beacon
	// If the system reaches this point at the intended state, 
	//	we proceed 
	
	switch ( irda_comm_state ) {
		case IRDA_SLAT_FIRST_RESPONSE:
		irda_timed_out = pdTRUE;
		irda_comm_state = IRDA_SLAT_PING;	// Go back to the Ping Mode
		
		//port_pin_set_output_level(LED_BUSY, pdFALSE);
		//vTracePrintF(event_channel, "Sync TO!");
		//port_pin_set_output_level(LED_ERROR, pdFALSE);
		// There was no significant response to the ping,
		// Reset accordingly
		usart_abort_job( &irda_master, USART_TRANSCEIVER_RX );
		
		// The IrDA task is now to reset and ping again
		//vTaskResume( irda_task_handler );
		break;
	}
	
}