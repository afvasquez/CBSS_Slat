#include "asf.h"
#include "bastian_sercom.h"

static void irda_master_callback_received(const struct usart_module *const module);
static void irda_master_callback_transmitted(const struct usart_module *const module);

//////////////////////////////////////////////////////////////////////////
// IrDA Port COnfiguration
void bastian_IrDA_configuration (void){
	// USART Configuration Structure
	struct usart_config irda_conf;
	
	// Get defaults for this protocol
	usart_get_config_defaults(&irda_conf);
	
	// Port Configuration
	irda_conf.transfer_mode = USART_TRANSFER_ASYNCHRONOUSLY;	// Asynchronous Communication Mode
	irda_conf.generator_source = GCLK_GENERATOR_0;				// Use the Generic Clock 0 as source
	irda_conf.baudrate = 115200;								// IrDA Baudrate
	irda_conf.character_size = USART_CHARACTER_SIZE_8BIT;
	irda_conf.stopbits = USART_STOPBITS_1;
	irda_conf.parity = USART_PARITY_EVEN;
	irda_conf.encoding_format_enable = true;	// Enable IrDA Encoding
	
	// Pin Multiplexer Settings
	irda_conf.mux_setting = USART_RX_1_TX_0_XCK_1;
	irda_conf.pinmux_pad0 = PINMUX_PA22C_SERCOM3_PAD0;
	irda_conf.pinmux_pad1 = PINMUX_PA23C_SERCOM3_PAD1;
	irda_conf.pinmux_pad2 = PINMUX_UNUSED;
	irda_conf.pinmux_pad3 = PINMUX_UNUSED;

	// Initialize the previous settings
	usart_init((struct usart_module*) &irda_master, SERCOM3, &irda_conf);

	// Enable the module
	usart_enable((struct usart_module*) &irda_master);

	// ******** Callback setup
	usart_register_callback((struct usart_module*) &irda_master, (usart_callback_t)irda_master_callback_received, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback((struct usart_module*) &irda_master, USART_CALLBACK_BUFFER_RECEIVED);

	usart_register_callback((struct usart_module*) &irda_master, (usart_callback_t)irda_master_callback_transmitted, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback((struct usart_module*) &irda_master, USART_CALLBACK_BUFFER_TRANSMITTED);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
////////////////////// IrDA CALLBACK FUNCTIONS ////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// IrDA Rx Callback Function
BaseType_t lock_allow_main_discovery = pdTRUE;
TickType_t lock_main_discovery_count;
BaseType_t lock_trace = pdFALSE;
TickType_t unlock_count;
BaseType_t irda_timed_out = pdFALSE;
static void irda_master_callback_received(const struct usart_module *const module) {
	
	BaseType_t xYieldRequired; 
	
	if ( !irda_timed_out ) {
		usart_disable_transceiver(&irda_master, USART_TRANSCEIVER_RX);
	
		switch ( irda_comm_state )
		{
			case IRDA_SLAT_PING:
						// Check if main discovery is allowed
				if ( lock_allow_main_discovery == pdFALSE ) {
					unlock_count = xTaskGetTickCountFromISR();
				
					if ( unlock_count >= lock_main_discovery_count ) {
						lock_allow_main_discovery = pdTRUE;
					}
				}
	
				if ( irda_rx_array[0] == irda_rx_array[1] && irda_rx_array[1] == irda_rx_array[2] && 
						irda_rx_array[0] == 0xAA )
				{	
					if ( lock_allow_main_discovery ) {
						if ( lock_trace == pdFALSE ) {
							lock_trace = pdTRUE;
					
							// Start the trace
							//uiTraceStart();
						}
						
						// r010716-1608: The LED action was disabled so that we can test the revision
						//port_pin_set_output_level(LED_BUSY, pdTRUE);
 						//port_pin_set_output_level(LED_ERROR, pdFALSE);

 						//vTracePrintF(event_channel, "Rxd Header!"); 					

							//port_pin_toggle_output_level(LED_BUSY);
 						irda_comm_state = IRDA_SLAT_FIRST;	// Change state to send first response

 						// The board has been discovered, note so that 0xAA is ignored next pass
 						//lock_allow_main_discovery = pdFALSE;
 						//lock_main_discovery_count = xTaskGetTickCountFromISR();
 						//lock_main_discovery_count += 16;	// Wait half-a-second
						xYieldRequired = xTaskResumeFromISR( irda_task_handler );
					
						if( xYieldRequired == pdTRUE )
						{
							// We should switch context so the ISR returns to a different task.
							// NOTE:  How this is done depends on the port you are using.  Check
							// the documentation and examples for your port.
							//vTracePrintF(event_channel, "Yield ISR!");
							portYIELD_FROM_ISR(xYieldRequired);
						} else {
							irda_comm_state = IRDA_SLAT_PING;
							//vTracePrintF(event_channel, "No Yield! Changed to PING");
						}
					}
				} else {
					irda_comm_state = IRDA_SLAT_PING;
					//vTracePrintF(event_channel, "Wrong Data!");
					//port_pin_set_output_level(LED_ERROR, pdTRUE);
					//irda_comm_state = IRDA_SLAT_PING;
				}
			break;
			case IRDA_SLAT_FIRST_RESPONSE:	// r010716-1608: This the code to execute when we get 0xCC. Stage 6
						// If this is correct, this board can be safely assumed to be synchronized
				if ( crc_check(&irda_rx_array, 4) )
				{
					

					//vTracePrintF(event_channel, "Rxd Header!");

					//port_pin_toggle_output_level(LED_BUSY);
					irda_comm_state = IRDA_SLAT_STAGE_7A;	// Change state to send first response
						// The slat card has been synched at this point.
					port_pin_set_output_level(LED_BUSY, pdTRUE);
					
					// Resetting the timer
					xTimerResetFromISR ( timer_IrDA_Ping, 0 );
					
					xYieldRequired = xTaskResumeFromISR( irda_task_handler );
					
					if( xYieldRequired == pdTRUE )
					{
						// We should switch context so the ISR returns to a different task.
						// NOTE:  How this is done depends on the port you are using.  Check
						// the documentation and examples for your port.
						//vTracePrintF(event_channel, "Yield ISR!");
						portYIELD_FROM_ISR(xYieldRequired);
					} else {
						//vTracePrintF(event_channel, "No Yield! Changed to PING");
					}
				} else {
					//vTracePrintF(event_channel, "Wrong Data!");
					//port_pin_set_output_level(LED_ERROR, pdTRUE);
					irda_comm_state = IRDA_SLAT_PING;
				}
			break;
			case IRDA_SLAT_STAGE_7B:	// r010716-1608: This the code to execute when we get 0xCC. Stage 6
				// If this is correct, this board can be safely assumed to be synchronized
				if ( crc_check(&irda_rx_array, 4) )
				{
					irda_comm_state = IRDA_SLAT_PING;	// Change state to send first response
					// The slat card has been synched at this point.
					//port_pin_set_output_level(LED_BUSY, pdTRUE);
				
					// Resetting the timer
					xTimerResetFromISR ( timer_IrDA_Ping, 0 );
				} else {
					//vTracePrintF(event_channel, "Wrong Data!");
					//port_pin_set_output_level(LED_ERROR, pdTRUE);
					irda_comm_state = IRDA_SLAT_PING;
				}
			break;
		}	
	}
}
// IrDA Tx Callback Function
static void irda_master_callback_transmitted(const struct usart_module *const module) {
	BaseType_t xYieldRequired; 
	
	// Update post r010716-1608
		// ** In this update, the machine state is changed so that we can continue receiving data
		// Change::: IRDA_SLAT_PING to IRDA_SLAT_FIRST_RESPONSE
		// ** In this update, we also queue the Rx job for 0xCC
	
	switch ( irda_comm_state ) {
		case IRDA_SLAT_FIRST:	// This is the case where the first Response has been sent
			// r010716-1608: Change the machine state accordingly
			irda_comm_state = IRDA_SLAT_FIRST_RESPONSE;	// Go back to the First Response mode
			
			
			
				// Make sure to reset the timer
			xTimerResetFromISR ( timer_IrDA_Ping, 0 );
			
			
			// r010716-1608: Request the 0xCC data reception right away
			usart_enable_transceiver( &irda_master, USART_TRANSCEIVER_RX );	// Enable Receiving Transceiver
			
			
			usart_read_buffer_job( &irda_master, irda_rx_array, 5 );	// Try to get the 3-Byte ping
			
			//vTracePrintF(event_channel, "Resp Sent.");
			 
			//xYieldRequired = xTaskResumeFromISR( irda_task_handler );
			//if( xYieldRequired == pdTRUE )
			//{
				// We should switch context so the ISR returns to a different task.
				// NOTE:  How this is done depends on the port you are using.  Check
				// the documentation and examples for your port.
			//	portYIELD_FROM_ISR(xYieldRequired);
			//}
		break;
		case IRDA_SLAT_STAGE_7A:
			// Post: r010716-1818: Change the machine state accordingly
			irda_comm_state = IRDA_SLAT_STAGE_7B;	// Go back to the First Response mode
			
			
			
			// Make sure to reset the timer
			xTimerResetFromISR ( timer_IrDA_Ping, 0 );
			
			
			// Post: r010716-1818: Request the 0xCC data reception right away
			usart_enable_transceiver( &irda_master, USART_TRANSCEIVER_RX );	// Enable Receiving Transceiver
			usart_read_buffer_job( &irda_master, irda_rx_array, 5 );	// Try to get the 3-Byte ping
		break;
	}
}
