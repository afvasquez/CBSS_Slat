/*
 * bastian_sercom.h
 *
 * Created: 12/1/2015 11:05:24 AM
 *  Author: avasquez
 */ 


#ifndef BASTIAN_SERCOM_H_
#define BASTIAN_SERCOM_H_

#define LED_BUSY	PIN_PA27
#define LED_ERROR	PIN_PA25

#define IRDA_SLAT_PING				(( uint8_t ) 0x01 )
#define IRDA_SLAT_PRE_PING			(( uint8_t ) 0x11 )
#define IRDA_SLAT_FIRST				(( uint8_t ) 0x02 )
#define IRDA_SLAT_FIRST_RESPONSE	(( uint8_t ) 0x03 )
#define IRDA_SLAT_RESET				(( uint8_t ) 0x0E )


////////////////////  BASTIAN SERCOM  ////////////////////////////////////
//		Perform complete SERCOM module setup for the application
//		Takes no arguments
void bastian_IrDA_configuration(void);

////////////////////  BASTIAN SERCOM  ////////////////////////////////////
//		MODULE: Slave Module -> spi_slave
//		module handler for the SPI interaction with GREEN
volatile struct usart_module ser_master;
//		MODULE: IrDA Module -> irda_master
//		module handler for the IR interaction with RED-SLAT
volatile struct usart_module irda_master;

extern TaskHandle_t irda_task_handler;
extern BaseType_t lock_allow_main_discovery;
extern uint8_t irda_comm_state;

extern uint8_t irda_tx_array[6];
extern uint8_t irda_rx_array[6];

extern traceLabel event_channel;

extern BaseType_t irda_timed_out;

// Define the handler for the timer
extern TimerHandle_t timer_IrDA_Ping;
extern TimerHandle_t timer_IrDA_Sync;

#endif /* BASTIAN_SERCOM_H_ */ 