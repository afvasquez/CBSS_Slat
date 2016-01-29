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

#define IRDA_SLAT_PING		(( uint8_t ) 0x01 )
#define IRDA_SLAT_PRE_PING		(( uint8_t ) 0x11 )
#define IRDA_SLAT_FIRST		(( uint8_t ) 0x02 )
#define IRDA_SLAT_FIRST_RESPONSE		(( uint8_t ) 0x03 )

#define IRDA_SLAT_STAGE_7A	(( uint8_t ) 0x05 )
#define IRDA_SLAT_STAGE_7B	(( uint8_t ) 0x06 )

#define IRDA_SLAT_RESET		(( uint8_t ) 0x0E )
	
	// Half a second wait from observed ping to the next
#define IRDA_PING_PERIOD	(( TickType_t ) 500 )	

// Address Pin Definitons
#define ADDR_BIT_0 PIN_PA28
#define ADDR_BIT_1 PIN_PA07
#define ADDR_BIT_2 PIN_PA06
#define ADDR_BIT_3 PIN_PA05
#define ADDR_BIT_4 PIN_PA04
#define ADDR_BIT_5 PIN_PA03
#define ADDR_BIT_6 PIN_PA02
#define ADDR_BIT_7 PIN_PA15
#define ADDR_BIT_8 PIN_PA14

//////////////////////////////////////////////////////////////////////////
///////////////////////////  BASTIAN IrDA  ///////////////////////////////
struct slat_systems {
	// IrDA Communication Timeout Counter
	//uint32_t system_clock;
	
	// Slat Address
	uint8_t system_address;
	
	// Job status and utilities
	// Current Job Number
	//uint8_t	job_number;
	//uint8_t next_job_number;
	// Job description
	//uint8_t job_description;
	// Is Job done (needs to be done?
	//bool is_job_done;
	
	// Is job to be done
	//bool is_job_to_start;
	
	// Debug int
	//uint8_t debug_integer;
};

extern volatile struct slat_systems slat;

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

//extern traceLabel event_channel;

extern BaseType_t irda_timed_out;

// Define the handler for the timer
extern TimerHandle_t timer_IrDA_Ping;

//////////////////////////////////////////////////////////////////////////
///////////////////////////  BASTIAN IrDA  ///////////////////////////////
struct ebm_papst_motor {
	// Motor Delay Variable
// 	uint32_t motor_comm_delay;
// 	uint32_t motor_job_delay;
// 	uint32_t motor_timeout;
	
	// Motor Communication State
	uint8_t motor_state;
	uint8_t motor_state_callback;
	
	// Motor Health
	uint8_t health;
	
	// Is the motor synced to card
 	bool motor_is_synced;
	 	
// 	// Is the motor currently on health check
// 	bool motor_is_on_health_check;
// 	// Is the motor currently on a job
// 	bool motor_is_doing_a_job;
// 	// Is the error to be reported a hard fault?
// 	bool is_hard_fault;
 	// Perform Motor Actions
 	bool job_is_incoming;
 	
 	// Motor Communication Buffers
 	uint8_t rx_buffer[20];
 	uint8_t tx_buffer[20];
	 
	uint8_t rx_data_A[4];
	uint8_t rx_data_B[4];
		
	uint16_t rx_ramp_value;
	uint16_t rx_duration_value;
	int16_t rx_speed_value;
	uint16_t rx_delay_value;
// 	uint8_t rx_data_length;
// 	uint8_t tx_data_length;

	// Motor Run Variables
		// Total run duration
	bool is_motor_ready;			// Is the motor ready to run?
	bool is_motor_queued;			// Is there an action ready to be processed?
	bool is_motor_running;			// The motor is running and will not respond to any beacon pings at this point
	bool is_motor_error;			// Is there an outstanding error in the motor
	uint8_t motor_job_number;		// This is the ID for the job that is being done
	TickType_t run_delay_duration;	// Total time to delay before the motor runs again
	TickType_t run_total_duration;	// Total duration of the run => t_r + t_p + t_r
	TickType_t run_nzdata_duration;	// Total duration of the run with regards to actual data being sent;
	TickType_t motor_dead_time;		// This is the amount of time that the motor will be idle
	uint16_t motor_speed_rpm;		// Speed command for the motor
	uint16_t motor_ramp_ms;			// Ramp in ms-per-1krpm to be sent to motor
	uint8_t ramp_high_nibble;		// These are the utilities to send the correct data
	uint8_t ramp_low_nibble;
	uint8_t speed_high_nibble;		// These are the utilities to send the correct data
	uint8_t speed_low_nibble;
	uint8_t allocated_power;
	uint16_t allocated_current;
};

extern volatile struct ebm_papst_motor motor;

//////////////////////////////////////////////////////////////////////////
//	############# Necessary Motor Library
//////////////////////////////////////////////////////////////////////////
// Motor Interface definition
#define MOTOR_TRANSCEIVER_ENABLE_PIN	PIN_PA10
#define MOTOR_A_SIGNAL					PIN_PA11

//////////////////////////////////////////////////////////////////////////
void ebp_papst_motor_setup( void );
static void motor_callback_received(const struct usart_module *const module);
static void motor_callback_transmitted(const struct usart_module *const module);
void ebp_papst_motor_task_setup( void );

////////////////////  BASTIAN SERCOM  ////////////////////////////////////
//		MODULE: Motor Module -> motor_serial
//			module handler for the Serial interaction with Motor
extern volatile struct usart_module motor_serial;
//		External reference to the task
extern TaskHandle_t motor_task_handler;

////////////////////////// CRC Utilities /////////////////////////////////
//	This function checks for the validity of the CRC byte on a receiving
//		byte array.

BaseType_t crc_check( uint8_t* data, uint8_t size);
void crc_generate( uint8_t* data, uint8_t size );

#endif /* BASTIAN_SERCOM_H_ */ 