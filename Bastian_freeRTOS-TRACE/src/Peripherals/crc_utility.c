#include "asf.h"
#include "bastian_sercom.h"

// This is the function that will check the validity of the crc byte as received
BaseType_t crc_check( uint8_t* data, uint8_t size ) {
	uint8_t i;
	uint8_t crc_result = 0;
	
	// Add all the contents of the packet
	for( i=0; i<size; i++) crc_result += *( data + i );
	crc_result |= 0x55;	// OR the result and get our crc
	
	// Return result as boolean
	// pdTRUE  -> CRC Checks Out
	// pdFALSE -> CRC does NOT check out
	//*( data + 5 ) = crc_result;
	if ( crc_result == *(data + size) ) return pdTRUE;
	else return pdFALSE;
}

// This is the function that generates the CRC byte
// The size does not include the spot for the CRC byte
void crc_generate( uint8_t* data, uint8_t size ) {
	uint8_t i;
	*( data + size ) = 0;
	
	// Add all the contents of the packet
	for( i=0; i<size; i++) *( data + size ) += *( data + i );
	
	*( data + size ) |= 0x55;	// OR the CRC byte
}
