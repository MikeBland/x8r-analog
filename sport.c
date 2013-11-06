/* ============================================================
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ============================================================*/

// Author: Mike Blandford

// ATTINY version of X8R Sport analog input
// Built in resistors are 15K and 3K3 giving max. voltage of 18.3v (4S)

// Changes:
// For Attiny13, add pullup on pin 5 (PB0), scale for 3S if pin 5 pulled low
// Change timing method, use TIMER0 for hardware timing

#include <inttypes.h>
#include <avr/io.h>
#include <avr/wdt.h>
//#include <avr/interrupt.h>

// ATTINY44, INT0 is on PB2, ADC1 is on PA1
// ATTINY85, INT0 is on PB2, ADC3 is on PB3

// ATTINY13 Fuse Settings:
//HIGH 0xFD
//LOW 0x1A

//No self program
//No DWEN
//Brownout at 1.8V
//No Reset disable
//SPIEN on
//EESAVE on
//Watchdog enabled
//No CLKDIV8
//int RC @ 9.6MHz, 14CK and 64mS



#define SENSOR_ID		0x1B
#define A2_ID       0xF103

#define PINCHANGE		1

#define DEBUG		1

#define DEBUG_BIT		0x10
#define DEBUG_PORT	PORTB
#define DEBUG_DDR		DDRB
#define DEBUG_PIN		PINB

#define LINK_BIT		0x01
#define LINK_PORT		PORTB
#define LINK_DDR		DDRB
#define LINK_PIN		PINB

//#define CLK_BIT			0x08
//#define CLK_PORT		PORTB
//#define CLK_DDR			DDRB
//#define CLK_PIN			PINB

// SPORT_BIT is the one with INT0 available
// Unless PINCHANGE is defined
#define SPORT_BIT		0x02
#define SPORT_PORT	PORTB
#define SPORT_DDR		DDRB
#define SPORT_PIN		PINB

// These for 9.6MHz
//#if F_CPU == 9600000
//#define RXCENTRE		23
//#define RXINTRA			48 //51
//#define RXSTOP			48 //51
//#define TXDELAY			50
//#define XMIT_START_ADJUSTMENT 3
//#define OSCCALHIGH	149
//#define OSCCALLOW		101
//#define OSCCALCENTRE	125
//#define IDLEDELAYTIME	250
//#else
//	#if F_CPU == 8000000		// Untested
//	#define RXCENTRE		19
//	#define RXINTRA			43
//	#define RXSTOP			43
//	#define TXDELAY			41
//	#define XMIT_START_ADJUSTMENT 3
//	#define OSCCALHIGH	42
//	#define OSCCALLOW		29
//	#define OSCCALCENTRE	35
//	#define IDLEDELAYTIME	208
//	#endif
//#endif

#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))

struct t_anacontrol
{
	uint8_t Analog ;
	uint16_t AnaAve ;
	uint8_t AnaCount ;
} AnalogControl ;

uint8_t BitRate ;
uint8_t RxCentre ;
//uint8_t Osccal2 ;

uint8_t Crc ;

void waitCompA( void ) ;
static uint8_t recv( void ) ;
static uint8_t rx_pin_read( void ) ;
inline void setTX( void ) ;
inline void setRX( void ) ;
static void initSerial( void ) ;
int swrite(uint8_t b) ;
static void sendCrc( void ) ;
static void sendData( void ) ;
static void sendValue( uint8_t value ) ;
static void readSensors( void ) ;
void wait4msIdle( void ) ;
//void clockOut( void ) ;
void main( void ) ;


static uint8_t rx_pin_read()
{
	return SPORT_PIN & SPORT_BIT ;
}


//void tunedDelay( uint8_t delay)
//{ 
//	do
//	{
//		asm("") ;
//	} while ( --delay ) ;
//}

void waitCompA()
{
	while ( ( TIFR0 & (1 << OCF0A) ) == 0 )
	{
		// null body
	}
	TIFR0 = (1 << OCF0A) ; 		// Clear flag
}


//
// The receive routine called by the interrupt handler
//
static uint8_t recv()
{
  uint8_t d = 0 ;
  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
#if DEBUG
	DEBUG_PORT |= DEBUG_BIT ;
#endif
#if PINCHANGE==0
  if (rx_pin_read())
  {
#endif
		
    // Wait approximately 1/2 of a bit width to "centre" the sample
		waitCompA() ;
#if DEBUG
	DEBUG_PORT &= ~DEBUG_BIT ;
	DEBUG_PORT |= DEBUG_BIT ;
#endif
    // Read each of the 8 bits
    for ( uint8_t i = 0 ; i < 8 ; i += 1 )
    {
      d >>= 1 ;
			OCR0A += BitRate ;
			waitCompA() ;
#if DEBUG
	DEBUG_PORT &= ~DEBUG_BIT ;
#endif
      if (!rx_pin_read())
			{
        d |= 0x80 ;
			}
#if DEBUG
	DEBUG_PORT |= DEBUG_BIT ;
#endif
    }
    // skip the stop bit
		OCR0A += BitRate ;
		waitCompA() ;
#if PINCHANGE==0
  }
#endif
#if DEBUG
	DEBUG_PORT &= ~DEBUG_BIT ;
#endif
	return d ;
}

inline void setTX()
{
	SPORT_DDR |= SPORT_BIT ;
	SPORT_PORT &= ~SPORT_BIT ;		// low as inverse logic
}

inline void setRX()
{
	SPORT_DDR &= ~SPORT_BIT ;
	SPORT_PORT &= ~SPORT_BIT ;		// low so no pullup
}

int swrite(uint8_t b)
{
	// Write the start bit
	OCR0A = TCNT0 + BitRate ;
	TIFR0 = (1 << OCF0A) ; 		// Clear flag
	SPORT_PORT |= SPORT_BIT ;		// high for start bit as inverse logic
	waitCompA() ;
  // Write each of the 8 bits
    for ( uint8_t i = 8 ; i ; i -= 1 )
    {
      if (b & 1) // test bit
				SPORT_PORT &= ~SPORT_BIT ;		// send 1
      else
				SPORT_PORT |= SPORT_BIT ;			// send 0
			OCR0A += BitRate ;
			waitCompA() ;
			b >>= 1 ;
    }
		SPORT_PORT &= ~SPORT_BIT ;		// restore pin to natural state

	OCR0A += BitRate ;
	waitCompA() ;
  
  return 1 ;
}

static void initSerial()
{
  setRX() ;
}

void sendByte( uint8_t byte )
{
	uint16_t lcrc = Crc ;
	// CRC update
  lcrc += byte; //0-1FF
  lcrc += lcrc >> 8; //0-100
  Crc = lcrc ;

	if ( ( byte == 0x7E ) || ( byte == 0x7D ) )
	{
  	swrite( 0x7D ) ;
		
		byte &= ~0x20 ;
	}
  swrite(byte) ;
}


static void sendCrc()
{
  sendByte(0xFF-Crc) ;
}

static void sendValue( uint8_t value )
{
	setTX() ;
  Crc = 0;
  sendByte(0x10); // DATA_FRAME
  sendByte( (uint8_t)A2_ID );
  sendByte( A2_ID >> 8 );
  sendByte(value);
  sendByte(0);
  sendByte(0);
  sendByte(0);
  sendCrc();
	setRX() ;
}


static void sendData()
{
	TCCR0B = 1 ;		// Clock div 1
  sendValue( AnalogControl.Analog ) ;
}

static void readSensors()
{
	struct t_anacontrol *panalog ;

	panalog = &AnalogControl ;
	FORCE_INDIRECT( panalog ) ;

  // set the reference to Vcc and the measurement to ADC1
  ADMUX = _BV(MUX0) ;

	ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint16_t val = ADC ; // read the value from the sensor

	if ( LINK_PIN & LINK_BIT )
	{
		val += 1 ;
		val >>= 2 ;	
	}
	
	panalog->AnaAve += val ;
	panalog->AnaCount += 1 ;
	
	if ( panalog->AnaCount > 3 )
	{
		if ( LINK_PIN & LINK_BIT )
		{
			val = panalog->AnaAve >> 2 ;
		}
		else
		{
			// Scale 15K/3.3K to 10K/3.3K
			// Map 744 counts to 256
			// We have 4 samples added so 2976 goes to 256
			// Use 11/128, is about 0.2% out, but MUCH better than the resistor tolerance
//			val = panalog->AnaAve * 11 ;
			// Shifts and adds are faster and shorter than a multiply
			val = panalog->AnaAve << 2 ;	// *4
			val += panalog->AnaAve ;			// *5
			val <<= 1 ;										// *10
			val += panalog->AnaAve ;			// *11
			val >>= 7 ;		// divide by 128
		}	
		if ( val > 255 )
		{
			val = 255 ;		
		}
		panalog->Analog = val ;
		panalog->AnaCount = 0 ;
		panalog->AnaAve = 0 ;
	}
}

void wait4msIdle()
{
	// We want 4mS with NO transitions on INT0
	// and INT0 (PB1) is LOW ( = idle high for inverted data)
	TCNT0 = 0 ;
	TCCR0B = 4 ;		// Clock div 256, 26.7 uS per count
	// 150 counts is 4mS
#if PINCHANGE
	GIFR = (1 << PCIF) ;		// CLEAR flag
#else
	GIFR = (1 << INTF0) ;		// CLEAR flag
#endif
	do
	{
#if PINCHANGE
		if ( ( GIFR & (1 << PCIF) ) || ( SPORT_PIN & SPORT_BIT) )
#else
		if ( ( GIFR & (1 << INTF0) ) || ( SPORT_PIN & SPORT_BIT) )
#endif
		{
#if PINCHANGE
			GIFR = (1 << PCIF) ;		// CLEAR flag
#else
			GIFR = (1 << INTF0) ;
#endif
			TCNT0 = 0 ;			
		}
		wdt_reset() ;
	} while ( TCNT0 < 151 ) ;
	TCCR0B = 0 ;		// Stop timer
}

//void clockOut()
//{
//	// CLOCK DEBUG
//	if ( ( CLK_PIN & CLK_BIT) == 0 )		// Pin pulled low
//	{
//		uint16_t i ;
//		CLK_DDR |= CLK_BIT ;		// Output
//		TCCR0B = 2 ;		// Clock div 8, 1.2 uS per count
//		for( i = 0 ; i < 16384 ; )
//		{
//			if ( TIFR0 & (1 << TOV0) )
//			{
//				TIFR0 = (1 << TOV0) ; 		// Clear flag
//				CLK_PORT ^= CLK_BIT ;				
//				i += 1 ;
//			}
//			wdt_reset() ;
//		}	
//		CLK_DDR &= ~CLK_BIT ;		// Input
//		CLK_PORT |= CLK_BIT ;		// with pull up
//	}
//}


void main()
{
  static uint8_t lastRx = 0 ;
	uint8_t rx ;

	wdt_reset() ;
	/* Start timed sequence */
	WDTCR |= (1<<WDCE) | (1<<WDE) ;
	/* Set new prescaler(time-out) value = 16K cycles (~0.125 s) */
	WDTCR = (1<<WDE) | (1<<WDP1) | (1<<WDP0) ;
	wdt_reset() ;

  initSerial() ;
	DIDR0 = 4 ;
	ADCSRA = 0x86 ;
#if PINCHANGE
	PCMSK |= SPORT_BIT ;
#else
	MCUCR |= 3 ;		// INT0 interrupt on rising edge
#endif

#if DEBUG
	DEBUG_DDR |= DEBUG_BIT ;
	DEBUG_PORT &= ~DEBUG_BIT ;
#endif

	LINK_DDR &= ~LINK_BIT ;		// Input
	LINK_PORT |= LINK_BIT ;		// with pull up

//	CLK_DDR &= ~CLK_BIT ;		// Input
//	CLK_PORT |= CLK_BIT ;		// with pull up
  
	wdt_reset() ;
	// Get the first value
	readSensors() ;
  readSensors() ;
  readSensors() ;
  readSensors() ;

	wdt_reset() ;

//	clockOut() ;

	wait4msIdle() ;
	// Had 4mS idle, now we expect a 0x7E
	// So we can time the 6 one bits in the middle

	TCCR0B = 2 ;		// Clock div 8, 0.833 uS per count
#if PINCHANGE
	GIFR = (1 << PCIF) ;	// CLEAR flag
#else
	GIFR = (1 << INTF0) ;		// CLEAR flag
#endif
	for(;;)
	{
#if PINCHANGE
		if ( GIFR & (1 << PCIF) )
#else
		if ( GIFR & (1 << INTF0) )
#endif
		{
			rx = TCNT0 ;		// Note timer value
#if PINCHANGE
			GIFR = (1 << PCIF) ;		// CLEAR flag
#else
			GIFR = (1 << INTF0) ;
#endif
			break ;
		}
		wdt_reset() ;
	}
	// Got the start of the start bit

	for(;;)
	{
#if PINCHANGE
		if ( GIFR & (1 << PCIF) )
#else
		if ( GIFR & (1 << INTF0) )
#endif
		{
#if PINCHANGE
			GIFR = (1 << PCIF) ;		// CLEAR flag
#else
			GIFR = (1 << INTF0) ;
#endif
			break ;
		}
		wdt_reset() ;
	}

#if PINCHANGE==0
	MCUCR &= ~1 ;		// INT0 interrupt on falling edge
#endif
	for(;;)
	{
#if PINCHANGE
		if ( GIFR & (1 << PCIF) )
#else
		if ( GIFR & (1 << INTF0) )
#endif
		{
			rx = TCNT0 - rx ;
#if PINCHANGE
			GIFR = (1 << PCIF) ;		// CLEAR flag
#else
			GIFR = (1 << INTF0) ;
#endif
			break ;
		}
		wdt_reset() ;
	}
	// rx now the time (in 0.8333uS units) of the 8 bits

	BitRate = rx ;
	RxCentre = rx/2 - 24 ;
//	Osccal1 = OSCCAL ;
//	if ( rx < OSCCALHIGH )
//	{
//		if ( rx > OSCCALLOW )
//		{
//			int16_t value ;
//			value = OSCCALCENTRE - rx ;		// +/- 5

//			// check here for OSCCAL not wrapping
//			if ( value )
//			{
//				value += OSCCAL ;
//				if ( value < 0 )
//				{
//					value = 0 ;					
//				}
//				if ( value > 127 )
//				{
//					value = 127 ;					
//				}
//				OSCCAL = value ;
//			}
//		}
//	}
//	Osccal2 = OSCCAL ;

#if PINCHANGE==0
	MCUCR |= 3 ;		// INT0 interrupt on rising edge
#endif

	wait4msIdle() ;
	// Had 4mS idle, now we expect a 0x7E

#if PINCHANGE
	GIFR = (1 << PCIF) ;		// CLEAR flag
#else
	GIFR = (1 << INTF0) ;
#endif
	for( ;; )
	{
#if PINCHANGE
		if ( GIFR & (1 << PCIF) )
#else
		if ( GIFR & (1 << INTF0) )
#endif
		{
			TCCR0B = 0 ;		// stop timer
			TCNT0 = 0 ;
			TCCR0B = 1 ;		// Clock div 1
			OCR0A = RxCentre ;
			TIFR0 = (1 << OCF0A) ; 		// Clear flag
#if PINCHANGE
			GIFR = (1 << PCIF) ;		// CLEAR flag
			if ( SPORT_PIN & SPORT_BIT)
			{
#else
			GIFR = (1 << INTF0) ;
#endif
				rx = recv() ;
		    if (lastRx == 0x7e && rx == SENSOR_ID)
				{
    		  lastRx = 0 ;
					// Delay around 400uS
					TCCR0B = 0 ;		// stop timer
					TIFR0 = (1 << OCF0A) ; 		// Clear flag
					OCR0A = TCNT0 + 60 ;
					TCCR0B = 3 ;		// Clock div 64
					waitCompA() ;
  		  	sendData() ;
  		  	readSensors() ;
				}
				else
				{
					lastRx = rx ;
				}
#if PINCHANGE
			}
			GIFR = (1 << PCIF) ;		// CLEAR flag
#else
			GIFR = (1 << INTF0) ;
#endif
		}
		wdt_reset() ;
//		clockOut() ;

  }
}	

