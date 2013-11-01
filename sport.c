// ATTINY version of X8R Sport analog input

#include <inttypes.h>
#include <avr/io.h>
//#include <avr/interrupt.h>

// ATTINY44, INT0 is on PB2, ADC1 is on PA1
// ATTINY85, INT0 is on PB2, ADC3 is on PB3

#define SENSOR_ID		0x1B
#define A2_ID       0xF103

#define PINCHANGE		1

#define DEBUG		1

#define DEBUG_BIT		0x10
#define DEBUG_PORT	PORTB
#define DEBUG_DDR		DDRB
#define DEBUG_PIN		PINB


// SPORT_BIT is the one with INT0 available
#define SPORT_BIT		0x02
#define SPORT_PORT	PORTB
#define SPORT_DDR		DDRB
#define SPORT_PIN		PINB

// These for 9.6MHz
#if F_CPU == 9600000
#define RXCENTRE		23
#define RXINTRA			52
#define RXSTOP			52
#define TXDELAY			50
#define XMIT_START_ADJUSTMENT 3
#define OSCCALHIGH	35
#define OSCCALLOW		24
#define OSCCALCENTRE	29
#define IDLEDELAYTIME	250
#else
	#if F_CPU == 8000000
	#define RXCENTRE		19
	#define RXINTRA			43
	#define RXSTOP			43
	#define TXDELAY			41
	#define XMIT_START_ADJUSTMENT 3
	#define OSCCALHIGH	29
	#define OSCCALLOW		20
	#define OSCCALCENTRE	24
	#define IDLEDELAYTIME	208
	#endif
#endif

#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))

struct t_anacontrol
{
	uint8_t Analog ;
	uint16_t AnaAve ;
	uint8_t AnaCount ;
} AnalogControl ;


uint8_t Crc ;

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
void main( void ) ;


static uint8_t rx_pin_read()
{
	return PINB & SPORT_BIT ;
}


void tunedDelay( uint8_t delay)
{ 
	do
	{
		asm("") ;
	} while ( --delay ) ;
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
  if (rx_pin_read())
  {
    // Wait approximately 1/2 of a bit width to "centre" the sample
    tunedDelay(RXCENTRE) ;
#if DEBUG
	DEBUG_PORT &= ~DEBUG_BIT ;
	DEBUG_PORT |= DEBUG_BIT ;
#endif
    // Read each of the 8 bits
    for ( uint8_t i = 0 ; i < 8 ; i += 1 )
    {
      tunedDelay(RXINTRA);
#if DEBUG
	DEBUG_PORT &= ~DEBUG_BIT ;
	DEBUG_PORT |= DEBUG_BIT ;
#endif
      d >>= 1 ;
      if (rx_pin_read())
        d &= ~0x80 ;
      else // else clause added to ensure function timing is ~balanced
        d |= 0x80 ;
    }

    // skip the stop bit
    tunedDelay(RXSTOP);
  }
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
	SPORT_PORT |= SPORT_BIT ;		// high for start bit as inverse logic
  tunedDelay(TXDELAY + XMIT_START_ADJUSTMENT);

  // Write each of the 8 bits
    for ( uint8_t i = 8 ; i ; i -= 1 )
    {
      if (b & 1) // test bit
				SPORT_PORT &= ~SPORT_BIT ;		// send 1
      else
				SPORT_PORT |= SPORT_BIT ;			// send 0
    
      tunedDelay(TXDELAY);
			b >>= 1 ;
    }
		SPORT_PORT &= ~SPORT_BIT ;		// restore pin to natural state

  tunedDelay(TXDELAY) ;
  
  return 1;
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

	val += 1 ;
	val >>= 2 ;	
	panalog->AnaAve += val ;
	panalog->AnaCount += 1 ;
	if ( panalog->AnaCount > 3 )
	{
		val = panalog->AnaAve >> 2 ;
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
	// We want 4mS with NO transisitions on INT0
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
	} while ( TCNT0 < 151 ) ;
	TCCR0B = 0 ;		// Stop timer
}

void main()
{
  static uint8_t lastRx = 0 ;
	uint8_t rx ;

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
  
	// Get the first value
	readSensors() ;
  readSensors() ;
  readSensors() ;
  readSensors() ;

	wait4msIdle() ;
	// Had 4mS idle, now we expect a 0x7E
	// So we can time a start bit and the first logic 0 bit (2 bits)

	TCCR0B = 2 ;		// Clock div 8, 1.2 uS per count
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
			rx = TCNT0 ;
#if PINCHANGE
			GIFR = (1 << PCIF) ;		// CLEAR flag
#else
			GIFR = (1 << INTF0) ;
#endif
			break ;
		}
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
			rx = TCNT0 = rx ;
#if PINCHANGE
			GIFR = (1 << PCIF) ;		// CLEAR flag
#else
			GIFR = (1 << INTF0) ;
#endif
			break ;
		}
	}

	if ( rx < OSCCALHIGH )
	{
		if ( rx > OSCCALLOW )
		{
			int8_t value ;
			value = rx - OSCCALCENTRE ;
			if ( value )
			{
				OSCCAL += value ;
			}
		}
	}

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
					for ( uint8_t i = 5 ; i ; i -= 1 )
					{
    				tunedDelay( IDLEDELAYTIME ) ;
					}
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
  }
}	

