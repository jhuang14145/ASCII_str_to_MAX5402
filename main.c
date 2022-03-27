/*
* ASCII_str_to_MAX5402.c
*
* Created: 3/24/2022 7:05:42 PM
* Author : jhuan
*/

#define F_CPU 4000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define USART_RX_BUFFER_SIZE 128     /* 2,4,8,16,32,64,128 or 256 bytes */
#define USART_TX_BUFFER_SIZE 128     /* 2,4,8,16,32,64,128 or 256 bytes */
#define USART_RX_BUFFER_MASK ( USART_RX_BUFFER_SIZE - 1 )
#define USART_TX_BUFFER_MASK ( USART_TX_BUFFER_SIZE - 1 )
#if ( USART_RX_BUFFER_SIZE & USART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif
#if ( USART_TX_BUFFER_SIZE & USART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif

/* Static Variables */
static unsigned char USART_RxBuf[USART_RX_BUFFER_SIZE];
static volatile unsigned char USART_RxHead;
static volatile unsigned char USART_RxTail;
//static unsigned char USART_TxBuf[USART_TX_BUFFER_SIZE];
static volatile unsigned char USART_TxHead;
static volatile unsigned char USART_TxTail;

/* Prototypes */
void USART0_Init( unsigned int baudrate );
unsigned char USART0_Receive( void );
void USART0_Transmit( unsigned char data );
int DataInReceiveBuffer();

void MAX5402_SPI0_init(void);
void MAX5402_SPI_write(uint8_t data);

uint8_t sdr;	//serial data received
uint8_t MAX5402_data;	//data to be written to MAX5402
uint8_t pstate = 0; //present state
uint8_t d2, d1, d0;	//digits of the decimal value received
uint32_t decimal;		//binary value equal to decimal value received

int main(void)
{
	USART_RxTail = 0x00;
	USART_RxHead = 0x00;
	USART_TxTail = 0x00;
	USART_TxHead = 0x00;
	
	PORTB.DIR &= ~PIN2_bm;
	PORTB.PIN2CTRL = 0x08;
	sei();          /* Enable interrupts => enable UART interrupts */
	USART0_Init( 9600 );   /* Set the baudrate to 19,200 bps using a 3.6864MHz crystal */
	
	/* Replace with your application code */
	MAX5402_SPI0_init(); // call init function
	while (1)
	{
		if(DataInReceiveBuffer()){
			sdr = (USART0_Receive()); // take in the serial data
			FSM: switch (pstate)
			{
				case 0:
					if (sdr == 'V')
						pstate = 1;
					else
						pstate = 0;
					break;
				
				case 1:
					if ((sdr >= '0') && (sdr <= '9'))
					{
						d2 = sdr & 0x0F;
						pstate = 2;
					}
					else
						pstate = 0;
					break;
				
				case 2:
					if ((sdr >= '0') && (sdr <= '9'))
					{
						d1 = sdr & 0x0F;
						pstate = 3;
					}
					else
						pstate = '0';
					break;
				
				case 3:
					if ((sdr >= '0') && (sdr <= '9'))
					{
						d0 = sdr & 0x0F;
						pstate = 4;
					}
					else
						pstate = 0;
					break;
				
				case 4:
					if (sdr == 0x0d)
						pstate = 5;
					else
						pstate = 0;
					break;
				
				case 5:
					if (sdr == 0x0a)
					{
						pstate = 0;
						decimal = (((d2 * 10) + d1) * 10) + d0;
						MAX5402_data = (uint8_t)(((decimal) * 255)/333);
						MAX5402_SPI_write(MAX5402_data); // write the built data
					}
					else
						pstate = 0;
					break;
					
				default:
					pstate = 0;
			}	
			_delay_ms(1000);
		}

		
	}
	
}


// use module 0 (SPI0)
// **********************************************************************
// Function: MAX5402_SPI0_init
// Parameters: None
// Return: None
// Description: Enables all the proper pins and sets up the input/output pins
//
// **********************************************************************
void MAX5402_SPI0_init(void){
    // setup pin input/output
    VPORTA_DIR = PIN4_bm | PIN6_bm;// | PIN7_bm;
    VPORTF_DIR = PIN2_bm; // setup CS/SS pin
    VPORTF_OUT = PIN2_bm; // deselect spi0;
    //PORTMUX.SPIROUTEA = 0x0; // select device SPI0
    SPI0.CTRLA = SPI_MASTER_bm | SPI_ENABLE_bm; //setup the functions and bits
    SPI0.CTRLB = SPI_SSD_bm; // turn off buffering
    
    //VPORTF_OUT = ~PIN2_bm; // deselect
    //SPI0.DATA = 0x23; // output dummy
}
// *****************************************************************************
// Function: MAX5402_SPI0_write
// Parameters: uint8_t
// Return: None
// Description: selects device, then writes 'data' to output spi0.data register
//
// *****************************************************************************
void MAX5402_SPI_write(uint8_t data){
    //MAX5402_SPI0_init();
    VPORTF_OUT &= ~PIN2_bm; // seelct
    SPI0.DATA = data; //output data
    while((SPI0.INTFLAGS & SPI_IF_bm) != SPI_IF_bm){ // wait until data is complete
        ; // do nothing until intflags is set, meaning transfer is complete
    }
    //SPI0_INTFLAGS &= ~SPI_IF_bm;
    VPORTF_OUT |= PIN2_bm; // deselect
}

// *****************************************************************************
// Function: USART0_Init
// Parameters: baudrate
// Return: None
// Description: Sets up usart for receiving
//
// *****************************************************************************
void USART0_Init( unsigned int baudrate )
{
    PORTB.DIR = 0x01;
    unsigned char x;

    /* Set the baud rate */
    //UBRR0H = (unsigned char) (baudrate>>8);
    USART3.BAUD = (64.0*4000000)/(16.0*baudrate);
    

    
    USART3.CTRLC = 0x03;
    
    /* Set frame format: 8 data 2stop */
    USART3.CTRLA = 0b10000000;             //For devices with Extended IO
    //UCSR0C = (1<<URSEL)|(1<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00);   //For devices without Extended IO
    
    /* Enable UART receiver and transmitter */
    USART3.CTRLB = 0b10000000;
    
    /* Flush receive buffer */
    x = 0;

    USART_RxTail = x;
    USART_RxHead = x;
    USART_TxTail = x;
    USART_TxHead = x;
}

// *****************************************************************************
// Function: USART3_RXC_vect ISR
// Parameters: None
// Return: None
// Description: Triggers when something is being received. then adds that
// received value to the circular buffer
// *****************************************************************************
ISR(USART3_RXC_vect)
{
    cli();
    unsigned char data;
    unsigned char tmphead;

    /* Read the received data */
    data = USART3.RXDATAL;
    /* Calculate buffer index */
    tmphead = ( USART_RxHead + 1 ) & USART_RX_BUFFER_MASK;
    USART_RxHead = tmphead;      /* Store new index */

    if ( tmphead == USART_RxTail )
    {
        /* ERROR! Receive buffer overflow */
    }
    
    USART_RxBuf[tmphead] = data; /* Store received data in buffer */
    sei();
}

// *****************************************************************************
// Function: USART0_Receive
// Parameters: None
// Return: char
// Description: takes out something in the circuclar buffer and returns it then
// updates the circular buffer
// *****************************************************************************
unsigned char USART0_Receive( void )
{
    unsigned char tmptail;
    
    while ( USART_RxHead == USART_RxTail )  /* Wait for incomming data */
    ;
    tmptail = ( USART_RxTail + 1 ) & USART_RX_BUFFER_MASK;/* Calculate buffer index */
    
    USART_RxTail = tmptail;                /* Store new index */
    
    return USART_RxBuf[tmptail];           /* Return data */
}

// *****************************************************************************
// Function: DataInReceiveBuffer
// Parameters: None
// Return: int
// Description: Returns a 1, if the circular buffer is NOT empty, otherwise
// will return false.
// *****************************************************************************
int DataInReceiveBuffer( void )
{
    return ( USART_RxHead != USART_RxTail ); /* Return 0 (FALSE) if the receive buffer is empty */
}
