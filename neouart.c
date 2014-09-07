#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>		// Needed for memset()
#include <ctype.h>  	// Needed for ishexdigit() and isdigit() in parser
#include <stdbool.h>	// Needed for bool type

/**
 *
 /*
 *  NeoUART - A utility for controlling a NeoPixel from a Raspberry PI
 *
 *  Connect...
 *   NeoPixel data pin -> Raspberry PI GPIO 14 (pin P1-08)
 *   Neopixel GND  pin -> Raspberry PI any ground (pin P1-06, for example)
 *   NeoPixel VCC  pin -> Raspberry PI 3V3 pin (P1-01)
 *
 *
 */

// Timing constants

#define CPU_FRQ 		(250000000)			// Main system clock at 250mhz
#define BIT_FRQ			(1000000000/400)	// Target output bit width = 400ns as per WS2812data sheet. Note that we will make the H and L data pulses out of 1 or 2 of these bits

#define UART_FRQ 		(BIT_FRQ*8)			// There are uart 8 cycles per bit on the mini uart BAP11
#define BAUD_DIV  		(CPU_FRQ/UART_FRQ)	// How many cpu cycles per UART cycle?
											// Note that this calculation rounds down to a divisor of 12, which yields a bit width of 480ns which is perfect for NeoPixels!

#define NEOPIXEL_RES_US	(50)				// How long to hold the data line low to latch a NeoPixel as per WS2812B data sheet
											// Note: imperically I have found this only needs to be about 10us, but we will go by the book here.


// Lots of extremely helpful and well-written code & ideas borrowed from...
//
// https://github.com/richardghirst/PiBits/blob/master/ServoBlaster/user/servod.c
// Example of clean way to map and access peripherals

// https://github.com/dwelch67/raspberrypi/blob/master/uart01/uart01.c
// Example of how to access the mini-UART

// All code below is written for clarity and not speed. There are so many places here that would
// be so satisfying to rewrite in super-optimized, hand crafted ARM assembler. Fight the urge,
// becuase this program spends 99.999% of its time waiting for the UART, so all the work would be for nothing.

// All references in the form BAPn mean refer to page n in the BCM2835 ARM Peripherals guide here...
// http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf


// Note that all addresses here are physical as needed by memmap()
// so 0x7E00 0000 in the Broadcom data sheet = 0x2000 0000 here


// For defining peripheral register locations, we first define the
// physical base address as xxx_BASE and then then each register is
// defined as the offset from the base. xxx_LEN sets the length of the peripheral
// address space. The AP() macro is used to apply an offset to a map_peripheral returned pointer.



// General Purpose I/O (GPIO)
#define GPIO_BASE 			0x20200000						// BAP-89

#define GPFSEL1 			0x04
#define GPSET0 				0x1C
#define GPCLR0 				0x28
#define GPPUD 				0x94
#define GPPUDCLK0 			0x98
#define GPIO_LEN 			0xB4


// Auxiliary peripherals Register Map
#define AUX_BASE 			0x20215000						// BAP-8

#define AUX_ENABLES 		0x04							// BAP9

#define AUX_ENABLES_SPI2		(BV(2))							// Enable AUX SPI2
#define AUX_ENABLES_SPI1		(BV(1))							// Enable AUX SPI1
#define AUX_ENABLES_UART 		(BV(0))							// Enable AUX mini UART

#define AUX_MU_IO_REG 		0x40
#define AUX_MU_IER_REG 		0x44

#define AUX_MU_IER_REG_RXCLEAR 	(BV(1))				// Clear RX FIFO BAP13
#define AUX_MU_IER_REG_TXCLEAR 	(BV(2))				// Clear RX FIFO BAP13


#define AUX_MU_IIR_REG 		0x48

#define AUX_MU_IIR_REG_CLR_TX_FIFO	(BV(2))			// BAP 13 Clear the transmit fifo
#define AUX_MU_IIR_REG_CLR_RX_FIFO	(BV(1))			// BAP 13 Clear the receive fifo


#define AUX_MU_LCR_REG 		0x4C

#define AUX_MU_LCR_REG_8BIT			(BV(0)|BV(1))		// 8 bit data size (docs wrong) BAP14
#define AUX_MU_LCR_REG_BREAK 		(BV(6))				// If set high the UART1_TX line is pulled low continuously BAP14


#define AUX_MU_MCR_REG 		0x50

#define AUX_MU_LSR_TX_ILDE			(BV(6))				// BAP 15 This bit is set if the transmit FIFO is empty and the transmitter is idle. (Finished shifting out the last bit).
#define AUX_MU_LSR_TX_EMPTY			(BV(5))				// BAP 15 This bit is set if the transmit FIFO can accept at least one byte.
#define AUX_MU_LSR_RX_OVERRUN		(BV(1))				// BAP 15 This bit is set if there was a receiver overrun.
#define AUX_MU_LSR_RX_READY			(BV(0))				// BAP 15 This bit is set if the receive FIFO holds at least 1 symbol.


#define AUX_MU_LSR_REG 		0x54


#define AUX_MU_MSR_REG 		0x58
#define AUX_MU_SCRATCH 		0x5C
#define AUX_MU_CNTL_REG		0x60

#define AUX_MU_CNTL_REG_TXEN	(BV(1))				// Enable the transmitter BAP17


#define AUX_MU_STAT_REG		0x64

#define AUX_MU_STAT_TX_DONE			(BV(9))				// BAP18 This bit is set if the transmitter is idle and the transmit FIFO is empty. It is a logic AND of bits 2 and 8
#define AUX_MU_STAT_TX_EMPTY 		(BV(8))				// BAP18 If this bit is set the transmitter FIFO is empty. Thus it can accept 8 symbols.
#define AUX_MU_STAT_TX_FULL			(BV(5))				// BAP18 Transmit FIFO is full. This is the inverse of bit 1.
#define AUX_MU_STAT_TX_ILDE			(BV(3))				// BAP18 This bit tells if the transmitter is idle. Note that the bit will set only for a short time if the transmit FIFO contains data. Normally you want to use bit 9: Transmitter done.
#define AUX_MU_STAT_TX_SPACE		(BV(1))				// BAP18 If this bit is set the mini UART transmitter FIFO can accept at least one more symbol.


#define AUX_MU_BAUD_REG 	0x68
#define AUX_LEN 			0x6B



// AP = Access Peripheral
// Takes a base address returned from map_peripheral and an offset in bytes from the base

// We divide offset by 4 because it is given in bytes, but the pointer is to 32 bit words which are each 4 bytes wide
// so pointer arithmetic will multiply anything added to the base by 4

#define AP( base , offset ) (*(base+(offset/4)))


// Convenience function for bit value
#define BV(bit) (1<<bit)


// Map a peripheral from a physical address into a userspace memory pointer

static volatile unsigned *map_peripheral(unsigned base, unsigned len)
{

	int fd = open("/dev/mem", O_RDWR);
	void * vaddr;
	if (fd < 0) {
		perror("Failed to open /dev/mem");
		exit(-1);
	}
	vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);

		if (vaddr == MAP_FAILED) {

			perror("Failed to map peripheral");
			exit(-1);

	}
	close(fd);

	return vaddr;
}

volatile unsigned *aux;


// SetupUART1 gets everything ready for the mini UART to send NeoPixel data
// It leaves pin P1-08 in a low state by asserting break, the FIFO empty, and the TX disabled

void setupUart1()
{

	aux  = map_peripheral(  AUX_BASE ,  AUX_LEN );

	AP(aux, AUX_ENABLES) |= AUX_ENABLES_UART;			// enable AUX UART BAP9
														// Must do first becuase the other registers do not even show
														// up until this is enabled
														// Use OR to preserve SPI enable bits


	AP( aux, AUX_MU_LCR_REG)  = AUX_MU_LCR_REG_8BIT | AUX_MU_LCR_REG_BREAK; // This just makes sure that the break is asserted so that when we switch over the GPIO pin
																			// it will stay low - rather than going high which is the normal state for the UART.
																			// We also select 8-bit data which we will use when we enable transit later


	AP( aux, AUX_MU_MCR_REG)  = 0;					// BAP14
	AP( aux, AUX_MU_IER_REG)  = 0;					// BAP13 No interrupts

	// The following line does not seem to be necessary - even tested after a power cycle and seems you do not need to write 1 to bits 7 & 8
	//	AP( aux, AUX_MU_IIR_REG)  = 0xC6; 		// BAP12 access 8 bits of baud rate, but bit 7 of LCR should be=1? Also 0:1 turns off RX and TX interrupts

	AP( aux, AUX_MU_IIR_REG)  = AUX_MU_IIR_REG_CLR_TX_FIFO; 		// BAP13 clear the transmit FIFO (docs are mixed up)


	// We only need to access the GPIO registers to set up the P1-08 pin to be connected to the UART1 TX

	volatile unsigned *gpio;
	gpio = map_peripheral( GPIO_BASE , GPIO_LEN );

	/*

		****
		**** P1-08 is a serial port TX pin by default when the PI powers up, so I am going to
		**** thoughtfully skip the step of disabling the pull-ups and -downs which would
		**** otherwise go here.
		****
		**** If you have other code code that mangles this pin before running this code,
		**** note that the worst case failure mode is that the UART output has to fight a
		**** pull-up or -down as wastes a tiny bit of power, but it should work fine.
		****

	 */

	// we need to get GPIO pin 14 into alternate mode #5 for the AUX UART TX to show up there
	// The pin 14 alt function is controlled by FSEL14 which is GPFSEL1 bits 14-12 per BAP92

	unsigned gpfsel1_temp = AP( gpio , GPFSEL1 ); 	// Get the current value of the gpio select register

	// We need to assign FSEL14 (which is bits 14-12) to 010 to make GPIO pin 14 take alt function 5 (BAP92)

	gpfsel1_temp &= ~( 0b111 << 12 );			// Clear bits 12-14
	gpfsel1_temp |= 0b010 << 12;				// Assign bits 12-14 = 010, select alt function 5

	AP( gpio , GPFSEL1 ) = gpfsel1_temp;		// Put new calculated value back into the register

	// Ok, the UART1 TX should be on GPIO pin 14 now! Yeay!


}


// An EncodedNeoBuffer type holds 24 bits of pixel data encoded into 8 bytes
// read to be transmitted out a UART (the correct speed must be used) and receieved by a NeoPixel

typedef struct {

	char bytes[8];

} EncodedNeoBuffer;



// Actually send 8 bytes of NeoPixel data to the mini-uart on pin P1-08
// expects that the UART has been setup and and break is asserted
// exits with break asserted and all bytes completely sent


void sendUart1( EncodedNeoBuffer *encodedNeoBuffer ) {

	unsigned char *data = encodedNeoBuffer->bytes;


	AP( aux , AUX_MU_CNTL_REG ) = 0;			// disable transmitter. BAP16
												// this will let us queue up bytes in the FIFO without
												// worrying about them draining while we do it
												// (remember that we could get interrupted anywhere while doing it and the fifo would still drain).

	// Clear out anything that might be lingering in the TX FIFO

	AP( aux , AUX_MU_IIR_REG) = AUX_MU_IER_REG_TXCLEAR;

	// stuff the FIFO with our bytes so they are ready to go out as soon as we enable the transmitter..

	int len = sizeof( encodedNeoBuffer->bytes );

	while ( len ) {

		  AP( aux , AUX_MU_IO_REG) = *data;

		  data++;
		  len--;

	  }

	// Ok, this little turn-on dance is a bit complicated
	// Coming into here, the transmitter is disabled, so the data is waiting in the FIFO to go out
	// We also assume that the break is aserted, so the signal is low. We need to transition to sending the data
	// but don't want to glitch to high before the data starts going out.

	// We do this by fist setting the baud rate to the lowest possible value, which will mean that the


	AP( aux, AUX_MU_BAUD_REG) = 0xffff;			// Direct access the 16 bit baud rate counter, the register gets the divisor - 1 as per BAP19
												//
												// Set to the slowest possible speed. This will give us plenty of time between when the transmitter starts
												// sending the 1st stop bit (which should be a 0) and when we de-assert break and update the baud to the
												// real rate. We need this time because it is possible we could get prempted by an interrupt between
												// these steps.

												// with a baudreate_reg of ((2^16)-1), which is 65535, we each bit will last....
												//
												// (1/250mhz) * 8 * (baudrate_reg+1) = 2.097152 milliseconds
												// This should be long enough to reide though any interrupts or task switches that might preempt us
												// between when we enable the transmitter and when we disable the break generation.
												//
												// As long as we make sure the 1st bit out the gate is a 0, and that we wait until the transmitter
												// starts sending that 0 bit then the transition from break to data should
												// be seamless and glitch free.
												//
												// Note that having additional 0 bits at the begining of the 1st byte will give us even more time to
												// to ride out a possible interruptions. The encoding style used in NeoUART ensures that the first bit of
												// of every byte is 0, so we will actually get >4ms of ride though time.


	AP( aux , AUX_MU_CNTL_REG ) = AUX_MU_CNTL_REG_TXEN;			// Bit 1 = enable transmitter. BAP16
												// Note that the transmitter will immediately start sending the 1st byte in the fifo, but we cant see it
												// on the pin yet because the break is still asserted and that covers any data that the UART maybe sending


	while ( AP( aux , AUX_MU_STAT_REG ) & AUX_MU_STAT_TX_FULL ); 	// Wait for the 1st byte in the FIFO to get loaded into the transmitter
																	// which indicates two things...
																	// 1) there is now a zero start bit (slowly) being transmitted on the UART, so we are
																	//    safe to de-assert the break signal without causing a glitch
																	// 2) there is now a free byte at the end of the FIFO that we can stuff
																	//    with a trailing 0 as a buffer

	AP( aux , AUX_MU_IO_REG) = 0;				// Stuff a trailing 0 in the very end of the buffer as padding. This makes sure that there is an extra
												// start bit and some zero bits at the end of our real data. Imagine that the very last bit of real data is a
												// 0 and just after that bit goes out, we get pre-empted before we can asser tthe break signal. That 0 bit could now
												// become a 1. This trailing 0 will ensure that there is an extra  0 bit (9 of them, actually) to properly
												// terminate that final real 0 data bit. If something prevents us from asserting break in time after this
												// trailing 0 byte, the final stop bit could get expanded into a neopixel 1 bit, but that is ok
												// becuase the neopixel we are controlling will have already seen his 24 bits of good data and will
												// ignore that extra 1 bit.



	AP( aux, AUX_MU_LCR_REG)  = AUX_MU_LCR_REG_8BIT; // This will de-assert the break signal, so now the pin will now show whatever the uart is actually transmitting,
													 // which should be the beginning of the 1st byte in the fifo which should be going out very slowly...

	AP( aux, AUX_MU_BAUD_REG) = BAUD_DIV-1;		// direct access to the 16 bit baud rate counter, the register gets the divisor - 1 as per BAP19
												// Set the real target baud rate, which will speed up the remaining bits in the transmitter



    while ( ! (AP( aux , AUX_MU_STAT_REG ) & AUX_MU_STAT_TX_EMPTY) );	// Wait for all the bytes in the FIFO to be transmitted out...
    																	// Note that there will still be our trailing byte of 0 padding in the actual transmitter
    																	// even though the FIFO is empty. It is no problem if we assert break while those 0 bits are
    																	// are still being transmitted becuase the line will just be going from low (beucase of
    																	// of the tailing 0 bits) to low (becuase of the break signal).

    // If we get interrupted here it is no big deal because all the pixels already have thier data

    AP( aux, AUX_MU_LCR_REG)  = AUX_MU_LCR_REG_BREAK | AUX_MU_LCR_REG_8BIT;// BAP14 turn break on - make pin goto 0 volts

    usleep( NEOPIXEL_RES_US );										// Hold low for at least this long to let the NeoPixels latch
    																// Almost certainly not needed, but good from to have so the code will still work
    																// when the ARM77 @ 2.3THz comes out....

} // senddata



// Encode a 24 bit value into 8 bytes where each byte has the format...
// 0b?0?10?10
// Where each ? is a bit from the orginal 24 bit value.
// This wonky format is designed to generate correct NeoPixel
// signals when sent out a serial port and surrounded with stop and start bits.

void encodebits( unsigned x , EncodedNeoBuffer *buffer )  {

	int bits=24;

	unsigned char *b = buffer->bytes;

	while (bits) {

		// Process 3 bits of the input into 1 byte on the output
		//
		// Note that we processes the input by shifting up and checking the high bit (#23)
		// This is becuase NeoPixels actually take thier data in MSB first order
		// while serial ports send in LSB first order

		unsigned char t=0b00010010;		// initialize with all the known 1's

		if (x & (1<<23)  ) {
			t |= 0b00000100;
		}

		x <<= 1 ;
		bits--;


		if (x & (1<<23) ) {
			t |= 0b00100000;
		}

		x <<= 1 ;
		bits--;


		if (x & (1<<23) ) {
			t |= 0b10000000;
		}

		x <<= 1 ;
		bits--;

		*b = t;

		b++;

	}

}

// Display the passed color for durration on the attached NeoPixel
// Passed number in the format 0xDDRRGGBB
// DD = 0-ff durration in 1/100's of seconds
// RR = 0-ff red brightness
// GG = 0-ff green brightness
// BB = 0-ff blue brightness
//
// Nice format becuase durration defaults to 0 if ommited, and then you just have RRGGBB.


bool ws2811mode = false; 	// For WS28122 NeoPixels, the bytes are sent in RGB order.

bool verbosemode = true;	// print messages


void showpixel( unsigned pixelspec )  {

	EncodedNeoBuffer buffer;


	unsigned pixeldata;

	if (ws2811mode) {			// WS2811 expects data in RGB order

		pixeldata =   (pixelspec  & 0xFFFFFF);

	} else {					// WS2812 expects data in GRB order

		pixeldata =   ( (pixelspec  & 0x00FF00) << 8 ) | ( ( pixelspec & 0xFF0000 ) >> 8 ) | ( pixelspec & 0x0000FF) ;

	}


	encodebits(  pixeldata , &buffer );

	sendUart1( &buffer   );

	// Get durration byte. (I know AND not nessisary, but cleaner code)

	unsigned duration = ( ( pixelspec & 0xff000000 ) >> 24);

	if (verbosemode) {
		printf("showing pixel r=%02x g=%02x b=%02x d=%0d.%02ds\n", (pixelspec & 0xff0000) >> 16  , (pixelspec & 0xff00) >> 8  , (pixelspec & 0xff)   , duration / 100 , duration % 100 );
	}

	if (duration) usleep( duration * 10000); 	// Convert 1/100s to us

}

signed smoothsteps=0; 			// number of hundreths of seconds to smooth consecutive colors
								// This has to be signed becuase c does not do int/unsigned=int

// Previous pixel - start at zero
int r_previous=0;
int g_previous=0;
int b_previous=0;

// show next pixel with smoothing between consecutive pixels.

void shownextpixel( unsigned pixelspec ) {

	if (smoothsteps) {

		// Don't let all this bitshifting scare you - the ARM can do it exreemely efficiently

		int r_new = (pixelspec & 0xff0000) >> 16;
		int g_new = (pixelspec & 0x00FF00) >>  8;
		int b_new = (pixelspec & 0x0000FF);

		// Intermediate values held as 1/256th fixed point
		// Since the most number of steps is 255, this should avoid rounding errors

		int r_current_256ths = r_previous * 256;
		int g_current_256ths = g_previous * 256;
		int b_current_256ths = b_previous * 256;

		// needed this intermediate step to force the calculation to be signed.
		// I could not figure out a way to make a signed int liteereal without a cast. Can u?

		int r_diff = (r_new - r_previous);
		int g_diff = (g_new - g_previous);
		int b_diff = (b_new - b_previous);

		int r_step_256ths = (r_diff*256) / smoothsteps;
		int g_step_256ths = (g_diff*256) / smoothsteps;
		int b_step_256ths = (b_diff*256) / smoothsteps;

		int i;

		for( i= 0 ; i<(smoothsteps-1) ; i++ ) {	// Stop one step before final value. We will send final speorately to make suire we land directly on it rather than accumulaing rounding errors.

			r_current_256ths += r_step_256ths;
			g_current_256ths += g_step_256ths;
			b_current_256ths += b_step_256ths;

			int r = r_current_256ths/256;
			int g = g_current_256ths/256;
			int b = b_current_256ths/256;

			// Show each pixel step for 1/100th of a second
			unsigned pixelspec = ( 0x01 << 24 ) | ( r << 16) | ( (g  << 8 ) |  b );

			showpixel( pixelspec );

		}

		r_previous = r_new;
		g_previous = g_new;
		b_previous = b_new;

	}

	showpixel(pixelspec);

}



int main(int argc, char **argv)
{

  int arg=1;

  while (arg<argc && argv[arg][0]=='-') {

	  switch (tolower(argv[arg][1])) {

	      case 'q':
	    	  verbosemode=false;
	    	  break;

	      case 'i':
	    	  ws2811mode = true;

	    	  if (verbosemode) {
	    		  printf("Sending data in WS2811 mode.\n");
	    	  }
	    	  break;

	      case 's':
	    	  smoothsteps = atoi( argv[arg] +2  );

	    	  if (verbosemode) {
	    		  printf("Smoothing each change over %0d.%02d seconds.\n" , smoothsteps/100 , smoothsteps % 100);
	    	  }
	    	  break;

	      case '?':
			  printf("NeoUART (c)2014, josh. See http://josh.com for more info.\n");
			  printf("Drives a NeoPixel connected to Raspberry Pi pin P1-08\n");
			  printf("\n");
			  printf("Usage: neouart [-i] [-sn] [-q] [pixelspec ...]");
			  printf("\n");
			  printf("-i WS2811 mode where colors are sent in RGB order (default GRB)\n");
			  printf("-s smoothly trnasition between colors over n hundreths of a second.\n");
			  printf("-q quiet mode.\n");
			  printf("\n");
			  printf("Accepts pixelspecs in the format [DD]RRGGBB where...\n");
			  printf("[DD] is an optional duration in 1/100s of seconds, and\n");
			  printf("RRGGBB are each the 2 digit hex brightness level for\n");
			  printf("red, green, and blue respecively.\n");
			  printf("\n");
			  printf("All pixelspec values are hex numbers in the range 00-ff.\n");
			  printf("\n");
			  printf("Examples of pixelspecs:\n");
			  printf("000000=Black, 05FFFF=white for .05s, 800080=50% blue for 1.28s\n");
			  printf("\n");
			  printf("You can specify one or more pixelspecs on the command line, or run the\n");
			  printf("program without parameters and it will read and execute pixelspecs from\n");
			  printf("stdin. \n");
			  exit( -1 );

          default:
        	  printf("Unknown arg. Try %s -? for help.\n" , argv[0] );
        	  exit(-1);

	  }

	  arg++;

  }


  setupUart1();

  if (arg == argc )  { // No args specified, read pixelspec values from stdin...

	  if (verbosemode) {
		  printf("Reading from stdin. Try %s -? for help.\n" , argv[0] );
	  }

	  unsigned pixelspec;

	  while ( scanf( " %x" , &pixelspec ) == 1 ) {			// Read untill we hit something that can't be parsed or EOF

		  shownextpixel( pixelspec );

	  }

  } else {		// At least one param on the command line...

	  unsigned pixelspec;

	  while ( arg < argc ) {

		  sscanf( argv[arg] , "%x" , &pixelspec );

		  shownextpixel( pixelspec );

		  arg++;

	  }

  }

} // main
