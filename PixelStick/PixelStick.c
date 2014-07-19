/*
 * PixelStick.c
 *
 * Project: PixelStick
 * Copyright (c) 2014 Shelby Merrick
 * http://www.forkineye.com
 *
 *  This program is provided free for you to use in any way that you wish,
 *  subject to the laws and regulations where you are using it.  Due diligence
 *  is strongly suggested before using this code.  Please give credit where due.
 *
 *  The Author makes no warranty of any kind, express or implied, with regard
 *  to this program or the documentation contained in this document.  The
 *  Author shall not be liable in any event for incidental or consequential
 *  damages in connection with, or arising out of, the furnishing, performance
 *  or use of these programs.
 *
 */ 

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>
#include "XUSART/XUSART.h"
#include "XNRF24L01/XNRF24L01.h"
#include "renard.h"

#define STATUS_LED_ON   PORTA.OUTCLR = PIN0_bm
#define STATUS_LED_OFF  PORTA.OUTSET = PIN0_bm
#define STATUS_LED_TGL  PORTA.OUTTGL = PIN0_bm
#define DATA_LED_ON		PORTA.OUTCLR = PIN1_bm
#define DATA_LED_OFF	PORTA.OUTSET = PIN1_bm
#define DATA_LED_TGL	PORTA.OUTTGL = PIN1_bm
//TODO: Change this to 50 and have chip wait til USART is cleared
#define WS2811_RESET	_delay_us(60)	// Spec is >50us

// Temp defines
#define NUM_PIXELS 12
#define START_CHANNEL 0

/* XSPI configuration structure */
xspi_config_t xspi_config = {
    .spi = &SPIC,
    .port = &PORTC,
    .mosi_pin = 7,
    .miso_pin = 6,
    .sck_pin = 5,
    .ss_pin = 4
};

/* XNRF24L01 configuration structure */
xnrf_config_t xnrf_config = {
    .spi = &SPIC,
    .spi_port = &PORTC,
    .ss_port = &PORTC,
    .ss_pin = 4,
    .ce_port = &PORTC,
    .ce_pin  = 2,
    .addr_width = 5,
    .payload_width = 32,
    .confbits = NRF_CBITS
};

/* XUSART configuration structure */
xusart_config_t xusart_config = {
    .usart = &USARTD0,
    .port = &PORTD,
    .tx_pin = 3
};

uint64_t    addr_p0 = ADDR_P0;					/* default nRF address for TX and Pipe 0 RX */
uint64_t    addr_p1 = ADDR_P1;					/* default nRF address for Pipe 1 RX */    
//RingBuff_t  ringbuff;							/* Ring buffer to hold our data */
volatile uint8_t rxbuff[32];					/* Packet buffer */
volatile uint8_t unibuff[512];					/* Universe buffer */
volatile uint8_t offset = 0;					/* Current offset within RX buffer */
volatile uint16_t channels = NUM_PIXELS * 3;	/* Total number of channels */
volatile bool DFLAG = false;

/* Initialize the board */
//TODO: Add DMA support
void init() {
    // Configure clock to 32MHz
    OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;    /* Enable the internal 32MHz & 32KHz oscillators */
    while(!(OSC.STATUS & OSC_RC32KRDY_bm));         /* Wait for 32Khz oscillator to stabilize */
    while(!(OSC.STATUS & OSC_RC32MRDY_bm));         /* Wait for 32MHz oscillator to stabilize */
    DFLLRC32M.CTRL = DFLL_ENABLE_bm ;               /* Enable DFLL - defaults to calibrate against internal 32Khz clock */
    CCP = CCP_IOREG_gc;                             /* Disable register security for clock update */
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;                /* Switch to 32MHz clock */
    OSC.CTRL &= ~OSC_RC2MEN_bm;                     /* Disable 2Mhz oscillator */

    // Initialize ring buffer
    //RingBuffer_InitBuffer(&ringbuff);
        
    // Configure general IO
    PORTA.DIRSET = PIN0_bm | PIN1_bm;	/* Setup Status and DATA LEDs as outputs */
    STATUS_LED_OFF;
	DATA_LED_OFF;
    
    // Configure the nRF radio
    xnrf_init(&xnrf_config, &xspi_config);                  /* Initialize the XNRF driver */
    xnrf_set_channel(&xnrf_config, NRF_CHANNEL);            /* Set our channel */
    xnrf_set_datarate(&xnrf_config, NRF_RATE);              /* Set our data rate */
    xnrf_write_register(&xnrf_config, EN_AA, 0);            /* Disable auto ack's */
    xnrf_write_register(&xnrf_config, SETUP_RETR, 0);       /* Disable auto retries */
    xnrf_write_register(&xnrf_config, EN_RXADDR, 3);        /* Listen on pipes 0 & 1 */
    xnrf_set_tx_address(&xnrf_config, (uint8_t*)&addr_p0);  /* Set TX address */
    xnrf_set_rx0_address(&xnrf_config, (uint8_t*)&addr_p0); /* Set Pipe 0 address */
    xnrf_set_rx1_address(&xnrf_config, (uint8_t*)&addr_p1); /* Set Pipe 1 address */

    // Setup pin change interrupt handling for the nRF on PC3
    PORTC_PIN3CTRL = PORT_ISC_FALLING_gc;   /* Setup PC3 to sense falling edge */
    PORTC.INTMASK = PIN3_bm;                /* Enable pin change interrupt for PC3 */
    PORTC.INTCTRL = PORT_INTLVL_LO_gc;      /* Set Port C for low level interrupts */

    // Initialize listening on nRF.
    xnrf_config_rx(&xnrf_config);   /* Configure nRF for RX mode */
    xnrf_powerup(&xnrf_config);     /* Power-up the nRF */
    _delay_ms(5);                   /* Let the radio stabilize - Section 6.1.7 - Tpd2stdby */
    
    // Enable interrupts and start listening
    PMIC.CTRL |= PMIC_LOLVLEN_bm;   /* Enable low interrupts */
    sei();                          /* Enable global interrupt flag */
    xnrf_enable(&xnrf_config);      /* start listening on nRF */
}

/* Interrupt handler for nRF hardware interrupt on PC3
 * - This grabs an entire RF24 packet and puts the full universe in the buffer.
 *   We'll pick and process our desired channels later during pixel stream generation.
 */
ISR(PORTC_INT_vect) {
    xnrf_read_payload(&xnrf_config, unibuff + offset, xnrf_config.payload_width);    /* Retrieve the payload */
    xnrf_write_register(&xnrf_config, NRF_STATUS, (1 << RX_DR));					/* Reset the RX_DR status */
	offset += 30;																	/* Discard frame and reserved bytes on offset increment */
    PORTC.INTFLAGS = PIN3_bm;														/* Clear interrupt flag for PC3 */
 }

/* Main loop for handling WS2811 pixels */
void loop_ws2811() {
	/* Blank the universe */
	uint16_t channel = channels;
	while (channel--)
		xusart_putchar(xusart_config.usart, 0);
	WS2811_RESET;

    while(1) {
        while(offset < channels);	/* Spin our wheels until we have a packet */
		offset = 0;					/* Reset our offset */

		for (uint16_t i = START_CHANNEL; i < channels; i++)
			xusart_putchar(xusart_config.usart, unibuff[i]);	/* Send the channel data */
		WS2811_RESET;											/* Trigger WS2811 Reset signal */
    }        
}    

/* Setup a WS2811 data stream using TC4, XCL, and USARTD0 */
void setup_ws2811() {
	// Configure the USART module
	PORTD.DIRSET = PIN1_bm | PIN3_bm;									/* Setup TX and Clock lines as outputs */
	xusart_init(&xusart_config);                                        /* Initialize the XUSART driver */
	xusart_config.usart->CTRLC = USART_CMODE_MSPI_gc | USART_UCPHA_bm;	/* Setup USART as Master SPI, Mode 1 */
	xusart_set_baudrate(xusart_config.usart, USART_BAUDRATE, F_CPU);    /* Set baud rate */
	xusart_enable_tx(xusart_config.usart);                              /* Enable module TX */

	// Setup event system channels
	EVSYS.CH7MUX = EVSYS_CHMUX_PORTD_PIN1_gc;		/* SPI Clock (PD1) to CH7 */	
	PORTD.PIN1CTRL |= PORT_ISC_RISING_gc;	
	EVSYS.CH1MUX = EVSYS_CHMUX_PORTD_PIN3_gc;		/* TXD (PD3) to CH1 / LUT IN2 */
	PORTD.PIN3CTRL |= PORT_ISC_LEVEL_gc;
	EVSYS.CH6MUX = EVSYS_CHMUX_PORTD_PIN4_gc;		/* CCA (PD4) Compare to CH6 / LUT IN0 */
	PORTD.PIN4CTRL |= PORT_ISC_LEVEL_gc;
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTD_PIN5_gc;		/* CCB (PD5) Compare to CH0 / LUT IN1 */
	PORTD.PIN5CTRL |= PORT_ISC_LEVEL_gc;

	PORTD.DIRSET = PIN4_bm | PIN5_bm;									/* Enable output on PD4 & PD5 for compare channels */
	TCD5.CTRLB = TC45_WGMODE_SINGLESLOPE_gc;							/* Single Slope PWM */
	TCD5.CTRLD = TC45_EVACT_RESTART_gc | TC45_EVSEL_CH7_gc;				/* Restart on CH7 pulse - rising clock edge */
	TCD5.CTRLE = TC45_CCAMODE_COMP_gc | TC45_CCBMODE_COMP_gc;			/* Enable output compare on CCA & CCB */
	TCD5.PER = 40 - 1;													/* At 32MHz, 1 cycle = 31.25ns.  Define top of counter for a 1250ns pulse: (32MHz / 800KHz) */
	TCD5.CCA = 8;														/* Compare for 0 bit @ 250ns (31.25ns * 8). Output is on PD4 */
	TCD5.CCB = 32;														/* Compare for 1 bit @ 1000ns (31.25ns * 32). Output is on PD5 */
	TCD5.CTRLA = TC45_CLKSEL_DIV1_gc | TC5_EVSTART_bm | TC5_UPSTOP_bm;	/* Start and stop the timer on each event occurrence, full speed clock */

	// Setup XCL
	PORTD.DIRSET = PIN0_bm;														/* Enable output on PD0 for LUT OUT0 */
	XCL.CTRLA = XCL_LUTOUTEN_PIN0_gc | XCL_PORTSEL_PD_gc | XCL_LUTCONF_MUX_gc;	/* Setup LUT with output on PD0 as MUX */
	XCL.CTRLD = 0b10100000;														/* Truth Tables - Ignore 0 since its a MUX. Setup LUT1 to pass IN2. */
}

/* Configure DMA channels */
void setup_dma() {
	// Reset the DMA controller
	EDMA.CTRL = 0;
	EDMA.CTRL = EDMA_RESET_bm;
	
	// Configure DMA Peripheral Channel
	EDMA.CH0.CTRLA = EDMA_CH_SINGLE_bm;										/* Enable Single-Shot data transfer */
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;	/* Reload address after transaction, increment pointer */
	EDMA.CH0.TRIGSRC = EDMA_CH_TRIGSRC_USARTD0_DRE_gc;						/* Use USARTD0 DRE as DMA trigger */
	EDMA.CH0.TRFCNT = channels;												/* Set transfer count size */
	EDMA.CH0.ADDR = (uint16_t)rxbuff;										/* Set buffer address */

	// Enable DMA
	EDMA.CTRL = EDMA_ENABLE_bm;				/* Enable DMA controller */
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;	/* Enable our DMA Channel */
}

/* Pulse entire universe full white - ignore configured channel count */
void identify() {
	uint16_t	i;
	
	while(1) {
		i = 512; while (i--)
			xusart_putchar(xusart_config.usart, 0xFF);
		STATUS_LED_TGL;     /* Toggle status LED */
		_delay_ms(250);

		i = 512; while (i--)
			xusart_putchar(xusart_config.usart, 0x00);
		STATUS_LED_TGL;     /* Toggle status LED */
		_delay_ms(250);
	}
}

int main(void) {
    init();

	//setup_dma();
	setup_ws2811();
	loop_ws2811();
	
	//identify();	
}