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

#define STATUS_LED_ON   PORTA.OUTSET = PIN5_bm
#define STATUS_LED_OFF  PORTA.OUTCLR = PIN5_bm
#define STATUS_LED_TGL  PORTA.OUTTGL = PIN5_bm
#define DATA_LED_ON		PORTA.OUTSET = PIN6_bm
#define DATA_LED_OFF	PORTA.OUTCLR = PIN6_bm
#define DATA_LED_TGL	PORTA.OUTTGL = PIN6_bm

/* XSPI configuration structure for nRF24L01 */
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

/* XUSART configuration structure for pixel data stream */
xusart_config_t xusart_config = {
    .usart = &USARTD0,
    .port = &PORTD,
    .tx_pin = 3
};

uint64_t    addr_p0 = ADDR_P0;							/* default nRF address for TX and Pipe 0 RX */
uint64_t    addr_p1 = ADDR_P1;							/* default nRF address for Pipe 1 RX */    
volatile uint8_t rxbuff[32];							/* Packet buffer */
volatile uint8_t unibuff[512];							/* Universe buffer */
volatile uint8_t offset = 0;							/* Universe buffer offset */
volatile uint16_t channels = PIXEL_NUM * PIXEL_SIZE;	/* Total number of channels */
volatile bool	DFLAG = false;							/* Data-Ready flag */

/* Initialize the board */
void init() {
    // Configure clock to 32MHz
    OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;    /* Enable the internal 32MHz & 32KHz oscillators */
    while(!(OSC.STATUS & OSC_RC32KRDY_bm));         /* Wait for 32Khz oscillator to stabilize */
    while(!(OSC.STATUS & OSC_RC32MRDY_bm));         /* Wait for 32MHz oscillator to stabilize */
    DFLLRC32M.CTRL = DFLL_ENABLE_bm ;               /* Enable DFLL - defaults to calibrate against internal 32Khz clock */
    CCP = CCP_IOREG_gc;                             /* Disable register security for clock update */
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;                /* Switch to 32MHz clock */
    OSC.CTRL &= ~OSC_RC2MEN_bm;                     /* Disable 2Mhz oscillator */

    // Configure general IO
    PORTA.DIRSET = PIN5_bm | PIN6_bm;				/* Setup Status and DATA LEDs as outputs */
    STATUS_LED_OFF;
	DATA_LED_OFF;
	
	//TODO: Pull buffer enable low.  Don't need on new boards
	PORTD.DIRSET = PIN2_bm;
	PORTD.OUTCLR = PIN2_bm;
	
	// Configure the DMA controller
	EDMA.CTRL = 0;									/* Disable EDMA controller so we can update it */
	EDMA.CTRL = EDMA_RESET_bm;						/* Reset the EDMA controller */
	EDMA.CTRL = EDMA_CHMODE_STD0_gc;				/* Configure for 1 standard (CH0) and 2 peripheral (CH2-3) channels */
	EDMA.CTRL |= EDMA_ENABLE_bm;					/* Enable the EDMA controller */
		
	// Configure DMA Standard Channel 0 for rxbuff to unibuff transfer */
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;		/* Increment source pointer during transfer and reset after transaction */
	EDMA.CH0.DESTADDRCTRL = EDMA_CH_DIR_INC_gc;									/* Increment destination pointer during transfer */
	EDMA.CH0.TRFCNT = 30;														/* Transfer 30 bytes per transaction */
	EDMA.CH0.ADDR = (uint16_t)rxbuff;											/* Source address is rxbuff[] */
	EDMA.CH0.DESTADDR = (uint16_t)unibuff;										/* Destination address is unibuff[] */
	
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
	PMIC.CTRL |= PMIC_LOLVLEN_bm;			/* Enable low interrupts */

    // Initialize listening on nRF.
    xnrf_config_rx(&xnrf_config);   /* Configure nRF for RX mode */
    xnrf_powerup(&xnrf_config);     /* Power-up the nRF */
    _delay_ms(5);                   /* Let the radio stabilize - Section 6.1.7 - Tpd2stdby */
}

/* Interrupt handler for nRF hardware interrupt on PC3
 * - This grabs an entire RF24 packet and puts the full universe in the buffer.
 *   We'll pick and process our desired channels later during pixel stream generation.
 */
ISR(PORTC_INT_vect) {
	DATA_LED_ON;
    xnrf_read_payload(&xnrf_config, rxbuff, xnrf_config.payload_width);		/* Retrieve the payload */
    xnrf_write_register(&xnrf_config, NRF_STATUS, (1 << RX_DR));			/* Reset nRF RX_DR status */

	//TODO: Add check for command byte
	offset = rxbuff[RFSC_FRAME] * RFSC_FRAME;					/* Set offset according to frame byte */
	EDMA.CH0.DESTADDR = (uint16_t)unibuff + offset;				/* Set DMA CH0 destination address to correct offset for this frame */
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;	/* Enable and trigger DMA channel 0 */
    PORTC.INTFLAGS = PIN3_bm;									/* Clear interrupt flag for PC3 */

	DFLAG = true;												/* Set our Data-Ready flag */
	DATA_LED_OFF;
 }

/* Setup a WS2811 data stream using TC4, XCL, and USARTD0 */
void setup_ws2811() {
	// Configure the USART module
	PORTD.DIRSET = PIN1_bm | PIN3_bm;										/* Setup TX and Clock lines as outputs */
	xusart_spi_init(&xusart_config, USART_UCPHA_bm);						/* Initialize USART as Master SPI, Mode 1 */
	xusart_spi_set_baudrate(xusart_config.usart, WS2811_BAUDRATE, F_CPU);	/* Set baud rate */
	xusart_enable_tx(xusart_config.usart);									/* Enable module TX */
	
	// Setup event system channels
	EVSYS.CH7MUX = EVSYS_CHMUX_PORTD_PIN1_gc;		/* SPI Clock (PD1) to CH7 */	
	PORTD.PIN1CTRL |= PORT_ISC_RISING_gc;			/* Sense rising edge on PD1 */
	EVSYS.CH1MUX = EVSYS_CHMUX_PORTD_PIN3_gc;		/* TXD (PD3) to CH1 / LUT IN2 */
	PORTD.PIN3CTRL |= PORT_ISC_LEVEL_gc;			/* Sense level on PD3 */
	EVSYS.CH6MUX = EVSYS_CHMUX_PORTD_PIN4_gc;		/* CCA (PD4) Compare to CH6 / LUT IN0 */
	PORTD.PIN4CTRL |= PORT_ISC_LEVEL_gc;			/* Sense level on PD4 */
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTD_PIN5_gc;		/* CCB (PD5) Compare to CH0 / LUT IN1 */
	PORTD.PIN5CTRL |= PORT_ISC_LEVEL_gc;			/* Sense level on PD5 */

	// Setup timers for WS2811 waveform generation
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
	
	/* Blank the universe */
	uint16_t channel = channels;
	while (channel--)
		xusart_putchar(xusart_config.usart, 0);
	WS2811_RESET;
}

/* Configure DMA channels */
void setup_dma() {
	// Configure DMA Peripheral Channel for USART Pixel TX
//	EDMA.CH2.CTRLA = EDMA_CH_SINGLE_bm;										/* Enable Single-Shot data transfer */
//	EDMA.CH2.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;	/* Reload address after transaction, increment pointer */
//	EDMA.CH2.TRIGSRC = EDMA_CH_TRIGSRC_USARTD0_DRE_gc;						/* Use USARTD0 DRE as DMA trigger */
//	EDMA.CH2.TRFCNT = channels;												/* Set transfer count size */
//	EDMA.CH2.ADDR = (uint16_t)rxbuff;										/* Set buffer address */

	// Enable DMA Channels
	//EDMA.CH2.CTRLA |= EDMA_CH_ENABLE_bm;	/* Enable USART TX channel */
}

/* Pulse entire universe full white - ignore configured channel count */
void identify() {
	uint16_t	i;
	
	while(1) {
		i = 512; while (i--)
			xusart_putchar(xusart_config.usart, 0xFF);
		STATUS_LED_ON;
		_delay_ms(250);

		i = 512; while (i--)
			xusart_putchar(xusart_config.usart, 0x00);
		STATUS_LED_OFF;
		_delay_ms(250);
	}
}

int main(void) {
    init();				/* Initialize the board */
	
	//TODO: Add logic to setup based on pixel type. Right now, we're only ws2811
	setup_ws2811();		/* Configure for WS2811 output */
	
	// Enable global interrupts and start listening
	sei();                          /* Enable global interrupt flag */
	xnrf_enable(&xnrf_config);      /* Start listening on nRF */
	
	//identify();


	while(1) {
		//TODO:  This should trigger on completion of DMA Channel - rxbuff->unibuff transfer complete
		while(!DFLAG);		/* Spin our wheels until we have a new packet */
		DFLAG = false;		/* Clear the flag */
		STATUS_LED_ON;

		//while(!(EDMA.CH0.CTRLB & EDMA_CH_TRNIF_bm));
		//EDMA.CH0.CTRLB = EDMA_CH_TRNIF_bm;

		for (uint16_t i = CHANNEL_START; i < channels; i++)
			xusart_putchar(xusart_config.usart, unibuff[i]);	/* Send the channel data */
		STATUS_LED_OFF;
	}
}