/*
 * config.h
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

#ifndef CONFIG_H_
#define CONFIG_H_

/* Set our clock define so delay functions are happy */
#define F_CPU   32000000UL

/* XNRF24L01 Config */
#define NRF_CHANNEL 100             /* default nRF channel */
#define NRF_RATE    XNRF_250KBPS    /* default nRF data rate */
#define ADDR_P0     0xF0F0F0F0E1LL  /* default Pipe 0 address */
#define ADDR_P1     0xF0F0F0F0D2LL  /* default Pipe 1 address */
#define NRF_CBITS   0b00111100      /* default configuration bits - 2 byte CRC, RX_DR enabled */
/*                    ^^^^^^^^
 *                    ||||||||_____ PRIM_RX - RX/TX control
 *                    |||||||______ PWR_UP - Power control         
 *                    ||||||_______ CRCO - CRC encoding scheme; '0' - 1 byte, '1' - 2 bytes
 *                    |||||________ EN_CRC - Enable CRC
 *                    ||||_________ MASK_MAX_RT - Reflect max retry on IRQ pin - '0' to enable
 *                    |||__________ MASK_TX_DS - Reflect TX data sent on IRQ pin - '0' to enable
 *                    ||___________ MASK_RX_DR - Reflect RX data received on IRQ pin - '0' to enable
 *                    |____________ RESERVED - Only '0' allowed
 */

/* XUSART Config */
#define USART_BAUDRATE  100000	/* Baudrate for WS2811 stream generation */

/* RingBuffer Config */
#define BUFFER_SIZE     255

#endif /* CONFIG_H_ */