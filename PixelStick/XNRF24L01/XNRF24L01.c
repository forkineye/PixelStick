/*
 * XNRF24L01.c
 *
 * Project: XNRF24L01
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

#include "XNRF24L01.h"
#include <avr/io.h>

//TODO: Change xnrf_init so it doesn't assume a 32MHz clock, or change xspi_master_init to reference baud rates.
//TODO: Change this to xnrf_init_spi and add xnrf_init_usart??
void xnrf_init(xnrf_config_t *xnrf_config, xspi_config_t *xspi_config) {
    // Initialize SPI to 4Mhz, assume a 32Mhz clock
    xnrf_config->ss_port->DIRSET = (1 << xnrf_config->ss_pin);
    xnrf_config->ce_port->DIRSET = (1 << xnrf_config->ce_pin);
    xspi_master_init(xspi_config, SPI_MODE_0_gc, false, SPI_PRESCALER_DIV16_gc, true);

    // Make sure our nRF is powered, disabled, and stabilized per the datasheet for power-on state transition.
    xnrf_disable(xnrf_config);
    _delay_ms(100);
    
    // Set default configuration
    xnrf_write_register(xnrf_config, CONFIG, xnrf_config->confbits);
    
    // Configure address width
    xnrf_set_address_width(xnrf_config, xnrf_config->addr_width);
    
    //TODO: change this to only set default width when pipe is enabled? Does it matter?
    // configure default payload widths for all pipes
    xnrf_write_register(xnrf_config, RX_PW_P0, xnrf_config->payload_width);
    xnrf_write_register(xnrf_config, RX_PW_P1, xnrf_config->payload_width);
    xnrf_write_register(xnrf_config, RX_PW_P2, xnrf_config->payload_width);
    xnrf_write_register(xnrf_config, RX_PW_P3, xnrf_config->payload_width);
    xnrf_write_register(xnrf_config, RX_PW_P4, xnrf_config->payload_width);
    xnrf_write_register(xnrf_config, RX_PW_P5, xnrf_config->payload_width);
    
    // Clear nRF status and FIFOs in case we're coming out of a soft reset
    xnrf_write_register(xnrf_config, NRF_STATUS, ((1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT)));
    xnrf_flush_rx(xnrf_config);
    xnrf_flush_tx(xnrf_config);
}

void xnrf_set_datarate(xnrf_config_t *config, xnrf_datarate_t rate) {
    uint8_t setup = xnrf_read_register(config, RF_SETUP);

    switch (rate) {
        case XNRF_250KBPS:
            setup |= (1 << RF_DR_LOW);
            setup &= ~(1 << RF_DR_HIGH);
            break;
        case XNRF_1MBPS:
            setup &= ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));
            break;
        case XNRF_2MBPS:
            setup &= ~(1 << RF_DR_LOW);
            setup |= (1 << RF_DR_HIGH);
            break;
    }
    xnrf_write_register(config, RF_SETUP, setup);	
}

void xnrf_set_address_width(xnrf_config_t *config, uint8_t width) {
    if (width == 3)
        xnrf_write_register(config, SETUP_AW, 1);
    else if (width == 4)
        xnrf_write_register(config, SETUP_AW, 2);
    else
        xnrf_write_register(config, SETUP_AW, 3);
}