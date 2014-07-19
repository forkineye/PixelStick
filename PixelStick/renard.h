/*
 * renard.h
 *
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


#ifndef RENARD_H_
#define RENARD_H_

// Special Renard bytes
#define RENARD_PAD      0x7D    // pad byte to be discarded
#define	RENARD_SYNC     0x7E    // sync byte to reset state machine
#define RENARD_ESCAPE   0X7F    // escape byte
#define	RENARD_ADDR     0x80    // command / address byte. 0x80 designates packet for this device
#define RENARD_ESC_7D   0x2F    // encoded value for data byte of 7D
#define RENARD_ESC_7E   0x30    // encoded value for data byte of 7E
#define RENARD_ESC_7F   0x31    // encoded value for data byte of 7F

// Renard state machine flags
typedef enum {
    RENSTATE_SYNC,
    RENSTATE_ESCAPE,
    RENSTATE_ADDR,
    RENSTATE_NULL
} renstate_t;

#endif /* RENARD_H_ */