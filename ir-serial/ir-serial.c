/*
  LUFA Library
  Copyright (C) Dean Camera, 2010.
              
  dean [at] fourwalledcubicle [dot] com
  www.fourwalledcubicle.com
*/

/*
  Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this 
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in 
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting 
  documentation, and that the name of the author not be used in 
  advertising or publicity pertaining to distribution of the 
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the VirtualSerial demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */
 
#include "ir-serial.h"

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
  {
    .Config =
    {
      .ControlInterfaceNumber         = 0,

      .DataINEndpointNumber           = CDC_TX_EPNUM,
      .DataINEndpointSize             = CDC_TXRX_EPSIZE,
      .DataINEndpointDoubleBank       = false,

      .DataOUTEndpointNumber          = CDC_RX_EPNUM,
      .DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
      .DataOUTEndpointDoubleBank      = false,

      .NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
      .NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
      .NotificationEndpointDoubleBank = false,
    },
  };

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs
 */
static FILE USBSerialStream;

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* Disable clock division */
  clock_prescale_set(clock_div_1);

  /* Millisecond timer initialization, with output compare interrupt enabled for the idle timing */
  OCR0A  = 250;
  TCCR0A = (1 << WGM01);
  TCCR0B = ((1 << CS01) | (1 << CS00));
  TIMSK0 = (1 << OCIE0A);

  /* Set up debug port */
  DDRD = 0xff;

  USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  if (!(CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface)))
    ;
}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_UnhandledControlRequest(void)
{
  CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** ISR for the timer 0 compare vector. This ISR fires once each millisecond, and increments the
 *  scheduler elapsed idle period counter when the host has set an idle period.
 */
static volatile uint8_t TimerCounter = 0;

ISR(TIMER0_COMPA_vect, ISR_BLOCK)
{
  if (TimerCounter) {
    TimerCounter--;
  }
}

static inline bool
is_low(void)
{
  return (PINB & 2) == 0;
}

static inline bool
is_high(void)
{
  return (PINB & 2) == 2;
}

static inline uint8_t
read_byte(void)
{
  uint8_t retval = 0;

  for (uint8_t i = 0; i < 8; i++) {
    PORTD &= ~16;

    while (TimerCounter && is_low())
      ;

    _delay_us(800);
    PORTD |= 16;
    
    if (is_low()) {
      retval |= 1;
    } else {
      while (TimerCounter && is_high())
        ;
    }
    retval <<= 1;
  }
  PORTD &= ~16;

  return retval;
}

enum { none, new, repeat }
read_frame(uint8_t bytes[4])
{
  PORTD = 0;
  
  // wait for start frame, low > 7ms, high > 3ms

  if (is_high())
    return none;

  PORTD |= 2;
  
  TimerCounter = 7;

  while (is_low())
    ;

  if (TimerCounter)
    return none;

  PORTD |= 4;
  
  TimerCounter = 3;
  
  while (is_high())
    ;

  if (TimerCounter)
    return none;

  PORTD |= 8;

  TimerCounter = 100;

  bytes[0] = read_byte();
  bytes[1] = read_byte();
  bytes[2] = read_byte();
  bytes[3] = read_byte();

  if (TimerCounter) {
    return new;
  } else {
    return repeat;
  }
}

void
process_key(uint8_t data[4])
{
  fprintf(&USBSerialStream, "%02x %02x %02x %02x\r\n", data[0], data[1], data[2], data[3]);
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
  SetupHardware();
	
  /* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
  CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

  PORTD = 0;
	
  for (;;) {
    static uint8_t previous[4];
    uint8_t buf[4];
    switch (read_frame(buf)) {
    case none:
      break;
    case repeat:
      memcpy(buf, previous, 4);
    case new:
      process_key(buf);
      memcpy(previous, buf, 4);
    }

    /* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
    while (CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface))
      CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

    CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    USB_USBTask();
  }
}

