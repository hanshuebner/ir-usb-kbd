/*
  LUFA Library
  Copyright (C) Dean Camera, 2010.
              
  dean [at] fourwalledcubicle [dot] com
  www.fourwalledcubicle.com
*/

/*
  Copyright 2010  Denver Gingerich (denver [at] ossguy [dot] com)
  Based on code by Dean Camera (dean [at] fourwalledcubicle [dot] com)

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
 *  Main source file for the Keyboard demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "IRKeyboard.h"

uint8_t translationTable[256] = {
  /*          0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F */
  /* 0 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 
  /* 1 */  0x00, 0x00, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x0F, 
  /* 2 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  /* 3 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 
  /* 4 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 
  /* 5 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x0C, 
  /* 6 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 
  /* 7 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE5, 0x00, 0x22, 
  /* 8 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 
  /* 9 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x0B, 
  /* A */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x16, 
  /* B */  0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 
  /* C */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  /* D */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 
  /* E */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x08,
  /* F */  0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00
};

/** Buffer to hold the previously generated Keyboard HID report, for comparison purposes inside the HID class driver. */
uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface =
  {
    .Config =
    {
      .InterfaceNumber              = 0,

      .ReportINEndpointNumber       = KEYBOARD_EPNUM,
      .ReportINEndpointSize         = KEYBOARD_EPSIZE,
      .ReportINEndpointDoubleBank   = false,

      .PrevReportINBuffer           = PrevKeyboardHIDReportBuffer,
      .PrevReportINBufferSize       = sizeof(PrevKeyboardHIDReportBuffer),
    },
  };

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware()
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

  /* Hardware Initialization */
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
  if (!(HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface)))
    ;

  USB_Device_EnableSOFEvents();
}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_UnhandledControlRequest(void)
{
  HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
  HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID  Report ID of the received report from the host
 *  \param[in] ReportData  Pointer to a buffer where the created report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo, const uint8_t ReportID,
                                          const void* ReportData, const uint16_t ReportSize)
{
#if 0
  uint8_t  LEDMask   = LEDS_NO_LEDS;
  uint8_t* LEDReport = (uint8_t*)ReportData;

  if (*LEDReport & HID_KEYBOARD_LED_NUMLOCK)
    LEDMask |= LEDS_LED1;
	
  if (*LEDReport & HID_KEYBOARD_LED_CAPSLOCK)
    LEDMask |= LEDS_LED3;

  if (*LEDReport & HID_KEYBOARD_LED_SCROLLLOCK)
    LEDMask |= LEDS_LED4;
	  
  LEDs_SetAllLEDs(LEDMask);
#endif
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
    retval <<= 1;

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
  }
  PORTD &= ~16;

  return retval;
}

uint8_t
read_ir_key(void)
{
  static uint8_t previous;
  uint8_t current = 0;

  PORTD = 0;
  
  // wait for start frame, low > 7ms, high > 3ms

  if (is_high()) {
    goto out;
  }

  PORTD |= 2;
  
  TimerCounter = 7;

  while (is_low())
    ;

  if (TimerCounter) {
    goto out;
  }

  PORTD |= 4;
  
  TimerCounter = 3;
  
  while (is_high())
    ;

  if (TimerCounter) {
    goto out;
  }

  TimerCounter = 100;

  if (read_byte() != 0xff
      || read_byte() != 0x00) {
    goto out;
  }

  {
    uint8_t data = read_byte();
    uint8_t dataNegated = read_byte();

    if ((dataNegated ^ data) != 0xff) {
      goto out;
    }

    current = data;
  }

  if (TimerCounter) {
    previous = current;         /* frame was read, use current database */
  } else {
    current = previous;         /* timed out, repeat previous key code */
  }

 out:
  TimerCounter = 0;
  return current;
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID  Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in] ReportType  Type of the report to create, either REPORT_ITEM_TYPE_In or REPORT_ITEM_TYPE_Feature
 *  \param[out] ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out] ReportSize  Number of bytes written in the report (or zero if no report is to be sent
 *
 *  \return Boolean true to force the sending of the report, false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo, uint8_t* const ReportID,
                                         const uint8_t ReportType, void* ReportData, uint16_t* ReportSize)
{
  USB_KeyboardReport_Data_t* KeyboardReport = (USB_KeyboardReport_Data_t*)ReportData;

  KeyboardReport->Modifier = HID_KEYBOARD_MODIFER_LEFTSHIFT | HID_KEYBOARD_MODIFER_LEFTALT | HID_KEYBOARD_MODIFER_LEFTGUI;
  KeyboardReport->Reserved = 0;
  memset(KeyboardReport->KeyCode, 0, 6);
  KeyboardReport->KeyCode[0] = translationTable[read_ir_key()];
  if (KeyboardReport->KeyCode[0]) {
      PORTD |= 32;
      PORTD &= ~32;
  }
  *ReportSize = sizeof(USB_KeyboardReport_Data_t);

  return false;
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
  SetupHardware();

  PORTD = 0;
	
  for (;;) {

    HID_Device_USBTask(&Keyboard_HID_Interface);
    USB_USBTask();
  }
}

