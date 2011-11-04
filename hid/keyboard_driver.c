/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support  -  ROUSSET  -
 * ----------------------------------------------------------------------------
 * Copyright (c) 2006, Atmel Corporation

 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaiimer below.
 *
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the disclaimer below in the documentation and/or
 * other materials provided with the distribution.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*
$Id: keyboard_driver.c 119 2006-10-17 12:51:47Z jjoannic $
*/

//------------------------------------------------------------------------------
//      Includes
//------------------------------------------------------------------------------

#include "core/common.h"
#include "core/device.h"
#include "core/board.h"
#include "core/trace.h"
#include "core/usb.h"
#include "core/standard.h"
#include "hid.h"
#include "keyboard_driver.h"

//------------------------------------------------------------------------------
//      Global variables
//------------------------------------------------------------------------------

// Descriptors
//------------------------------------------------------------------------------
//! \brief  Report descriptor for the keyboard driver
static const unsigned char pReport[] = {

    HID_GLOBAL_USAGE_PAGE + 1, HID_USAGE_PAGE_GENERIC_DESKTOP,
    HID_LOCAL_USAGE + 1, HID_USAGE_KEYBOARD,
    HID_MAIN_COLLECTION + 1, HID_COLLECTION_APPLICATION,
        HID_GLOBAL_REPORT_SIZE + 1, 1,
        HID_GLOBAL_REPORT_COUNT + 1, 8,
        HID_GLOBAL_USAGE_PAGE + 1, HID_USAGE_PAGE_KEYBOARD_KEYPAD,
        HID_LOCAL_USAGE_MINIMUM + 1, 224,
        HID_LOCAL_USAGE_MAXIMUM + 1, 231,
        HID_GLOBAL_LOGICAL_MINIMUM + 1, 0,
        HID_GLOBAL_LOGICAL_MAXIMUM + 1, 1,
        HID_MAIN_INPUT + 1, HID_VARIABLE,
        HID_GLOBAL_REPORT_COUNT + 1, 3,
        HID_GLOBAL_REPORT_SIZE + 1, 8,
        HID_GLOBAL_LOGICAL_MINIMUM + 1, 0,
        HID_GLOBAL_LOGICAL_MAXIMUM + 1, 101,
        HID_GLOBAL_USAGE_PAGE + 1, HID_USAGE_PAGE_KEYBOARD_KEYPAD,
        HID_LOCAL_USAGE_MINIMUM + 1, 0,
        HID_LOCAL_USAGE_MAXIMUM + 1, 101,
        HID_MAIN_INPUT + 1, HID_DATA,
        HID_GLOBAL_REPORT_COUNT + 1, 5,
        HID_GLOBAL_REPORT_SIZE + 1, 1,
        HID_GLOBAL_USAGE_PAGE + 1, HID_USAGE_PAGE_LEDS,
        HID_LOCAL_USAGE_MINIMUM + 1, 1,
        HID_LOCAL_USAGE_MAXIMUM + 1, 5,
        HID_MAIN_OUTPUT + 1, HID_VARIABLE,
        HID_GLOBAL_REPORT_COUNT + 1, 1,
        HID_GLOBAL_REPORT_SIZE + 1, 3,
        HID_MAIN_OUTPUT + 1, HID_CONSTANT,
    HID_MAIN_ENDCOLLECTION
};

//! \brief  Standard USB device descriptor
//! \see    S_usb_device_descriptor
static const S_usb_device_descriptor sDevice = {

    sizeof(S_usb_device_descriptor), // Size of this descriptor in bytes (18)
    USB_DEVICE_DESCRIPTOR,           // DEVICE Descriptor Type
    USB2_00,                         // USB 2.0 specification
    USB_CLASS_DEVICE,                // Class is specified in the interface descriptor.
    0x00,                            // No device subclass code
    0x00,                            // No device protocol code
    USB_ENDPOINT0_MAXPACKETSIZE,     // Maximum packet size for endpoint zero
    USB_VENDOR_ATMEL,                // ATMEL Vendor ID
    KBD_PRODUCT_ID,                  // Product ID (6201h)
    0x0001,                          // Device release number 0.01
    0x01,                            // Index of manufacturer description
    0x02,                            // Index of product description
    0x03,                            // Index of serial number description
    0x01                             // One possible configuration
};

//! \brief  Device configuration descriptor
//! \see    S_kbd_configuration_descriptor
static const S_kbd_configuration_descriptor sConfiguration = {

    // Standard configuration descriptor
    {
        sizeof(S_usb_configuration_descriptor), // Size of this descriptor in bytes
        USB_CONFIGURATION_DESCRIPTOR,           // CONFIGURATION descriptor type
        sizeof(S_kbd_configuration_descriptor), // Total length of data returned for this configuration
        0x01,                                   // One interface is used by this configuration
        0x01,                                   // Value 0x01 is used to select this configuration
        0x00,                                   // No string is used to describe this configuration
        USB_CONFIG_SELF_NOWAKEUP,               // Device is self-powered and does not support remote wakeup
        USB_POWER_MA(100)                       // maximum power consumption in mA
    },
    // First Interface Descriptor
    {
        sizeof(S_usb_interface_descriptor), // Size of this descriptor in bytes
        USB_INTERFACE_DESCRIPTOR,           // INTERFACE Descriptor Type
        0x00,                               // Interface 0
        0x00,                               // No alternate settings
        2,                                  // Two endpoint used
        USB_CLASS_HID,                      // HID class
        0,                                  // No subclass code
        0,                                  // No protocol code
        0x00                                // No associated string descriptor
    },
    // HID-Specific Descriptor
    {
        sizeof(S_hid_descriptor), // Size of this descriptor in bytes
        HID_DESCRIPTOR,           // HID descriptor type
        HID_1_11,                 // HID Class Specification 0x101
        0x00,                     // CountryCode
        0x01,                     // 1 HID class descriptor
        HID_REPORT_DESCRIPTOR,    // Report descriptor type
        sizeof(pReport)           // Total length of Report descriptor
    },
    // Interrupt IN Endpoint Descriptor
    {
        sizeof(S_usb_endpoint_descriptor),  // Size of this descriptor in bytes
        USB_ENDPOINT_DESCRIPTOR,            // ENDPOINT descriptor type
        KBD_INTERRUPT_IN | USB_ENDPOINT_IN, // IN Endpoint number 01h
        ENDPOINT_TYPE_INTERRUPT,            // Interrupt endpoint
        KBD_INPUT_REPORT_SIZE,              // Maximum packet size is 4 bytes
        10                                  // Interval for polling: 10ms
    },
    // Interrupt OUT Endpoint Descriptor
    {
        sizeof(S_usb_endpoint_descriptor),   // Size of this descriptor in bytes
        USB_ENDPOINT_DESCRIPTOR,             // ENDPOINT descriptor type
        KBD_INTERRUPT_OUT | USB_ENDPOINT_IN, // OUT Endpoint number 02h
        ENDPOINT_TYPE_INTERRUPT,             // Interrupt endpoint
        KBD_OUTPUT_REPORT_SIZE,              // Maximum packet size is 1 byte
        10                                   // Interval for polling: 10ms
    }
};

//! \brief  Language ID string descriptor
static const S_usb_language_id sLanguageID = {

    USB_STRING_DESCRIPTOR_SIZE(1),
    USB_STRING_DESCRIPTOR,
    USB_LANGUAGE_ENGLISH_US
};

//! \brief  Manufacturer string descriptor
static const char pManufacturer[] = {

    USB_STRING_DESCRIPTOR_SIZE(5),
    USB_STRING_DESCRIPTOR,
    USB_UNICODE('A'),
    USB_UNICODE('T'),
    USB_UNICODE('M'),
    USB_UNICODE('E'),
    USB_UNICODE('L')
};

//! \brief  Product string descriptor
static const char pProduct[] = {

    USB_STRING_DESCRIPTOR_SIZE(23),
    USB_STRING_DESCRIPTOR,
    USB_UNICODE('A'),
    USB_UNICODE('T'),
    USB_UNICODE('M'),
    USB_UNICODE('E'),
    USB_UNICODE('L'),
    USB_UNICODE(' '),
    USB_UNICODE('A'),
    USB_UNICODE('T'),
    USB_UNICODE('9'),
    USB_UNICODE('1'),
    USB_UNICODE(' '),
    USB_UNICODE('H'),
    USB_UNICODE('I'),
    USB_UNICODE('D'),
    USB_UNICODE(' '),
    USB_UNICODE('K'),
    USB_UNICODE('E'),
    USB_UNICODE('Y'),
    USB_UNICODE('B'),
    USB_UNICODE('O'),
    USB_UNICODE('A'),
    USB_UNICODE('R'),
    USB_UNICODE('D')
};

//! \brief  Serial number string descriptor
static const char pSerialNumber[] = {

    USB_STRING_DESCRIPTOR_SIZE(12),
    USB_STRING_DESCRIPTOR,
    USB_UNICODE('0'),
    USB_UNICODE('1'),
    USB_UNICODE('2'),
    USB_UNICODE('3'),
    USB_UNICODE('4'),
    USB_UNICODE('5'),
    USB_UNICODE('6'),
    USB_UNICODE('7'),
    USB_UNICODE('8'),
    USB_UNICODE('9'),
    USB_UNICODE('A'),
    USB_UNICODE('F')
};

//! \brief  List of string descriptors
static const char *pStrings[] = {

    (char *) &sLanguageID,
    pManufacturer,
    pProduct,
    pSerialNumber
};

//! \brief  List of endpoint descriptors
static const S_usb_endpoint_descriptor *pEndpoints[] = {

    &(sConfiguration.sInterruptIn),
    &(sConfiguration.sInterruptOut)
};

//! \brief  Standard descriptors list
static const S_std_descriptors sDescriptors = {

    &sDevice,
    (S_usb_configuration_descriptor *) &sConfiguration,
    pStrings,
    pEndpoints
};

//------------------------------------------------------------------------------
//      Internal functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//! \brief  Invokes the "report received" callback if it is set.
//! \param  pKbd Pointer to a S_kbd instance
//------------------------------------------------------------------------------
static void KBD_ReportReceived(const S_kbd *pKbd)
{
    if (pKbd->fReportReceivedCallback != 0) {

        pKbd->fReportReceivedCallback(0, 0, 0, 0);
    }

    // Send a ZLP to the host
    USB_SendZLP0(pKbd->sClass.pUsb, 0, 0);
}

//------------------------------------------------------------------------------
//      Exported functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//! \brief  Handles standard and class-specific SETUP requests.
//!
//! \param  pKbd Pointer to a S_kbd instance
//! \see    S_kbd
//------------------------------------------------------------------------------
void KBD_RequestHandler(S_kbd *pKbd)
{
    const S_usb   *pUsb = pKbd->sClass.pUsb;
    S_usb_request *pSetup = USB_GetSetup(pUsb);

    TRACE_DEBUG_M("NewReq ");

    // Check request type
    if (USB_REQUEST_TYPE(pSetup) == USB_STANDARD_REQUEST) {

        // Handle standard requests
        switch (pSetup->bRequest) {
        //----------------------
        case USB_GET_DESCRIPTOR:
        //----------------------
            // Identify requested descriptor
            switch (HBYTE(pSetup->wValue)) {
            //-------------------------
            case HID_REPORT_DESCRIPTOR:
            //-------------------------
                // Send report descriptor
                TRACE_DEBUG_M("Report ");
                USB_Write(pUsb,
                          0,
                          pReport,
                          min(sizeof(pReport), pSetup->wLength),
                          0,
                          0);
                break;

            //------------------
            case HID_DESCRIPTOR:
            //------------------
                // Send HID descriptor
                TRACE_DEBUG_M("HID ");

                USB_Write(pUsb,
                          0,
                          (void *) &sConfiguration.sHID,
                          min(sizeof(S_hid_descriptor), pSetup->wLength),
                          0,
                          0);
                break;

            //------
            default:
            //------
                // Forward request to standard handler
                STD_RequestHandler(&(pKbd->sClass));
                break;
            }
            break;

        //------
        default:
        //------
            // Forward request to standard handler
            STD_RequestHandler(&(pKbd->sClass));
            break;
        }
    }
    else {

        // Handle class-specific requests
        switch (pSetup->bRequest) {
        //----------------
        case HID_GET_IDLE:
        //----------------
            // Return current Idle rate
            TRACE_DEBUG_M("gIdle ");

            USB_Write(pUsb, 0, &(pKbd->bIdleRate), 1, 0, 0);
            break;

        //----------------
        case HID_SET_IDLE:
        //----------------
            // Get new Idle rate
            pKbd->bIdleRate = HBYTE(pSetup->wValue);
            TRACE_DEBUG_M("sIdle(%d) ", pKbd->bIdleRate);

            USB_SendZLP0(pUsb, 0, 0);
            break;

        //------------------
        case HID_GET_REPORT:
        //------------------
            // Send the current report to the host
            TRACE_DEBUG_M("gReport ");

            // Check report type
            if (HBYTE(pSetup->wValue) == HID_INPUT_REPORT) {

                // Send report
                TRACE_DEBUG_M("In ");
                USB_Write(pUsb, 0, &(pKbd->sInputReport),
                          KBD_INPUT_REPORT_SIZE, 0, 0);
            }
            else if (HBYTE(pSetup->wValue) == HID_OUTPUT_REPORT) {

                // Send report
                TRACE_DEBUG_M("Out ");
                USB_Write(pUsb, 0, &(pKbd->sOutputReport),
                          KBD_OUTPUT_REPORT_SIZE, 0, 0);
            }
            else {

                // STALL endpoint 0
                USB_Stall(pUsb, 0);
            }
            break;

        //------------------
        case HID_SET_REPORT:
        //------------------
            // Retrieve new report
            TRACE_DEBUG_M("sReport ");

            // Check report type
            if (HBYTE(pSetup->wValue) == HID_OUTPUT_REPORT) {

                // Retrieve report
                USB_Read(pUsb, 0, &(pKbd->sOutputReport), KBD_OUTPUT_REPORT_SIZE,
                         (Callback_f) KBD_ReportReceived, pKbd);
            }
            else {

                // STALL endpoint 0
                USB_Stall(pUsb, 0);
            }
            break;

        //------
        default:
        //------
            // STALL endpoint 0
            TRACE_WARNING("W: KBD_RequestHandler: Unknown request 0x%02X\n\r",
                          pSetup->bRequest);
            USB_Stall(pUsb, 0);
        }
    }
}

//------------------------------------------------------------------------------
//! \brief  Initializes a HID keyboard driver
//!
//!         This method sets the standard descriptors of the device and the
//!         default HID keyboard configuration.
//! \param  pKbd      Pointer to a S_kbd instance
//! \param  pUsb      Pointer to the S_usb driver instance to use
//! \param  fCallback Callback function to invoke when a new report is received
//! \see    S_kbd
//! \see    S_usb
//------------------------------------------------------------------------------
void KBD_Init(S_kbd *pKbd, const S_usb *pUsb, Callback_f fCallback)
{
    // Initialize mouse driver
    pKbd->bIdleRate = 0;
    pKbd->sInputReport.bmModifierKeys = 0;
    pKbd->sInputReport.pPressedKeys[0] = 0;
    pKbd->sInputReport.pPressedKeys[1] = 0;
    pKbd->sInputReport.pPressedKeys[2] = 0;
    pKbd->sOutputReport.bmLeds = 0;
    pKbd->fReportReceivedCallback = fCallback;

    // Initialize standard class attributes
    pKbd->sClass.pUsb = pUsb;
    pKbd->sClass.pDescriptors = &sDescriptors;

    // Initialize the USB driver
    USB_Init(pUsb);
}

//------------------------------------------------------------------------------
//! \brief  Sends the current mouse report to the host, through the interrupt IN
//!         endpoint
//! \param  pKbd      Pointer to a S_kbd instance
//! \param  fCallback Optional callback function to invoke when the report is
//!                   sent
//! \param  pArgument Optional argument to pass to the callback function
//------------------------------------------------------------------------------
unsigned char KBD_SendReport(const S_kbd *pKbd,
                             Callback_f  fCallback,
                             void        *pArgument)
{
    return USB_Write(pKbd->sClass.pUsb, KBD_INTERRUPT_IN, &(pKbd->sInputReport),
                     KBD_INPUT_REPORT_SIZE, fCallback, pArgument);
}

//------------------------------------------------------------------------------
//! \brief  Waits for a new report from the host to be received.
//! \param  pKbd      Pointer to a S_kbd instance
//! \param  fCallback Optional callback function to invoke when the report is
//!                   received
//! \param  pArgument Optional argument to pass to the callback function
//------------------------------------------------------------------------------
unsigned char KBD_ReceiveReport(S_kbd *pKbd,
                                Callback_f  fCallback,
                                void        *pArgument)
{
    return USB_Read(pKbd->sClass.pUsb, KBD_INTERRUPT_OUT, &(pKbd->sOutputReport),
                    KBD_OUTPUT_REPORT_SIZE, fCallback, pArgument);
}
