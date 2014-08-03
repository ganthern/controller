/* Copyright (C) 2014 by Jacob Alexander
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// ----- Includes -----

// Compiler Includes
#include <Lib/OutputLib.h>

// Project Includes
#include <cli.h>
#include <print.h>
#include <scan_loop.h>

// USB Includes
#if defined(_at90usb162_) || defined(_atmega32u4_) || defined(_at90usb646_) || defined(_at90usb1286_)
#elif defined(_mk20dx128_) || defined(_mk20dx128vlf5_) || defined(_mk20dx256_)
#include "arm/uart_serial.h"
#endif

// Local Includes
#include "output_com.h"



// ----- Function Declarations -----

void cliFunc_kbdProtocol( char* args );
void cliFunc_readLEDs   ( char* args );
void cliFunc_sendKeys   ( char* args );
void cliFunc_setKeys    ( char* args );
void cliFunc_setMod     ( char* args );


// ----- Variables -----

// Output Module command dictionary
char*       outputCLIDictName = "USB Module Commands - NOT WORKING";
CLIDictItem outputCLIDict[] = {
	{ "kbdProtocol", "Keyboard Protocol Mode: 0 - Boot, 1 - OS/NKRO Mode", cliFunc_kbdProtocol },
	{ "readLEDs",    "Read LED byte:" NL "\t\t1 NumLck, 2 CapsLck, 4 ScrlLck, 16 Kana, etc.", cliFunc_readLEDs },
	{ "sendKeys",    "Send the prepared list of USB codes and modifier byte.", cliFunc_sendKeys },
	{ "setKeys",     "Prepare a space separated list of USB codes (decimal). Waits until \033[35msendKeys\033[0m.", cliFunc_setKeys },
	{ "setMod",      "Set the modfier byte:" NL "\t\t1 LCtrl, 2 LShft, 4 LAlt, 8 LGUI, 16 RCtrl, 32 RShft, 64 RAlt, 128 RGUI", cliFunc_setMod },
	{ 0, 0, 0 } // Null entry for dictionary end
};


// Which modifier keys are currently pressed
// 1=left ctrl,    2=left shift,   4=left alt,    8=left gui
// 16=right ctrl, 32=right shift, 64=right alt, 128=right gui
         uint8_t USBKeys_Modifiers    = 0;
         uint8_t USBKeys_ModifiersCLI = 0; // Separate CLI send buffer

// Currently pressed keys, max is defined by USB_MAX_KEY_SEND
         uint8_t USBKeys_Array   [USB_MAX_KEY_SEND];
         uint8_t USBKeys_ArrayCLI[USB_MAX_KEY_SEND]; // Separate CLI send buffer

// The number of keys sent to the usb in the array
         uint8_t USBKeys_Sent    = 0;
         uint8_t USBKeys_SentCLI = 0;

// 1=num lock, 2=caps lock, 4=scroll lock, 8=compose, 16=kana
volatile uint8_t USBKeys_LEDs = 0;

// Protocol setting from the host.
// 0 - Boot Mode (Default, until set by the host)
// 1 - NKRO Mode
volatile uint8_t USBKeys_Protocol = 1;

// the idle configuration, how often we send the report to the
// host (ms * 4) even when it hasn't changed
         uint8_t USBKeys_Idle_Config = 125;

// count until idle timeout
         uint8_t USBKeys_Idle_Count = 0;


// ----- Functions -----

// USB Module Setup
inline void Output_setup()
{
	// Setup UART
	uart_serial_setup();

	// Register USB Output CLI dictionary
	CLI_registerDictionary( outputCLIDict, outputCLIDictName );
}


// USB Data Send
inline void Output_send(void)
{
	// TODO
}


// Sets the device into firmware reload mode
inline void Output_firmwareReload()
{
	uart_device_reload();
}


// USB Input buffer available
inline unsigned int Output_availablechar()
{
	return uart_serial_available();
}


// USB Get Character from input buffer
inline int Output_getchar()
{
	// XXX Make sure to check output_availablechar() first! Information is lost with the cast (error codes) (AVR)
	return (int)uart_serial_getchar();
}


// USB Send Character to output buffer
inline int Output_putchar( char c )
{
	return uart_serial_putchar( c );
}


// USB Send String to output buffer, null terminated
inline int Output_putstr( char* str )
{
#if defined(_at90usb162_) || defined(_atmega32u4_) || defined(_at90usb646_) || defined(_at90usb1286_) // AVR
	uint16_t count = 0;
#elif defined(_mk20dx128_) || defined(_mk20dx128vlf5_) || defined(_mk20dx256_) // ARM
	uint32_t count = 0;
#endif
	// Count characters until NULL character, then send the amount counted
	while ( str[count] != '\0' )
		count++;

	return uart_serial_write( str, count );
}


// Soft Chip Reset
inline void Output_softReset()
{
	usb_device_software_reset();
}


// ----- CLI Command Functions -----

void cliFunc_kbdProtocol( char* args )
{
	print( NL );
	info_msg("Keyboard Protocol: ");
	printInt8( USBKeys_Protocol );
}


void cliFunc_readLEDs( char* args )
{
	print( NL );
	info_msg("LED State (This doesn't work yet...): ");
	printInt8( USBKeys_LEDs );
}


void cliFunc_sendKeys( char* args )
{
	// Copy USBKeys_ArrayCLI to USBKeys_Array
	for ( uint8_t key = 0; key < USBKeys_SentCLI; ++key )
	{
		USBKeys_Array[key] = USBKeys_ArrayCLI[key];
	}
	USBKeys_Sent = USBKeys_SentCLI;

	// Set modifier byte
	USBKeys_Modifiers = USBKeys_ModifiersCLI;
}


void cliFunc_setKeys( char* args )
{
	char* curArgs;
	char* arg1Ptr;
	char* arg2Ptr = args;

	// Parse up to USBKeys_MaxSize args (whichever is least)
	for ( USBKeys_SentCLI = 0; USBKeys_SentCLI < USBKeys_MaxSize; ++USBKeys_SentCLI )
	{
		curArgs = arg2Ptr;
		CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

		// Stop processing args if no more are found
		if ( *arg1Ptr == '\0' )
			break;

		// Add the USB code to be sent
		USBKeys_ArrayCLI[USBKeys_SentCLI] = decToInt( arg1Ptr );
	}
}


void cliFunc_setMod( char* args )
{
	// Parse number from argument
	//  NOTE: Only first argument is used
	char* arg1Ptr;
	char* arg2Ptr;
	CLI_argumentIsolation( args, &arg1Ptr, &arg2Ptr );

	USBKeys_ModifiersCLI = decToInt( arg1Ptr );
}
