/* Copyright (C) 2014-2017 by Jacob Alexander
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
#include <Lib/ScanLib.h>

// Project Includes
#include <cli.h>
#include <connect_scan.h>
#include <lcd_scan.h>
#include <led.h>
#include <led_scan.h>
#include <print.h>
#include <matrix_scan.h>
#include <macro.h>
#include <output_com.h>
#include <pixel.h>

// Local Includes
#include "scan_loop.h"



// ----- Variables -----

// ----- Functions -----

// Setup
inline void Scan_setup()
{
	// Setup UART Connect, if Output_Available, this is the master node
	Connect_setup( Output_Available, 1 );

	// Setup GPIO pins for matrix scanning
	Matrix_setup();

	// Setup ISSI chip to control the leds
	LED_setup();

	// Setup the ST/NHD lcd display
	LCD_setup();

	// Setup Pixel Map
	Pixel_setup();

	// Start Matrix Scanner
	Matrix_start();
}


// Main Poll Loop
// This is for operations that need to be run as often as possible
// Usually reserved for LED update routines and other things that need quick update rates
void Scan_poll()
{
	// Prepare any LED events
	Pixel_process();

	// Process any LED events
	LED_scan();

	// Process any LCD events
	LCD_scan();
}


// Main Periodic Scan
// This function is called periodically at a constant rate
// Useful for matrix scanning and anything that requires consistent attention
uint8_t Scan_periodic()
{
	// Process any interconnect commands
	Connect_scan();

	// Scan Matrix
	return Matrix_single_scan();
}


// Signal from Macro Module that all keys have been processed (that it knows about)
inline void Scan_finishedWithMacro( uint8_t sentKeys )
{
}


// Signal from Output Module that all keys have been processed (that it knows about)
inline void Scan_finishedWithOutput( uint8_t sentKeys )
{
}


// -------- Capabilities ---------
uint8_t state_map[100] = {0};
uint8_t current_height = 0;

// call on press/hold event
// returns >0 if we're on top, 0 when somehing else has a higher ID 
uint8_t register_ID(uint8_t ID) { 
	if(state_map[ID] == 0){ // not registered. we're a new pressed key
		current_height = current_height + 1;
		state_map[ID] = current_height;
		return current_height;
	} else { // already registered
		if(state_map[ID] >= current_height) {
			current_height = state_map[ID];
			return current_height;
		} else {
			return 0;
		}
	}
}

void unregister_ID(uint8_t ID) {
	if(state_map[ID] >= current_height) {
		current_height = 0; // some other held key is on top now
	}
	state_map[ID] = 0;
}

// 0x00 LSHIFT | 0x01 RSHIFT | 0x02 CAPS | 0x03 ALTGR
uint8_t mod_filter[4] = {0x00}; // actual state of the mod keys
uint8_t mod_toggles[4] = {0x00}; // toggle states
uint8_t mod_states[4] = {0x00}; // what the last shiftedKey sent out
uint8_t mod_codes[4] = {0xE1, 0xE5, 0x39, 0xE6}; // LSFT RSFT CAPS RALT

#define STATE_NONE 0x00
#define STATE_PRESS 0x01
#define STATE_HOLD 0x02
#define STATE_RELEASE 0x03

#define LSHIFT_INDEX 0
#define RSHIFT_INDEX 1
#define CAPS_INDEX 2
#define ALTGR_INDEX 3

#define LSHIFT_CODE mod_codes[0]
#define RSHIFT_CODE mod_codes[1]
#define CAPS_CODE mod_codes[2]
#define ALTGR_CODE mod_codes[3]

uint8_t is_shifted(){
	uint8_t lshift = mod_filter[LSHIFT_INDEX];
	uint8_t rshift = mod_filter[RSHIFT_INDEX];
	uint8_t caps = (mod_toggles[CAPS_INDEX] != 0);
	return (lshift || rshift) ^ caps;
}

void CustomAction_filterMod(TriggerMacro *trigger, uint8_t state, uint8_t stateType, uint8_t *args) {
	extern void Output_usbCodeSend_capability(TriggerMacro *trigger, uint8_t state, uint8_t stateType, uint8_t *args);
	CapabilityState cstate = KLL_CapabilityState(state, stateType);
	uint8_t key = args[0];

	//0x01 press, 0x02 hold, 0x03 release
	switch(cstate) {
		case CapabilityState_Initial: //Press
			mod_filter[key] = 0x01;
			if(current_height == 0 && mod_states[key] == 0x00){
				mod_states[key] = 0x01;
				mod_toggles[key] = (mod_toggles[key] + 1) % 4;
				Output_usbCodeSend_capability(trigger, STATE_PRESS, stateType, &mod_codes[key]);
			}
			break;
		case CapabilityState_Any: //Hold
			if(current_height == 0 && mod_states[key] != 0x00){
				Output_usbCodeSend_capability(trigger, STATE_HOLD, stateType, &mod_codes[key]);
			}
			break;
		case CapabilityState_Last: //Release
			mod_filter[key] = 0x00;
			if(current_height == 0 && mod_states[key] != 0x00){
				mod_states[key] = 0x00;
				mod_toggles[key] = (mod_toggles[key] + 1) % 4;
				Output_usbCodeSend_capability(trigger, STATE_RELEASE, stateType, &mod_codes[key]);
			}
			break;
		default:
			return;
	}
}

void CustomAction_shiftedKey(TriggerMacro *trigger, uint8_t state, uint8_t stateType, uint8_t *args) {
	extern void Output_usbCodeSend_capability(TriggerMacro *trigger, uint8_t state, uint8_t stateType, uint8_t *args);
	CapabilityState cstate = KLL_CapabilityState(state, stateType);

	// get values from args
	uint8_t shift;
	uint8_t altgr;
	uint8_t key;
	uint8_t id = args[4];
	if(is_shifted()){
		shift = args[2] & 0x01;
		altgr = args[2] & 0x02;
		key = args[3];
	} else {
		shift = args[0] & 0x01;
		altgr = args[0] & 0x02;
		key = args[1];
	}
	
	switch(cstate) {
		case CapabilityState_Initial: //Press
			register_ID(id); //sure to be >0
			if(shift == 0x00 && mod_states[LSHIFT_INDEX] != 0x00){ //lshift was pressed but shouldnt
				mod_states[LSHIFT_INDEX] = 0x00;
				Output_usbCodeSend_capability(trigger, STATE_RELEASE, stateType, &LSHIFT_CODE);
			}
			if(shift == 0x00 && mod_states[RSHIFT_INDEX] != 0x00){ //rshift was pressed but shouldnt
				mod_states[RSHIFT_INDEX] = 0x00;
				Output_usbCodeSend_capability(trigger, STATE_RELEASE, stateType, &RSHIFT_CODE);
			}
			if(altgr == 0x00 && mod_states[ALTGR_INDEX] != 0x00){ //altgr was pressed but shouldnt
				mod_states[ALTGR_INDEX] = 0x00;
				Output_usbCodeSend_capability(trigger, STATE_RELEASE, stateType, &ALTGR_CODE);
			}
			if(shift != 0x00 && mod_states[LSHIFT_INDEX] == 0x00){ // shift should be pressed
				mod_states[LSHIFT_INDEX] = 0x01;
				Output_usbCodeSend_capability(trigger, STATE_PRESS, stateType, &LSHIFT_CODE);
				if(mod_filter[RSHIFT_INDEX] != 0x00){
					mod_states[RSHIFT_INDEX] = 0x01;
					Output_usbCodeSend_capability(trigger, STATE_PRESS, stateType, &RSHIFT_CODE);
				}
			}
			if(altgr != 0x00 && mod_states[ALTGR_INDEX] == 0x00){ //altgr should be pressed
				mod_states[ALTGR_INDEX] = 0x01;
				Output_usbCodeSend_capability(trigger, STATE_PRESS, stateType, &ALTGR_CODE);
			}
			break;
		case CapabilityState_Any: //Hold
			if(register_ID(id) != 0) { // we control the mods
				if(shift != 0x00) {
					if(mod_states[LSHIFT_INDEX] != 0x00) {
						Output_usbCodeSend_capability(trigger, STATE_HOLD, stateType, &LSHIFT_CODE);
					} else {
						Output_usbCodeSend_capability(trigger, STATE_PRESS, stateType, &LSHIFT_CODE);
					}
					if(mod_filter[RSHIFT_INDEX] != 0x00){ // only use rshift if its actually pressed
						if(mod_states[RSHIFT_INDEX] != 0x00) {
							Output_usbCodeSend_capability(trigger, STATE_HOLD, stateType, &RSHIFT_CODE);
						} else {
							Output_usbCodeSend_capability(trigger, STATE_PRESS, stateType, &RSHIFT_CODE);
						}
					}
				} else {
					if(mod_states[LSHIFT_INDEX] != 0x00) {
						Output_usbCodeSend_capability(trigger, STATE_RELEASE, stateType, &LSHIFT_CODE);
					}
				}

				if(altgr != 0x00) {
					if(mod_states[ALTGR_INDEX] != 0x00) {
						Output_usbCodeSend_capability(trigger, STATE_HOLD, stateType, &ALTGR_CODE);
					} else {
						Output_usbCodeSend_capability(trigger, STATE_PRESS, stateType, &ALTGR_CODE);
					}
				} else {
					if(mod_states[ALTGR_INDEX] != 0x00) {
						Output_usbCodeSend_capability(trigger, STATE_RELEASE, stateType, &ALTGR_CODE);
					}
				}
			}
			break;
		case CapabilityState_Last: //Release
			if(register_ID(id) != 0){
				if(shift != 0x00){ // if we pressed shift, reöease them
					if(mod_states[LSHIFT_INDEX] != 0x00) {
						mod_states[LSHIFT_INDEX] = 0x00;
						Output_usbCodeSend_capability(trigger, STATE_RELEASE, stateType, &LSHIFT_CODE);
					}

					if(mod_states[RSHIFT_INDEX] != 0x00) {
						mod_states[RSHIFT_INDEX] = 0x00;
						Output_usbCodeSend_capability(trigger, STATE_RELEASE, stateType, &RSHIFT_CODE);
					}
				}

				if(altgr != 0x00 && mod_states[ALTGR_INDEX] != 0x00) {
					mod_states[ALTGR_INDEX] = 0x00;
					Output_usbCodeSend_capability(trigger, STATE_RELEASE, stateType, &ALTGR_CODE);
				}
			}
			unregister_ID(id);
		default:
			break;
	}

	Output_usbCodeSend_capability(trigger, state, stateType, &key);
}

// Signal from the Output Module that the available current has changed
// current - mA
void Scan_currentChange( unsigned int current )
{
	// Indicate to all submodules current change
	Connect_currentChange( current );
	Matrix_currentChange( current );
	LED_currentChange( current );
	LCD_currentChange( current );
}

