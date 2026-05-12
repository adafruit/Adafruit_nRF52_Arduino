/*
  NFC class for nRF52.
  Written by Chiara Ruggeri (chiara@arduino.org)
  
  Copyright (c) 2016 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Enjoy!  
*/


#ifndef NFC_h
#define NFC_h

#include "Arduino.h"
#include "nrf_clock.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "cortex-m4/nfc_t2t_lib.h"
#include "ndef/nfc_uri_msg.h"
#include "ndef/nfc_text_rec.h"
#include "ndef/nfc_launchapp_msg.h"
#include "ndef/nfc_ble_pair_msg.h"
#ifdef __cplusplus
}
#endif


class NFCClass{

	public:
		
		/**
		* @brief
		* Name:
		*			setTXTmessage
		* Description:
		*			Specify a text message that will pop up on a device when it is
		*			near to the board.
		* Arguments:
		*			-TXTMessage: array of char containing the message
		*			-language: array of char containing the message's language code
		*/
		void setTXTmessage(const char TXTMessage[], const char language[]);
	
	
		/**
		* @brief
		* Name:
		*			setURImessage
		* Description:
		*			Specify an URI message that will pop up on a device when it is
		*			near to the board.
		* Arguments:
		*			-URL: address to the resource to reach
		*			-type: type of the URI message (see documentation for details)
		*/
		void setURImessage( const char URL[], nfc_uri_id_t type);
	
	
		/**
		* @brief
		* Name:
		*			setAPPmessage
		* Description:
		*			Specify an application that will try to be open on a device when
		*			it is near to the board.
		* Arguments:
		*			-android_app: package of the Android application
		*			-windows_app: ID of the Windows application
		*/
		void setAPPmessage(const char android_app[], const char windows_app[]);
	
	
		/**
		* @brief
		* Name:
		*			setOobPairingKey
		* Description:
		*			Set a message for pairing the board with another BLE device.
		*			This function is part of OOB bond procedure and should be
		*			used with BLE library.
		*/
//		void setOobPairingKey(void);
	
	
		/**
		* @brief
		* Name:
		*			start
		* Description:
		*			Start the NFC module.
		*/
		void start(void);
		
		
		/**
		* @brief
		* Name:
		*			stop
		* Description:
		*			Stop the NFC module.
		*/
		void stop(void);
		
		
		/**
		* @brief
		* Name:
		*			registerCallback
		* Description:
		*			Attach a function passed by the user to the "field detected" event.
		* Argument:
		*			-function: function to be called when event happens
		*/
		void registerCallback(void(*function)(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t dataLength));
		
		
		/**
		* @brief
		* Name:
		*			onService
		* Description:
		*			Service function called by ISR.
		*/
		void onService(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t dataLength);
		
	private:
		// Callback user function
		void (*Callback)(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t dataLength);
		
};

extern NFCClass NFC;

#endif //NFC_h