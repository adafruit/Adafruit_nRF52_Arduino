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


#include "NFC.h"
#include <Arduino.h>
void nfc_callback(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t dataLength)
{
	NFC.onService(context,event,data,dataLength);
  switch (event) {
	case NFC_T2T_EVENT_FIELD_ON:
		pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, HIGH);
		break;
	default:
		break;
  }
}


uint8_t ndef_msg_buf[256];

void NFCClass::setTXTmessage(const char TXTMessage[], const char language[]){

	nfc_t2t_setup(nfc_callback, NULL);
	uint32_t len = sizeof(ndef_msg_buf);
	uint8_t sizeM=strlen(TXTMessage);
	uint8_t sizeL=strlen(language);	

	NFC_NDEF_MSG_DEF(welcome_msg, 1);	
    NFC_NDEF_TEXT_RECORD_DESC_DEF(en_text_rec,
                                  UTF_8,
                                  (uint8_t *)language,
                                  sizeL,
                                  (uint8_t *)TXTMessage,
                                  sizeM);
	nfc_ndef_msg_record_add( &NFC_NDEF_MSG(welcome_msg),
                                        &NFC_NDEF_TEXT_RECORD_DESC(en_text_rec));
		
	nfc_ndef_msg_encode(&NFC_NDEF_MSG(welcome_msg),
                                            ndef_msg_buf,
                                            &len);
	 
	nfc_t2t_payload_set((uint8_t *) ndef_msg_buf, len);
  //Serial.println("Send");
}

void NFCClass::setURImessage( const char URL[], nfc_uri_id_t type){
	
	uint8_t size=strlen(URL);
	memset(ndef_msg_buf, 0, 256);
	
	uint32_t len = sizeof(ndef_msg_buf);
	nfc_t2t_setup(nfc_callback, NULL);
	nfc_uri_msg_encode( type,
                         (uint8_t *) URL,
                         size,
                         ndef_msg_buf,
                         &len);
								   
	nfc_t2t_payload_set((uint8_t*)ndef_msg_buf, len);
	
}

void NFCClass::setAPPmessage(const char android_app[], const char windows_app[]){
	
	uint8_t sizeA=strlen(android_app);
	uint8_t sizeW=strlen(windows_app);
	uint32_t len = sizeof(ndef_msg_buf);
	
	nfc_t2t_setup(nfc_callback, NULL);
	nfc_launchapp_msg_encode((uint8_t *)android_app,
                                    sizeA,
                                    (uint8_t *)windows_app,
                                    sizeW,
                                    ndef_msg_buf,
                                    &len);
  
	nfc_t2t_payload_set((uint8_t *) ndef_msg_buf, len);
}

// void NFCClass::setOobPairingKey(){
	
// 	uint8_t key[16];
// 	uint8_t random_values_length, rand_values, generated;
// 	static ble_advdata_tk_value_t oob_auth_key;
// 	uint32_t len=sizeof(ndef_msg_buf);
	
// 	//try to generate the key randomly
// 	sd_rand_application_pool_capacity_get(&random_values_length);
// 	//we need only 16 bytes
// 	if(random_values_length>16)
// 		random_values_length=16;
// 	//wait until values are generated
// 	do
// 		sd_rand_application_bytes_available_get(&generated);
// 	while(generated<random_values_length);
// 	//get the random data
// 	sd_rand_application_vector_get(key, random_values_length);
// 	//if random values are less than 16 add static data to fill the buffer
// 	if(random_values_length<16)
// 		for(int i=random_values_length; i<16; i++)
// 			key[i]=i;

// 	memcpy(oob_auth_key.tk, key, 16);	

// 	nfc_t2t_setup(nfc_callback, NULL);
// 	nfc_ble_pair_default_msg_encode(NFC_BLE_PAIR_MSG_FULL,
//                                                &oob_auth_key,
//                                                ndef_msg_buf,
//                                                &len);
											   
// 	nfc_t2t_payload_set((char *) ndef_msg_buf, len);
// }

void NFCClass::start(){
	nfc_t2t_emulation_start();	
}

void NFCClass::stop(){
	nfc_t2t_emulation_stop();
}

void NFCClass::registerCallback(void(*function)(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t dataLength)){
	Callback=function;
}

void NFCClass::onService(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t dataLength){
	if(Callback)
		Callback(context,event,data,dataLength);
 }
 

NFCClass NFC;
