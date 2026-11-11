/*
  SendURL.ino
  
  Written by Chiara Ruggeri (chiara@arduino.org)
  
  This example for the Arduino Primo board shows how to use
  NFC library.
  It sets an URI message and then starts the module, so that
  when a device with NFC is near to the board the message
  http://www.arduino.org will be sent.
  A list of all possible URI message can be found here:

  NFC_URI_HTTP_WWW     "http://www."
  NFC_URI_HTTPS_WWW    "https://www."
  NFC_URI_HTTP         "http:"
  NFC_URI_HTTPS        "https:"
  NFC_URI_TEL          "tel:"
  NFC_URI_MAILTO       "mailto:"
  NFC_URI_FTP_ANONYMOUS  "ftp://anonymous:anonymous@"
  NFC_URI_FTP_FTP      "ftp://ftp."
  NFC_URI_FTPS         "ftps://"
  NFC_URI_SFTP,        "sftp://"
  NFC_URI_SMB          "smb://"
  NFC_URI_NFS          "nfs://"
  NFC_URI_FTP          "ftp://"
  NFC_URI_DAV          "dav://"
  NFC_URI_NEWS         "news:"
  NFC_URI_TELNET       "telnet://"
  NFC_URI_IMAP         "imap:" 
  NFC_URI_RTSP         "rtsp://"
  NFC_URI_URN          "urn:"
  NFC_URI_POP          "pop:"
  NFC_URI_SIP          "sip:"
  NFC_URI_SIPS         "sips:"
  NFC_URI_TFTP         "tftp:"
  NFC_URI_BTSPP        "btspp://"
  NFC_URI_BTL2CAP      "btl2cap://"
  NFC_URI_BTGOEP       "btgoep://"
  NFC_URI_TCPOBEX      "tcpobex://"
  NFC_URI_IRDAOBEX     "irdaobex://"
  NFC_URI_FILE         "file://"
  NFC_URI_URN_EPC_ID   "urn:epc:id:"
  NFC_URI_URN_EPC_TAG  "urn:epc:tag:"
  NFC_URI_URN_EPC_PAT  "urn:epc:pat:"
  NFC_URI_URN_EPC_RAW  "urn:epc:raw:"
  NFC_URI_URN_EPC      "urn:epc:"
  NFC_URI_URN_NFC      "urn:nfc:"
  
 This example code is in the public domain.
  
*/

#include <NFC.h>

void setup() {
  NFC.setURImessage("arduino.org", NFC_URI_HTTP_WWW);
  NFC.start();
}


void loop() {
}