/**********************************************************************************
 **********************************************************************************
 ***
 ***    serialport.h
 ***    - include file for serialport.c
 ***
 ***    Copyright (C) 2014 Christian Klippel <ck@atelier-klippel.de>
 ***
 ***    This program is free software; you can redistribute it and/or modify
 ***    it under the terms of the GNU General Public License as published by
 ***    the Free Software Foundation; either version 2 of the License, or
 ***    (at your option) any later version.
 ***
 ***    This program is distributed in the hope that it will be useful,
 ***    but WITHOUT ANY WARRANTY; without even the implied warranty of
 ***    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ***    GNU General Public License for more details.
 ***
 ***    You should have received a copy of the GNU General Public License along
 ***    with this program; if not, write to the Free Software Foundation, Inc.,
 ***    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 ***
 **/


#ifndef SERIALPORT_H
#define SERIALPORT_H

int serialport_open(const char *dev, unsigned int baudrate);
int serialport_close(void);
void serialport_set_dtr(unsigned char val);
void serialport_set_rts(unsigned char val);
void serialport_send_break();
void serialport_set_timeout(unsigned int timeout);
unsigned serialport_get_timeout();
void serialport_drain(void);
void serialport_flush(void);
unsigned serialport_read(unsigned char* data, unsigned int size);
unsigned serialport_write(const unsigned char* data, unsigned int size);

int serialport_send_slip(unsigned char *data, unsigned int size);
int serialport_receive_slip(unsigned char *data, unsigned int size);
int serialport_send_C0(void);
int serialport_receive_C0(void);

#endif
