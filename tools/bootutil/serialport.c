/**********************************************************************************
 **********************************************************************************
 ***
 ***    serialport.c
 ***    - low level functions to access the serial port
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#if defined (_WIN32)
#include <Windows.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#endif

#include "serialport.h"
#include "infohelper.h"

#if defined (_WIN32)
static HANDLE sPort = NULL;
static DCB sDCB;
static COMMTIMEOUTS sTIMEOUTS;
#else
static int serial_port = -1;
static struct termios term;
static unsigned int timeout;
#endif


#ifdef _WIN32

#ifndef CBR_230400
#define CBR_230400 230400
#endif
#ifndef CBR_460800
#define CBR_460800 460800
#endif
#ifndef CBR_512000
#define CBR_512000 512000
#endif
#ifndef CBR_921600
#define CBR_921600 921600
#endif


void serialport_setbaudrate(unsigned int baudrate)
{
	DWORD br = 0;
	switch(baudrate)
	{
		case 9600: 		br = CBR_9600; 		break;
		case 14400: 	br = CBR_14400; 	break;
		case 19200: 	br = CBR_19200; 	break;
		case 38400: 	br = CBR_38400; 	break;
		case 56000: 	br = CBR_56000; 	break;
		case 57600: 	br = CBR_57600; 	break;
		case 115200: 	br = CBR_115200; 	break;
		case 128000: 	br = CBR_128000; 	break;
        case 230400:    br = CBR_230400;    break;
		case 256000:	br = CBR_256000;	break;
        case 512000:    br = CBR_512000;    break;
        case 460800:    br = CBR_460800;    break;
        case 921600:    br = CBR_921600;    break;
	}
	if (br == 0)
	{
		LOGWARN("unsupported baud rate: %d, using 115200", baudrate);
		br = CBR_115200;
	}

	memset(&sDCB, 0, sizeof(DCB));
	BuildCommDCB("baud=9600 parity=N data=8 stop=1",&sDCB);
	sDCB.DCBlength		= 	sizeof(DCB);
	sDCB.BaudRate		= 	br;
	sDCB.fBinary		=	TRUE;
	sDCB.fParity		=	FALSE;
//	sDCB.fOutxCtsFlow 	=	TRUE;
	sDCB.fDtrControl	=	DTR_CONTROL_DISABLE;
	sDCB.fDsrSensitivity=	FALSE;
	sDCB.fRtsControl	=	RTS_CONTROL_DISABLE;
	sDCB.ByteSize		=	8;
	sDCB.StopBits 		= 	ONESTOPBIT;
	sDCB.fAbortOnError	=	FALSE;
	sDCB.fOutX			=	FALSE;
	sDCB.fInX			=	FALSE;
	if (!SetCommState(sPort, &sDCB))
	{
		LOGDEBUG("SetCommState call failed");
	}
}

int serialport_open(const char *device, unsigned int baudrate)
{
	char portName[40];
	sprintf(portName,"\\\\.\\%s", device);
	sPort = CreateFile(portName, GENERIC_WRITE|GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (sPort == INVALID_HANDLE_VALUE)
	{
		LOGERR("Failed to open %s", device);
		return 0;
	}

	SetupComm(sPort, 256, 256);
	serialport_setbaudrate(baudrate);
	serialport_set_timeout(1000);
	return 1;
}

void serialport_set_timeout(unsigned int timeout)
{
	LOGDEBUG("setting serial port timeouts to %d ms", timeout);
	sTIMEOUTS.ReadIntervalTimeout 			= 0;
	sTIMEOUTS.ReadTotalTimeoutConstant 		= timeout;
	sTIMEOUTS.ReadTotalTimeoutMultiplier 	= 0;
	sTIMEOUTS.WriteTotalTimeoutConstant 	= timeout;
	sTIMEOUTS.WriteTotalTimeoutMultiplier 	= 0;
	if (!SetCommTimeouts(sPort,&sTIMEOUTS))
	{
		LOGDEBUG("SetCommTimeouts call failed");
	}
	SetCommMask(sPort, EV_TXEMPTY);
}

unsigned serialport_get_timeout()
{
    return sTIMEOUTS.ReadTotalTimeoutConstant;
}

unsigned serialport_read(unsigned char* data, unsigned int size)
{
	unsigned long cb;
	ReadFile(sPort, data, size, &cb, NULL);
	if (cb != size)
	{
		LOGDEBUG("read %d, requested %d", cb, size);
	}
	return (unsigned) cb;
}

unsigned serialport_write(const unsigned char* data, unsigned int size)
{
	unsigned long cb;
	WriteFile(sPort, data, size, &cb, NULL);
	if (cb != size)
	{
		LOGDEBUG("wrote %d, requested %d", cb, size);
	}
	return (unsigned) cb;
}

void serialport_flush(void)
{
	unsigned char tmp[512];
    if(sPort)
    {
		LOGDEBUG("flush start");
		unsigned old_timeout = sTIMEOUTS.ReadTotalTimeoutConstant;
		serialport_set_timeout(1);
		ClearCommError(sPort, NULL, NULL);
		PurgeComm(sPort, PURGE_TXCLEAR|PURGE_RXCLEAR);
		unsigned long cb;
		int result;
		do {
			result = ReadFile(sPort, tmp, 512, &cb, NULL);
			LOGVERBOSE("flushed %lu bytes", cb);
		} while(cb && result == 0);
		serialport_set_timeout(old_timeout);
		LOGDEBUG("flush complete");
    }
}

void serialport_drain(void)
{
    if(sPort)
    {
		FlushFileBuffers(sPort);
    }

}

int serialport_close(void)
{
    if(!sPort)
		return 0;
	serialport_drain();
	serialport_flush();
	CloseHandle(sPort);
	sPort = NULL;
	return 1;
}

void serialport_set_dtr(unsigned char val)
{
    if(sPort)
    {
		EscapeCommFunction(sPort,((val)?SETDTR:CLRDTR));
    }
}

void serialport_set_rts(unsigned char val)
{
    if(sPort)
    {
		EscapeCommFunction(sPort,((val)?SETRTS:CLRRTS));
    }
}

void serialport_send_break()
{
    if (sPort)
    {
        EscapeCommFunction(sPort, SETBREAK);
        Sleep(250);
        EscapeCommFunction(sPort, CLRBREAK);
    }
}


#else

void serialport_set_baudrate(unsigned int baudrate)
{
    switch(baudrate)
    {
        case 2400:
            cfsetispeed(&term,B2400);
            cfsetospeed(&term,B2400);
            break;

        case 4800:
            cfsetispeed(&term,B4800);
            cfsetospeed(&term,B4800);
            break;

        case 9600:
            cfsetispeed(&term,B9600);
            cfsetospeed(&term,B9600);
            break;

        case 19200:
            cfsetispeed(&term,B19200);
            cfsetospeed(&term,B19200);
            break;

        case 38400:
            cfsetispeed(&term,B38400);
            cfsetospeed(&term,B38400);
            break;

        case 57600:
            cfsetispeed(&term,B57600);
            cfsetospeed(&term,B57600);
            break;

        case 115200:
            cfsetispeed(&term,B115200);
            cfsetospeed(&term,B115200);
            break;

        case 230400:
            cfsetispeed(&term,B230400);
            cfsetospeed(&term,B230400);
            break;
#ifndef __APPLE__
        case 460800:
            cfsetispeed(&term,B460800);
            cfsetospeed(&term,B460800);
            break;

        case 921600:
            cfsetispeed(&term,B921600);
            cfsetospeed(&term,B921600);
            break;
#endif
        default:
            LOGWARN("serialport_set_baudrate: baud rate %d may not work", baudrate);
            cfsetispeed(&term,baudrate);
            cfsetospeed(&term,baudrate);
            break;
    }
}

void serialport_set_timeout(unsigned int t)
{
    if(t != timeout)
    {
        LOGDEBUG("setting timeout %i", t);
        timeout = t;
    }
}

unsigned serialport_get_timeout()
{
    return timeout;
}

int serialport_open(const char *device, unsigned int baudrate)
{
    LOGINFO("opening port %s at %d", device, baudrate);
    int flags = O_RDWR | O_NOCTTY;
#ifdef __APPLE__
    flags |= O_NONBLOCK;
#endif
    serial_port = open(device, flags);

    if(serial_port<0)
    {
        LOGERR("cannot access %s\n",device);
        return 0;
    }

#ifdef __APPLE__
    flags = fcntl(serial_port, F_GETFL, 0);
    fcntl(serial_port, F_SETFL, flags & (~O_NONBLOCK));
#endif

    serialport_set_dtr(0);

    LOGDEBUG("tcgetattr");
    tcgetattr(serial_port,&term);

    serialport_set_baudrate(baudrate);

    term.c_cflag = (term.c_cflag & ~CSIZE) | CS8;
    term.c_cflag |= CLOCAL | CREAD;

    term.c_cflag &= ~(PARENB | PARODD);
    term.c_cflag &= ~CSTOPB;

    term.c_iflag = IGNBRK;

    term.c_iflag &= ~(IXON | IXOFF);

    term.c_lflag = 0;

    term.c_oflag = 0;


    term.c_cc[VMIN]=0;
    term.c_cc[VTIME]=1;
    timeout = 100;



    LOGDEBUG("tcsetattr");
    if (tcsetattr(serial_port, TCSANOW, &term)!=0)
    {
        LOGERR("setattr stage 1 failed");
        return 0;
    }


    if (tcgetattr(serial_port, &term)!=0)
    {
        LOGERR("getattr failed");
        return 0;
    }

    term.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_port, TCSANOW, &term)!=0)
    {
        LOGERR("setattr stage 2 failed");
        return 0;
    }
    LOGDEBUG("serial open");
    return serial_port;
}

unsigned serialport_read(unsigned char* data, unsigned int size)
{
    struct timeval tv0, tv1;
    gettimeofday(&tv0, NULL);
    unsigned n = 0;
    unsigned time_spent = 0;
    do
    {
        unsigned cb = read(serial_port, data + n, size - n);
        n += cb;
        gettimeofday(&tv1, NULL);
        time_spent = (tv1.tv_sec - tv0.tv_sec) * 1000 + tv1.tv_usec / 1000 - tv0.tv_usec / 1000;
    } while (n < size && time_spent < timeout);
    return n;
}

unsigned serialport_write(const unsigned char* data, unsigned int size)
{
	return write(serial_port, data, size);
}

void serialport_flush(void)
{
    static unsigned char b;

    if(serial_port)
    {
        tcdrain(serial_port);
        while(read(serial_port, &b, 1) > 0);
    }
}

void serialport_drain(void)
{
    if(serial_port)
    {
        tcdrain(serial_port);
    }
}

int serialport_close(void)
{
    if(serial_port)
    {
        tcdrain(serial_port);
        tcflush(serial_port, TCIOFLUSH);
        close(serial_port);
        return 1;
    }
    else
    {
        return 0;
    }
}

void serialport_set_dtr(unsigned char val)
{
    int mcs;

    if(serial_port)
    {
        ioctl (serial_port, TIOCMGET, &mcs);

        if(val)
        {
            mcs |= TIOCM_DTR;
            ioctl (serial_port, TIOCMSET, &mcs);
        }
        else
        {
            mcs &= ~TIOCM_DTR;
            ioctl (serial_port, TIOCMSET, &mcs);
        }
    }
}

void serialport_set_rts(unsigned char val)
{
    int mcs;

    if(serial_port)
    {
        ioctl (serial_port, TIOCMGET, &mcs);

        if(val)
        {
            mcs |= TIOCM_RTS;
            ioctl (serial_port, TIOCMSET, &mcs);
        }
        else
        {
            mcs &= ~TIOCM_RTS;
            ioctl (serial_port, TIOCMSET, &mcs);
        }
    }
}

void serialport_send_break()
{
    tcsendbreak(serial_port, 0);
}



#endif
static unsigned char subst_C0[2] = { 0xDB, 0xDC };
static unsigned char subst_DB[2] = { 0xDB, 0xDD };

#define STATIC_SLIP_BUF_SIZE 4096
int serialport_send_slip_static(unsigned char *data, unsigned int size)
{
    unsigned char buf[STATIC_SLIP_BUF_SIZE];
    unsigned out_pos = 0;
    for (int i = 0; i < size; ++i)
    {
        unsigned char cur = data[i];
        if (cur == 0xC0)
        {
            buf[out_pos++] = subst_C0[0];
            buf[out_pos++] = subst_C0[1];
        }
        else if (cur == 0xDB)
        {
            buf[out_pos++] = subst_DB[0];
            buf[out_pos++] = subst_DB[1];
        }
        else
            buf[out_pos++] = cur;
    }

    if(serialport_write(buf, out_pos) != out_pos)
    {
        LOGERR("failed sending %d bytes", out_pos);
        return 0;
    }
    serialport_drain();
    return size;
}

int serialport_send_slip(unsigned char *data, unsigned int size)
{
    unsigned int sent;
    unsigned char cur_byte;

    if (size < STATIC_SLIP_BUF_SIZE / 2)
        return serialport_send_slip_static(data, size);

    sent = 0;

    while(sent != size)
    {
        cur_byte = *data++;
        if(cur_byte == 0xC0)
        {
            if(serialport_write(subst_C0, 2) != 2)
            {
                LOGERR("failed substituting 0xC0");
                return 0;
            }
        }
        else if(cur_byte == 0xDB)
        {
            if(serialport_write(subst_DB, 2) != 2)
            {
                LOGERR("failed substituting 0xDB");
                return 0;
            }
        }
        else
        {
            if(serialport_write(&cur_byte, 1) != 1)
            {
                LOGERR("failed sending byte %i", sent);
                return 0;
            }
        }
        sent++;
    }

    serialport_drain();

    return sent;
}

int serialport_receive_slip(unsigned char *data, unsigned int size)
{
    unsigned int received;
    unsigned char cur_byte;

    received = 0;

    while(received != size)
    {
        if(serialport_read(&cur_byte, 1) != 1)
        {
            LOGERR("failed reading byte");
            return 0;
        }

        if(cur_byte == 0xDB)
        {
            if(serialport_read(&cur_byte, 1) != 1)
            {
                LOGERR("failed reading byte for unslip");
                return 0;
            }

            if(cur_byte == 0xDC)
            {
                *data++ = 0xC0;
            }
            else if(cur_byte == 0xDD)
            {
                *data++ = 0xDB;
            }
            else
            {
                LOGERR("unslip sequence wrong");
                return 0;
            }
        }
        else
        {
            *data++ = cur_byte;
        }

        received++;
    }

    return received;
}

int serialport_send_C0(void)
{
    unsigned char b;

    b = 0xC0;

    if(serialport_write(&b, 1) != 1)
    {
        LOGERR("failed sending 0xC0");
        return 0;
    }
    serialport_drain();
    return 1;
}

int serialport_receive_C0(void)
{
    unsigned char b;

    b = 0x00;

    if(serialport_read(&b, 1) != 1)
    {
        return 0;
    }

    if(b != 0xC0)
    {
		LOGDEBUG("serialport_receive_C0: %02X instead of C0", b);
        return 0;
    }

    return 1;
}
