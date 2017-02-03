/* 
NMEA0183.cpp

2015-2016 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/

#include <NMEA0183.h>

tNMEA0183::tNMEA0183()
: porttype(port_undefined), port(0), usb(0),  MsgCheckSumStartPos(-1), MsgOutIdx(0),
  MsgInPos(0), MsgOutPos(0),
  MsgInStarted(false), MsgOutStarted(false),
  SourceID(0), MsgHandler(0)
{
  for(int i=0; i < MAX_OUT_BUF;i++) {
    MsgOutBuf[i][0]='\0';
  }
  for (uint8_t i=0; i < NMEA0183_MAX_FORWARD; i++) {
	  forwardport[i]=0;
  }
}

//*****************************************************************************
void tNMEA0183::Begin(HardwareSerial *_port, uint8_t _SourceID, unsigned long _baud) {
  porttype = port_hardware;
  SourceID=_SourceID;
  port=_port;
  port->begin(_baud);
}

void tNMEA0183::Begin(usb_serial_class *_port, uint8_t _SourceID, unsigned long _baud) {
	  porttype = port_usb;
	  SourceID=_SourceID;
	  usb =_port;
	  usb->begin(_baud);
	}
//*****************************************************************************
void tNMEA0183::SetForward (tNMEA0183 *_forward, uint8_t _channel) {
	if (_channel >=0 && _channel < NMEA0183_MAX_FORWARD) {
		forwardport[_channel] = _forward;
	}
}

//*****************************************************************************
void tNMEA0183::ParseMessages() {
  tNMEA0183Msg NMEA0183Msg;
    while (GetMessage(NMEA0183Msg)) {
      if (MsgHandler!=0) MsgHandler(NMEA0183Msg);
    }
}
/*Kluge to support USB serial as a NMEA0183 input/output
 *
 * Unfortunately the Stream base class for hardware serial and usb serial does not
 * define AvailableForWriteas a virtual function, so we have to do this to get round
 * the problem.  There's probably a more elegant way to do this but hey.
 *
 */
int tNMEA0183::serial_available () {
	switch (porttype) {
	case port_hardware:
		return port->available();
	case port_usb:
		return usb->available();
	default:
		return 0;
	}
}
int tNMEA0183::serial_read () {
	switch (porttype) {
	case port_hardware:
		return port->read();
	case port_usb:
		return usb->read();
	default:
		return 0;
	}
}

int tNMEA0183::serial_availableForWrite() {
	switch (porttype) {
	case port_hardware:
		return port->availableForWrite();
	case port_usb:
		return usb->availableForWrite();
	default:
		return 0;
	}
}

size_t tNMEA0183::serial_write(int n) {
	switch (porttype) {
	case port_hardware:
		return port->write(n);
	case port_usb:
		return usb->write(n);
	default:
		return 0;
	}
}


//*****************************************************************************
bool tNMEA0183::GetMessage(tNMEA0183Msg &NMEA0183Msg) {
  bool result=false;

  while (serial_available() > 0 && !result) {
    int NewByte=serial_read();
      if (NewByte=='$' || NewByte=='!') { // Message start
        MsgInStarted=true;
        MsgInPos=0;
        MsgInBuf[MsgInPos]=NewByte;
        MsgInPos++;
      } else if (MsgInStarted) {
        MsgInBuf[MsgInPos]=NewByte;
        if (NewByte=='*') MsgCheckSumStartPos=MsgInPos;
        MsgInPos++;
        if (MsgCheckSumStartPos>0 and MsgCheckSumStartPos+3==MsgInPos) { // We have full checksum and so full message
            MsgInBuf[MsgInPos]=0; // add null termination
            // see if we need to forward it to other streams
            // some devices require crlf to terminate the message, so add it anyway on forwarding
            for (uint8_t i=0; i < NMEA0183_MAX_FORWARD; i++) {
            	if (forwardport[i] != 0) {
            		forwardport[i]->SendMessage(MsgInBuf, add_crlf);
            	}
            }
          if (NMEA0183Msg.SetMessage(MsgInBuf)) {
            NMEA0183Msg.SourceID=SourceID;
            result=true;
          }
          MsgInStarted=false;
          MsgInPos=0;
          MsgCheckSumStartPos=-1;  
        }
        if (MsgInPos>=MAX_NMEA0183_MSG_BUF_LEN) { // Too may chars in message. Start from beginning
          MsgInStarted=false;
          MsgInPos=0;
          MsgCheckSumStartPos=-1;  
        }
      }
  }
  
  return result;
}
/*****************************************************************************
* The send message process uses an array of message buffers (MsgOutBuf) to
* provide a queue for outgoing messages.  This is necessary to deal with
* the fact that the Serial buffer in the core code is set to 64 bytes for
* transmit and receive, regardless of the data or speed, and NMEA0183 messages
* can be up to 82 characters long.
*
* A better solution would be simply to increase the serial buffer size, but it's not
* easy to change this without modifying a core module.  This could be done by
* defining SERIAL1_TX_BUFFER_SIZE = xxx in hardwareserial.h or serial1.c
*
* Another slightly more efficient approach might be to replace the queue with
* a circular character buffer.  However the message queue approach keeps messages
* intact and although complete messages may get dropped, they shouldn't get
* corrupted.
*/
//*****************************************************************************
int tNMEA0183::nextOutIdx(int idx)
{
  idx++;
  if (idx >= MAX_OUT_BUF) {
    idx = 0;
  }
  return idx;
}

//*****************************************************************************
// Kick function must be called as often as possible, especially if combining
// several streams to one output.
void tNMEA0183::kick() {
	if (MsgOutBuf[MsgOutIdx][0] != '\0') {
		MsgOutStarted = true;
		while(serial_availableForWrite() > 0) {
			serial_write(MsgOutBuf[MsgOutIdx][MsgOutPos]);
			MsgOutPos++;
			if (MsgOutBuf[MsgOutIdx][MsgOutPos] == '\0') {
				// Done with this message - clear it and prepare for next
				MsgOutBuf[MsgOutIdx][0] = '\0';
				MsgOutIdx = nextOutIdx(MsgOutIdx);
				MsgOutPos=0;
				if (MsgOutBuf[MsgOutIdx][0] == '\0') {
					MsgOutStarted=false; //no more messages to send
					return;
				};
			}
		}
	}
}
//*****************************************************************************
void AddChecksum(char* msg) {
  unsigned int i=1; // First character not included in checksum
  uint8_t chkSum = 0;
  char ascChkSum[4];
  uint8_t tmp;

  while (msg[i] != '\0') {
    chkSum ^= msg[i];
    i++;
  }
  ascChkSum[0] ='*';
  tmp = chkSum/16;
  ascChkSum[1] = tmp > 9 ? 'A' + tmp-10 : '0' + tmp;
  tmp = chkSum%16;
  ascChkSum[2] = tmp > 9 ? 'A' + tmp-10 : '0' + tmp;
  ascChkSum[3] = '\0';
  strcat(msg, ascChkSum);
}
//*****************************************************************************
char *tNMEA0183::nextOutBuf() {
	kick(); // clear out some space if possible

	if (!MsgOutStarted)
		return &MsgOutBuf[MsgOutIdx][0];

	for (int i = nextOutIdx(MsgOutIdx); i != MsgOutIdx; i = nextOutIdx(i)) {
		if (MsgOutBuf[i][0] == '\0') {
			return &MsgOutBuf[i][0]; // Yep it is free
		}
	}

	return 0;
}
//*****************************************************************************
bool tNMEA0183::SendMessage(const char *buf, NMEA0183_MsgSendType _sendtype) {

	int msglen = strlen(buf);
	switch (_sendtype) {
	case add_crlf:
		msglen += 2;
		break;
	case add_checksum:
		msglen += 5;
	}

	if (msglen >= MAX_NMEA0183_MSG_BUF_LEN)
		return false;

	char *outbuf = nextOutBuf();
	if (outbuf !=0 ){
		strcpy(outbuf, buf);
		if (_sendtype == add_checksum) {
			AddChecksum(outbuf);
			strcat(outbuf, "\r\n");
		}
		if (_sendtype == add_crlf) {
			strcat(outbuf, "\r\n");
		}
		kick();
		return true;
	}
	return false;

}

/*****************************************************************************
* SendMessage in tNMEA0183 class format
*
* Loads the message into a character string, inserting commas and adding the checksum.
* Assumes that the data in the message is valid - no checks on validity made here
*
*/

bool tNMEA0183::SendMessage(const tNMEA0183Msg &msg) {
	if (msg._FieldCount==0) return false;
	char buf[MAX_NMEA0183_MSG_BUF_LEN];
	int i=0, iData=0;

	buf[i] = msg.Prefix;
	i++;

	// Set sender
	for (; iData<2; i++, iData++) {
		buf[i]=msg.Data[iData];
	}
	iData++; // skip 0 after sender

	// Set message code. Read until end string.
	for (; msg.Data[iData]!=0 && iData<MAX_NMEA0183_MSG_LEN; i++, iData++) {
		buf[i]=msg.Data[iData];
	}
	// Insert comma and add the fields
	buf[i]=',';
	i++; iData++;


	for (int f = 0; f < msg._FieldCount && i<MAX_NMEA0183_MSG_LEN; i++, iData++) {
		buf[i]=msg.Data[iData];
		if (msg.Data[iData]==0) { // end of field
			f++;
			buf[i] = f==msg._FieldCount ? 0 : ','; // insert comma or terminate the string
		}
	}

	if (i >= MAX_NMEA0183_MSG_LEN) return false;

	// Message complete.  Now send it.

	return SendMessage(buf, add_checksum);

}


