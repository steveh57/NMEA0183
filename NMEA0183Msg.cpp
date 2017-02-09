/* 
NMEA0183Msg.cpp

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

#include <NMEA0183Msg.h>
#include <NMEA0183.h>

tNMEA0183Msg::tNMEA0183Msg() {
  Clear();
}

//*****************************************************************************
bool tNMEA0183Msg::SetMessage(const char *buf) {
  unsigned char csMsg;
  int i=0;
  uint8_t iData=0;
  bool result=false;

  Clear();
  _MessageTime=millis();
  
  if (buf[i]!='$') return result; // Invalid message
  Prefix=buf[i];
  i++; // Pass start prefix
  
//  Serial.println(buf);
  // Set sender
  for (; iData<2 && buf[i]!=0; i++, iData++) {
    CheckSum^=buf[i];
    Data[iData]=buf[i];
  }
  
  if (buf[i]==0) { Clear(); return result; } // Invalid message

  Data[iData]=0; iData++; // null termination for sender
  // Set message code. Read until next comma
  for (; buf[i]!=',' && buf[i]!=0 && iData<MAX_NMEA0183_MSG_LEN; i++, iData++) {
    CheckSum^=buf[i];
    Data[iData]=buf[i];
  }
  if (buf[i]!=',') { Clear(); return result; } // No separation after message code -> invalid message
  
  // Set the data and calculate checksum. Read until '*'
  for (; buf[i]!='*' && buf[i]!=0 && iData<MAX_NMEA0183_MSG_LEN; i++, iData++) {
    CheckSum^=buf[i];
    Data[iData]=buf[i];
    if (buf[i]==',') { // New field
      Data[iData]=0; // null termination for previous field
      Fields[_FieldCount]=iData+1;   // Set start of field
      _FieldCount++;
    }
  }
  
  if (buf[i]!='*') { Clear(); return false; } // No checksum -> invalid message
  Data[iData]=0; // null termination for previous field
  NextFieldPos = iData + 1;
  i++; // Pass '*';
  csMsg=(buf[i]<=57?buf[i]-48:buf[i]-55)<<4; i++;
  csMsg|=(buf[i]<=57?buf[i]-48:buf[i]-55);
  
  if (csMsg==CheckSum) { 
    result=true;
  } else {
    Clear();
  }
  
  return result;
}

//*****************************************************************************
void tNMEA0183Msg::Clear() {
  SourceID=0;
  Data[0]=0;  // Sender is empty
  Data[2]=0;  // Message code is empty
  _FieldCount=0;
  Fields[0]=0;
  _MessageTime=0;
  CheckSum=0;
  Prefix=' ';
  NextFieldPos = 0;
}
//*****************************************************************************
// Init
// Initialise a message with:
// - prefix - either $ or !#
// - 2 character sender i/d
// - 3 character message code

bool tNMEA0183Msg::Init(const char _prefix, const char* _sender, const char* _msgtype) {
	Clear();
	  if (_prefix != '$' && _prefix !='!' && strlen (_sender ) != 2 && strlen (_msgtype) !=3)
		  return false;
	Prefix = _prefix;
	strcpy (Data, _sender);
	strcpy (Data+3, _msgtype);
	NextFieldPos = 7;
	return true;
}
//*****************************************************************************
// AddField
// Adds one field to the message
bool tNMEA0183Msg::AddField (const char* _field) {
	if (_FieldCount>=MAX_NMEA0183_MSG_FIELDS) return false;  //already at limit

	uint8_t fieldlen = strlen(_field);
	if (NextFieldPos+fieldlen >= MAX_NMEA0183_MSG_LEN) return false;
	strcpy (Data+NextFieldPos, _field);

	Fields[_FieldCount]=NextFieldPos;
	_FieldCount++;
	NextFieldPos += fieldlen+1;
	return true;	
}

bool tNMEA0183Msg::AddField (const char _field) {
	// single character version
	if (_FieldCount>=MAX_NMEA0183_MSG_FIELDS) return false;  //already at limit

	if (NextFieldPos+1 >= MAX_NMEA0183_MSG_LEN) return false;
	Data[NextFieldPos] = _field;
	Data[NextFieldPos+1] = 0;

	Fields[_FieldCount]=NextFieldPos;
	_FieldCount++;
	NextFieldPos += 2;
	return true;
}

bool tNMEA0183Msg::AddField (const double _field) {
	char fld[10];
	int valInt = _field;
	int valFrag = (double)(_field - valInt) * 100;

	if (valFrag < 10) {
		sprintf(fld,"%d.0%d",valInt, valFrag);
	} else {
		sprintf(fld,"%d.%d", valInt, valFrag);
	}
//	Serial1.print ("AddField: "); Serial1.println(fld);

	return AddField (fld);
}
//*****************************************************************************
//void tNMEA0183Msg::PrintFields(Stream &port) const {
//}

//*****************************************************************************
// DEPRECATED:
// This version must not be used with NMEA0183 forwarding.  Messages may get
// mixed up.
void tNMEA0183Msg::Send(Stream &port) const {
  if (FieldCount()==0) return;
  port.print(Prefix);
  port.print(Sender());
  port.print(MessageCode());
  for (int i=0; i<FieldCount(); i++) {
    port.print(",");
    port.print(Field(i));
  }
  port.print("*");
  port.print(CheckSum,HEX);
  port.print("\r\n"); 
}

//*****************************************************************************
const char *tNMEA0183Msg::Field(uint8_t index) const {
  if (index<FieldCount()) {
    return Data+Fields[index];
  } else {
    return Data;
  }
}

//*****************************************************************************
unsigned int tNMEA0183Msg::FieldLen(uint8_t index) const {
  if (index<FieldCount()) {
    return strlen(Data+Fields[index]);
  } else {
    return 0;
  }
}
