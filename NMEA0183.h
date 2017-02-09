/* 
NMEA0183.h

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

#ifndef _tNMEA0183_H_
#define _tNMEA0183_H_
#include <Arduino.h>
#include <NMEA0183Msg.h>

// Definition changed to avoid confusion!
#define MAX_NMEA0183_MSG_BUF_LEN MAX_NMEA0183_MSG_LEN+2  // Allow space in buffer for terminator

// Number of output buffers required depends on data being streamed.
// For output at 38400 you may need to increase to 8 or more, particularly
// if messages are sent in clusters.
#define MAX_OUT_BUF 8
#define NMEA0183_MAX_FORWARD 3	//maximum number of forwarding ports supported

enum NMEA0183_PortType {port_undefined, port_hardware, port_usb};
enum NMEA0183_MsgSendType {send_raw, add_crlf, add_checksum};
enum NMEA0183status {nmea0183_success, nmea0183_invalid, nmea0183_nobuffers};

class tNMEA0183
{
  protected:
	NMEA0183_PortType porttype;
    HardwareSerial *port;
    usb_serial_class *usb;
    tNMEA0183 *forwardport[NMEA0183_MAX_FORWARD];
    int MsgCheckSumStartPos;
    char MsgInBuf[MAX_NMEA0183_MSG_BUF_LEN];
    char MsgOutBuf[MAX_OUT_BUF][MAX_NMEA0183_MSG_BUF_LEN];
    int MsgOutIdx;
    int MsgInPos;
    int MsgOutPos;
    bool MsgInStarted;
    bool MsgOutStarted;
    uint8_t SourceID;  // User defined ID for this message handler

    // Handler callback
    void (*MsgHandler)(const tNMEA0183Msg &NMEA0183Msg);
    int nextOutIdx(int idx);

  public:
    tNMEA0183();
    void Begin(HardwareSerial *_port, uint8_t _SourceID=0, unsigned long _baud=4800);
    void Begin(usb_serial_class *_port, uint8_t _SourceID=0, unsigned long _baud=115200);
    // Use SetForward to define another NMEA0183 stream to forward messages to automatically.
    // Any NMEA0183 can forward to multiple other streams, including itself.
    // Only valid messages are forwarded.
    void SetForward (tNMEA0183 *_forward, uint8_t _channel);
    void SetMsgHandler(void (*_MsgHandler)(const tNMEA0183Msg &NMEA0183Msg)) {MsgHandler=_MsgHandler;}
    void ParseMessages();
    bool GetMessage(tNMEA0183Msg &NMEA0183Msg);
    bool SendMessage(const char *buf, NMEA0183_MsgSendType _sendtype=send_raw);
    NMEA0183status SendMessage(const tNMEA0183Msg &msg);
    bool kick();
  private:
    int serial_available ();
    int serial_read ();
    int serial_availableForWrite();
    size_t serial_write(int n);
    char *nextOutBuf();



};

#endif
