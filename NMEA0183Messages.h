/* 
NMEA0183Messages.h

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

#ifndef _tNMEA0183_MESSAGES_H_
#define _tNMEA0183_MESSAGES_H_
#include <TimeLib.h>
#include <NMEA0183Msg.h>

bool NMEA0183ParseGGA_nc(const tNMEA0183Msg &NMEA0183Msg, double &GPSTime, double &Latitude, double &Longitude,
                      int &GPSQualityIndicator, int &SatelliteCount, double &HDOP, double &Altitude, double &GeoidalSeparation,
                      double &DGPSAge, int &DGPSReferenceStationID);

inline bool NMEA0183ParseGGA(const tNMEA0183Msg &NMEA0183Msg, double &GPSTime, double &Latitude, double &Longitude,
                      int &GPSQualityIndicator, int &SatelliteCount, double &HDOP, double &Altitude, double &GeoidalSeparation,
                      double &DGPSAge, int &DGPSReferenceStationID) {
  return (NMEA0183Msg.IsMessageCode("GGA")
            ?NMEA0183ParseGGA_nc(NMEA0183Msg,GPSTime,Latitude,Longitude,GPSQualityIndicator,SatelliteCount,HDOP,Altitude,GeoidalSeparation,DGPSAge,DGPSReferenceStationID)
            :false);
}

// RMC
bool NMEA0183ParseRMC_nc(const tNMEA0183Msg &NMEA0183Msg, double &GPSTime, double &Latitude, double &Longitude,
                      double &TrueCOG, double &SOG, unsigned long &DaysSince1970, double &Variation, time_t *DateTime=0);

inline bool NMEA0183ParseRMC(const tNMEA0183Msg &NMEA0183Msg, double &GPSTime, double &Latitude, double &Longitude,
                      double &TrueCOG, double &SOG, unsigned long &DaysSince1970, double &Variation, time_t *DateTime=0) {
  (void)DateTime;
  return (NMEA0183Msg.IsMessageCode("RMC")
            ?NMEA0183ParseRMC_nc(NMEA0183Msg, GPSTime, Latitude, Longitude, TrueCOG, SOG, DaysSince1970, Variation, DateTime)
            :false);
}
bool NMEA0183SetRMC ( tNMEA0183Msg &NMEA0183Msg, const char *Source, char &Status, time_t &DateTime, double &Latitude, double &Longitude,
                      double &TrueCOG, double &SOG, double &Variation, char &FAAMode);

// COG will be returned be in radians
// SOG will be returned in m/s
bool NMEA0183ParseVTG_nc(const tNMEA0183Msg &NMEA0183Msg, double &TrueCOG, double &MagneticCOG, double &SOG);

inline bool NMEA0183ParseVTG(const tNMEA0183Msg &NMEA0183Msg, double &TrueCOG, double &MagneticCOG, double &SOG) {
  return (NMEA0183Msg.IsMessageCode("VTG")
            ?NMEA0183ParseVTG_nc(NMEA0183Msg,TrueCOG,MagneticCOG,SOG)
            :false);
}
bool NMEA0183SetVTG(tNMEA0183Msg &NMEA0183Msg, const char *Source, double &TrueCOG, double &MagneticCOG, double &SOG, char FAAMode='A');

bool NMEA0183BuildVTG(char* msg, const char Src[], double TrueCOG, double MagneticCOG, double SOG);

// Heading will be returned be in radians
// HDT - Heading True
bool NMEA0183ParseHDT_nc(const tNMEA0183Msg &NMEA0183Msg,double &TrueHeading); 

inline bool NMEA0183ParseHDT(const tNMEA0183Msg &NMEA0183Msg, double &TrueHeading) {
  return (NMEA0183Msg.IsMessageCode("HDT")
            ?NMEA0183ParseHDT_nc(NMEA0183Msg,TrueHeading)
            :false);
}
bool NMEA0183SetHDT ( tNMEA0183Msg &NMEA0183Msg, const char *Source, const double &TrueHeading);
//HDM - Heading Magnetic
bool NMEA0183ParseHDM_nc(const tNMEA0183Msg &NMEA0183Msg,double &MagHeading);
inline bool NMEA0183ParseHDM(const tNMEA0183Msg &NMEA0183Msg,double &MagHeading){
	  return (NMEA0183Msg.IsMessageCode("HDM") ?
	            NMEA0183ParseHDM_nc(NMEA0183Msg,MagHeading) : false);
}
bool NMEA0183SetHDM ( tNMEA0183Msg &NMEA0183Msg, const char *Source, const double &MagHeading);

//HDG - Heading compass with deviation and variation
bool NMEA0183ParseHDG_nc(const tNMEA0183Msg &NMEA0183Msg,double &MagHeading,double &Deviation, double &Variation);
inline bool NMEA0183ParseHDG(const tNMEA0183Msg &NMEA0183Msg,double &MagHeading,double &Deviation, double &Variation) {
	  return (NMEA0183Msg.IsMessageCode("HDG") ?
	            NMEA0183ParseHDG_nc(NMEA0183Msg, MagHeading, Deviation, Variation) : false);
}
bool NMEA0183SetHDG ( tNMEA0183Msg &NMEA0183Msg, const char *Source, const double &MagHeading, const double &Deviation,
		const double &Variation);

// DBS, DBT, DBK Depth below surface/transducer/keel
enum eDepthType {depth_na, depth_surface, depth_keel, depth_transducer};
bool NMEA0183ParseDBX_nc(const tNMEA0183Msg &NMEA0183Msg, double &DepthMetres, eDepthType &DepthType);

inline bool NMEA0183ParseDBT(const tNMEA0183Msg &NMEA0183Msg,double &DepthMetres) {
	eDepthType dt;
	return ( NMEA0183Msg.IsMessageCode("DBT") ?
			NMEA0183ParseDBX_nc(NMEA0183Msg, DepthMetres, dt) : false);
}
inline bool NMEA0183ParseDBS(const tNMEA0183Msg &NMEA0183Msg,double &DepthMetres) {
	eDepthType dt;
	return ( NMEA0183Msg.IsMessageCode("DBS") ?
			NMEA0183ParseDBX_nc(NMEA0183Msg, DepthMetres, dt) : false);
}
inline bool NMEA0183ParseDBK(const tNMEA0183Msg &NMEA0183Msg,double &DepthMetres) {
	eDepthType dt;
	return ( NMEA0183Msg.IsMessageCode("DBK") ?
			NMEA0183ParseDBX_nc(NMEA0183Msg, DepthMetres, dt) : false);
}

bool NMEA0183SetDBX ( tNMEA0183Msg &NMEA0183Msg, const char *Source, const double &DepthMetres, const eDepthType DepthType );

// VDM is basically a bitstream
bool NMEA0183ParseVDM_nc(const tNMEA0183Msg &NMEA0183Msg,
			uint8_t &pkgCnt, uint8_t &pkgNmb,
			unsigned int &seqMessageId, char &channel,
			unsigned int &length, char *bitstream,
			unsigned int &fillBits);


inline bool NMEA0183ParseVDM(const tNMEA0183Msg &NMEA0183Msg, uint8_t &pkgCnt, uint8_t &pkgNmb,
						unsigned int &seqMessageId, char &channel,
						unsigned int &length, char* bitstream, unsigned int &fillBits) {
  return (NMEA0183Msg.IsMessageCode("VDM") ?
		NMEA0183ParseVDM_nc(NMEA0183Msg, pkgCnt, pkgNmb, seqMessageId, channel, length, bitstream, fillBits) : false);
}


#endif
