// Demo: NMEA2000 library. NMEA0183 Library.
//  NMEA 2000 bus Actisense format listener and sender. 
//   Example and test for converting NMEA2000 data to NMEA0183
//
//   Receive data received from serial (USB) in Actisense NMEA2000 format.
//   Use this e.g. with NMEA Simulator (see. http://www.kave.fi/Apps/index.html) to send simulated data to the bus.
//   Output NMEA0183 data to serial port.
//   Testing by observation of the NMEA0183 data on a serial terminal, analysis by Actisense Reader program,
//   and input to OpenCPN with data viewed on chartplotter and OpenCPN Dashboard.
//
//  Note that only a limited range of NMEA0183 sentences are generated at the moment.

#include <Arduino.h>
#include <NMEA2000_CAN.h>
#include <ActisenseReader.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>
#include <NMEA0183.h>
#include <NMEA0183Messages.h>

#define DISPLAYDATA false
#define DISPLAYPGN false
#define SENDACTISENSE false

typedef struct {
	unsigned long PGN;
	void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

struct tBoatData {
  double TrueHeading,SOG,COG,Variation, Latitude, Longitude;
  time_t COGSOGLastUpdate, LatLonLastUpdate, HeadingLastUpdate;

  tBoatData() {
    TrueHeading=0;
    SOG=0;
    COG=0;
    Variation=0;
    Latitude=0;
    Longitude=0;
    COGSOGLastUpdate=0;
    LatLonLastUpdate=0;
    HeadingLastUpdate=0;
  };
};

void SystemTime(const tN2kMsg &N2kMsg);
void EngineRapid(const tN2kMsg &N2kMsg);
void EngineDynamicParameters(const tN2kMsg &N2kMsg);
void TransmissionParameters(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);
void FluidLevel(const tN2kMsg &N2kMsg);
void OutsideEnvironmental(const tN2kMsg &N2kMsg);
void Temperature(const tN2kMsg &N2kMsg);
void TemperatureExt(const tN2kMsg &N2kMsg);
void DCStatus(const tN2kMsg &N2kMsg);
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg);
void COGSOG(const tN2kMsg &N2kMsg);
void GNSS(const tN2kMsg &N2kMsg);
void Attitude(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
		{126992L,&SystemTime},
		{127257L,&Attitude},
		{127488L,&EngineRapid},
		{127489L,&EngineDynamicParameters},
		{127493L,&TransmissionParameters},
		{127505L,&FluidLevel},
		{127506L,&DCStatus},
		{127513L,&BatteryConfigurationStatus},
		{128267L,&WaterDepth},
		{129026L,&COGSOG},
		{129029L,&GNSS},
		{130310L,&OutsideEnvironmental},
		{130312L,&Temperature},
		{130316L,&TemperatureExt},
		{127250L,&Heading},
		{0,0}
};
#define SERIALIN Serial
#define SERIALOUT Serial1
#define NMEA0183SourcePC 1

tActisenseReader ActisenseReader;
tNMEA0183 NMEA0183_out;
tBoatData BoatData;

void setup() {

	// Input is from a PC on USB Serial port sending NMEA2k data in Actisense format
	// Using NMEA Simulator to generate data.

	SERIALIN.begin(115200); // Data from PC - Actisense NMEA2k format
	ActisenseReader.SetReadStream(&SERIALIN);
	ActisenseReader.SetMsgHandler(HandleNMEA2000Msg);

	// Output is to a serial port (Serial1) connected to another PC
	// via a USB adaptor.

	NMEA0183_out.Begin(&SERIALOUT, NMEA0183SourcePC, 115200 );
}

//*****************************************************************************
template<typename T> void PrintVal(const char* label, T val, double (*ConvFunc)(double val)=0, bool AddLf=false ) {
	SERIALOUT.print(label);
	if (!N2kIsNA(val)) {
		if (ConvFunc) { SERIALOUT.print(ConvFunc(val)); } else { SERIALOUT.print(val); }
	} else SERIALOUT.print("not available");
	if (AddLf) SERIALOUT.println();
}

//*****************************************************************************
time_t convertN2kTime (uint16_t &DaysSince1970, double &SecondsSinceMidnight) {
	return DaysSince1970 * SECS_PER_DAY + floor(SecondsSinceMidnight);
}
void printTime (time_t &DateTime) {
	tmElements_t tm;
	char tmp[20];
	breakTime(DateTime, tm);
	sprintf (tmp, "%02u/%02u/%u %02u:%02u:%02u", tm.Day, tm.Month, tm.Year+1970, tm.Hour, tm.Minute, tm.Second);
	SERIALOUT.print (tmp);
}

//*****************************************************************************
void SystemTime(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	uint16_t SystemDate;
	double SystemTime;
	tN2kTimeSource TimeSource;

	if (ParseN2kSystemTime(N2kMsg,SID,SystemDate,SystemTime,TimeSource) ) {
		time_t dt = convertN2kTime(SystemDate, SystemTime);
		setTime (dt);
#if DISPLAYDATA

		PrintVal("System time: ",SID,0,true);
		PrintVal("  days since 1.1.1970: ",SystemDate,0,true);
		PrintVal("  seconds since midnight: ",SystemTime,0,true);
		SERIALOUT.print("  time source: "); PrintN2kEnumType(TimeSource,&SERIALOUT);
		SERIALOUT.print("  converted time: "); printTime (dt); SERIALOUT.println();
#endif
	} else {
		SERIALOUT.print("Failed to parse PGN: "); SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void EngineRapid(const tN2kMsg &N2kMsg) {
	unsigned char EngineInstance;
	double EngineSpeed;
	double EngineBoostPressure;
	int8_t EngineTiltTrim;

	if (ParseN2kEngineParamRapid(N2kMsg,EngineInstance,EngineSpeed,EngineBoostPressure,EngineTiltTrim) ) {
		PrintVal("Engine rapid params: ",EngineInstance,0,true);
		PrintVal("  RPM: ",EngineSpeed,0,true);
		PrintVal("  boost pressure (Pa): ",EngineBoostPressure,0,true);
		PrintVal("  tilt trim: ",EngineTiltTrim,0,true);
	} else {
		SERIALOUT.print("Failed to parse PGN: "); SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void EngineDynamicParameters(const tN2kMsg &N2kMsg) {
	unsigned char EngineInstance;
	double EngineOilPress;
	double EngineOilTemp;
	double EngineCoolantTemp;
	double AltenatorVoltage;
	double FuelRate;
	double EngineHours;
	double EngineCoolantPress;
	double EngineFuelPress;
	int8_t EngineLoad;
	int8_t EngineTorque;

	if (ParseN2kEngineDynamicParam(N2kMsg,EngineInstance,EngineOilPress,EngineOilTemp,EngineCoolantTemp,
			AltenatorVoltage,FuelRate,EngineHours,
			EngineCoolantPress,EngineFuelPress,
			EngineLoad,EngineTorque) ) {
		PrintVal("Engine dynamic params: ",EngineInstance,0,true);
		PrintVal("  oil pressure (Pa): ",EngineOilPress,0,true);
		PrintVal("  oil temp (C): ",EngineOilTemp,&KelvinToC,true);
		PrintVal("  coolant temp (C): ",EngineCoolantTemp,&KelvinToC,true);
		PrintVal("  altenator voltage (V): ",AltenatorVoltage,0,true);
		PrintVal("  fuel rate (l/h): ",FuelRate,0,true);
		PrintVal("  engine hours (h): ",EngineHours,&SecondsToh,true);
		PrintVal("  coolant pressure (Pa): ",EngineCoolantPress,0,true);
		PrintVal("  fuel pressure (Pa): ",EngineFuelPress,0,true);
		PrintVal("  engine load (%): ",EngineLoad,0,true);
		PrintVal("  engine torque (%): ",EngineTorque,0,true);
	} else {
		SERIALOUT.print("Failed to parse PGN: "); SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void TransmissionParameters(const tN2kMsg &N2kMsg) {
	unsigned char EngineInstance;
	tN2kTransmissionGear TransmissionGear;
	double OilPressure;
	double OilTemperature;
	unsigned char DiscreteStatus1;

	if (ParseN2kTransmissionParameters(N2kMsg,EngineInstance, TransmissionGear, OilPressure, OilTemperature, DiscreteStatus1) ) {
		PrintVal("Transmission params: ",EngineInstance,0,true);
		SERIALOUT.print("  gear: "); PrintN2kEnumType(TransmissionGear,&SERIALOUT);
		PrintVal("  oil pressure (Pa): ",OilPressure,0,true);
		PrintVal("  oil temperature (C): ",OilTemperature,&KelvinToC,true);
		PrintVal("  discrete status: ",DiscreteStatus1,0,true);
	} else {
		SERIALOUT.print("Failed to parse PGN: "); SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void COGSOG(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	tN2kHeadingReference HeadingReference;
	double COG;
	double SOG;

	if (ParseN2kCOGSOGRapid(N2kMsg,SID,HeadingReference,COG,SOG) ) {
#if DISPLAYDATA
		PrintVal("COG/SOG: ",SID,0,true);
		SERIALOUT.print("  reference: "); PrintN2kEnumType(HeadingReference,&SERIALOUT);
		PrintVal("  COG (deg): ",COG,&RadToDeg,true);
		PrintVal("  SOG (m/s): ",SOG,0,true);
#endif
	} else {
		SERIALOUT.print("Failed to parse PGN: "); SERIALOUT.println(N2kMsg.PGN);
	}
	tNMEA0183Msg NMEA0183Msg;
	if (NMEA0183SetVTG ( NMEA0183Msg, "GP", COG, COG, SOG)) {
		NMEA0183_out.SendMessage (NMEA0183Msg);
	}

//	char msg [90];
//	NMEA0183BuildVTG(msg, "GP", COG, COG, SOG);
//	NMEA0183_out.SendMessage (msg, add_crlf);
	BoatData.COG = COG;
	BoatData.SOG = SOG;
	BoatData.COGSOGLastUpdate = now();
}

//*****************************************************************************
void GNSS(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	uint16_t DaysSince1970;
	double SecondsSinceMidnight;
	double Latitude;
	double Longitude;
	double Altitude;
	tN2kGNSStype GNSStype;
	tN2kGNSSmethod GNSSmethod;
	unsigned char nSatellites;
	double HDOP;
	double PDOP;
	double GeoidalSeparation;
	unsigned char nReferenceStations;
	tN2kGNSStype ReferenceStationType;
	uint16_t ReferenceSationID;
	double AgeOfCorrection;

	if (ParseN2kGNSS(N2kMsg,SID,DaysSince1970,SecondsSinceMidnight,
			Latitude,Longitude,Altitude,
			GNSStype,GNSSmethod,
			nSatellites,HDOP,PDOP,GeoidalSeparation,
			nReferenceStations,ReferenceStationType,ReferenceSationID,
			AgeOfCorrection) ) {
#if DISPLAYDATA
		PrintVal("GNSS info: ",SID,0,true);
		PrintVal("  days since 1.1.1970: ",DaysSince1970,0,true);
		PrintVal("  seconds since midnight: ",SecondsSinceMidnight,0,true);
		PrintVal("  latitude: ",Latitude,0,true);
		PrintVal("  longitude: ",Longitude,0,true);
		PrintVal("  altitude: (m): ",Altitude,0,true);
		SERIALOUT.print("  GNSS type: "); PrintN2kEnumType(GNSStype,&SERIALOUT);
		SERIALOUT.print("  GNSS method: "); PrintN2kEnumType(GNSSmethod,&SERIALOUT);
		PrintVal("  satellite count: ",nSatellites,0,true);
		PrintVal("  HDOP: ",HDOP,0,true);
		PrintVal("  PDOP: ",PDOP,0,true);
		PrintVal("  geoidal separation: ",GeoidalSeparation,0,true);
		PrintVal("  reference stations: ",nReferenceStations,0,true);
#endif
		tNMEA0183Msg NMEA0183Msg;
		time_t DateTime = convertN2kTime (DaysSince1970, SecondsSinceMidnight);
		char FAAMode, Status;
		switch (GNSSmethod) {
		case N2kGNSSm_noGNSS:
			FAAMode = 'N'; Status = 'V'; break;
		case N2kGNSSm_DGNSS:
			FAAMode = 'D'; Status = 'A'; break;
		case N2kGNSSm_GNSSfix:
			FAAMode = 'A'; Status = 'A'; break;
		case N2kGNSSm_PreciseGNSS:
			FAAMode = 'P'; Status = 'A';
		}

		if (NMEA0183SetRMC (NMEA0183Msg, "GP", Status, DateTime, Latitude, Longitude,
		                      BoatData.COG, BoatData.SOG, BoatData.Variation, FAAMode)){
			 NMEA0183_out.SendMessage (NMEA0183Msg);
		}
	} else {
		SERIALOUT.print("Failed to parse PGN: "); SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void OutsideEnvironmental(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	double WaterTemperature;
	double OutsideAmbientAirTemperature;
	double AtmosphericPressure;

	if (ParseN2kOutsideEnvironmentalParameters(N2kMsg,SID,WaterTemperature,OutsideAmbientAirTemperature,AtmosphericPressure) ) {
		PrintVal("Water temp: ",WaterTemperature,&KelvinToC);
		PrintVal(", outside ambient temp: ",OutsideAmbientAirTemperature,&KelvinToC);
		PrintVal(", pressure: ",AtmosphericPressure,0,true);
	} else {
		SERIALOUT.print("Failed to parse PGN: ");  SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void Temperature(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	unsigned char TempInstance;
	tN2kTempSource TempSource;
	double ActualTemperature;
	double SetTemperature;

	if (ParseN2kTemperature(N2kMsg,SID,TempInstance,TempSource,ActualTemperature,SetTemperature) ) {
		SERIALOUT.print("Temperature source: "); PrintN2kEnumType(TempSource,&SERIALOUT,false);
		PrintVal(", actual temperature: ",ActualTemperature,&KelvinToC);
		PrintVal(", set temperature: ",SetTemperature,&KelvinToC,true);
	} else {
		SERIALOUT.print("Failed to parse PGN: ");  SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void TemperatureExt(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	unsigned char TempInstance;
	tN2kTempSource TempSource;
	double ActualTemperature;
	double SetTemperature;

	if (ParseN2kTemperatureExt(N2kMsg,SID,TempInstance,TempSource,ActualTemperature,SetTemperature) ) {
		SERIALOUT.print("Temperature source: "); PrintN2kEnumType(TempSource,&SERIALOUT,false);
		PrintVal(", actual temperature: ",ActualTemperature,&KelvinToC);
		PrintVal(", set temperature: ",SetTemperature,&KelvinToC,true);
	} else {
		SERIALOUT.print("Failed to parse PGN: ");  SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg) {
	unsigned char BatInstance;
	tN2kBatType BatType;
	tN2kBatEqSupport SupportsEqual;
	tN2kBatNomVolt BatNominalVoltage;
	tN2kBatChem BatChemistry;
	double BatCapacity;
	int8_t BatTemperatureCoefficient;
	double PeukertExponent;
	int8_t ChargeEfficiencyFactor;

	if (ParseN2kBatConf(N2kMsg,BatInstance,BatType,SupportsEqual,BatNominalVoltage,BatChemistry,BatCapacity,BatTemperatureCoefficient,PeukertExponent,ChargeEfficiencyFactor) ) {
		PrintVal("Battery instance: ",BatInstance,0,true);
		SERIALOUT.print("  - type: "); PrintN2kEnumType(BatType,&SERIALOUT);
		SERIALOUT.print("  - support equal.: "); PrintN2kEnumType(SupportsEqual,&SERIALOUT);
		SERIALOUT.print("  - nominal voltage: "); PrintN2kEnumType(BatNominalVoltage,&SERIALOUT);
		SERIALOUT.print("  - chemistry: "); PrintN2kEnumType(BatChemistry,&SERIALOUT);
		PrintVal("  - capacity (Ah): ",BatCapacity,&CoulombToAh,true);
		PrintVal("  - temperature coefficient (%): ",BatTemperatureCoefficient,0,true);
		PrintVal("  - peukert exponent: ",PeukertExponent,0,true);
		PrintVal("  - charge efficiency factor (%): ",ChargeEfficiencyFactor,0,true);
	} else {
		SERIALOUT.print("Failed to parse PGN: "); SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void DCStatus(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	unsigned char DCInstance;
	tN2kDCType DCType;
	unsigned char StateOfCharge;
	unsigned char StateOfHealth;
	double TimeRemaining;
	double RippleVoltage;

	if (ParseN2kDCStatus(N2kMsg,SID,DCInstance,DCType,StateOfCharge,StateOfHealth,TimeRemaining,RippleVoltage) ) {
		SERIALOUT.print("DC instance: ");
		SERIALOUT.println(DCInstance);
		SERIALOUT.print("  - type: "); PrintN2kEnumType(DCType,&SERIALOUT);
		SERIALOUT.print("  - state of charge (%): "); SERIALOUT.println(StateOfCharge);
		SERIALOUT.print("  - state of health (%): "); SERIALOUT.println(StateOfHealth);
		SERIALOUT.print("  - time remaining (h): "); SERIALOUT.println(TimeRemaining/60);
		SERIALOUT.print("  - ripple voltage: "); SERIALOUT.println(RippleVoltage);
	} else {
		SERIALOUT.print("Failed to parse PGN: ");  SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
void WaterDepth(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	double DepthBelowTransducer;
	double Offset;

	if (ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset) ) {
		if ( N2kIsNA(Offset) ) {
			PrintVal("Depth below transducer",DepthBelowTransducer);
			SERIALOUT.println(", offset not available");
		} else {
			if (Offset>0) {
				SERIALOUT.print("Water depth:");
			} else {
				SERIALOUT.print("Depth below keel:");
			}
			if ( N2kIsNA(DepthBelowTransducer) ) {
				SERIALOUT.println(DepthBelowTransducer+Offset);
			} else {  SERIALOUT.println(" not available"); }
		}
	}
}

//*****************************************************************************
void FluidLevel(const tN2kMsg &N2kMsg) {
	unsigned char Instance;
	tN2kFluidType FluidType;
	double Level=0;
	double Capacity=0;

	if (ParseN2kFluidLevel(N2kMsg,Instance,FluidType,Level,Capacity) ) {
		switch (FluidType) {
		case N2kft_Fuel:
			SERIALOUT.print("Fuel level :");
			break;
		case N2kft_Water:
			SERIALOUT.print("Water level :");
			break;
		case N2kft_GrayWater:
			SERIALOUT.print("Gray water level :");
			break;
		case N2kft_LiveWell:
			SERIALOUT.print("Live well level :");
			break;
		case N2kft_Oil:
			SERIALOUT.print("Oil level :");
			break;
		case N2kft_BlackWater:
			SERIALOUT.print("Black water level :");
			break;
		}
		SERIALOUT.print(Level); SERIALOUT.print("%");
		SERIALOUT.print(" ("); SERIALOUT.print(Capacity*Level/100); SERIALOUT.print("l)");
		SERIALOUT.print(" capacity :"); SERIALOUT.println(Capacity);
	}
}

//*****************************************************************************
void Attitude(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	double Yaw;
	double Pitch;
	double Roll;

	if (ParseN2kAttitude(N2kMsg,SID,Yaw,Pitch,Roll) ) {
		PrintVal("Attitude: ",SID,0,true);
		PrintVal("  Yaw (deg): ",Yaw,&RadToDeg,true);
		PrintVal("  Pitch (deg): ",Pitch,&RadToDeg,true);
		PrintVal("  Roll (deg): ",Roll,&RadToDeg,true);
	} else {
		SERIALOUT.print("Failed to parse PGN: "); SERIALOUT.println(N2kMsg.PGN);
	}
}
//*****************************************************************************
// Vessel Heading
// Output:
//  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
//                          to indicate that they are measured at same time.
//  - Heading               Heading in radians 
//  - Deviation             Magnetic deviation in radians. Use N2kDoubleNA for undefined value.
//  - Variation             Magnetic variation in radians. Use N2kDoubleNA for undefined value.
//  - ref                   Heading reference. See definition of tN2kHeadingReference.
void Heading(const tN2kMsg &N2kMsg){
	unsigned char SID;
	double Heading;
	double Deviation;
	double Variation;
	tN2kHeadingReference Ref;
	tNMEA0183Msg NMEA0183Msg;

	if (ParseN2kHeading (N2kMsg, SID, Heading, Deviation, Variation, Ref)) {
#if DISPLAYDATA
		PrintVal("Heading: ",SID,0,true);
		PrintVal("  Heading (rad): ", Heading, 0, true);
		PrintVal("  Heading (deg): ",Heading,&RadToDeg,true);
		PrintVal("  Deviation (deg): ",Deviation,&RadToDeg,true);
		PrintVal("  Variation (deg): ",Variation,&RadToDeg,true);
		SERIALOUT.print("  Ref: "); PrintN2kEnumType(Ref,&SERIALOUT);
#endif 
		bool result = false;
		if (Ref == N2khr_true) {
			result = NMEA0183SetHDT ( NMEA0183Msg, "HC", Heading);
			BoatData.TrueHeading = Heading;
			BoatData.HeadingLastUpdate = now();
		} else if (Ref == N2khr_magnetic) {
			result = NMEA0183SetHDM ( NMEA0183Msg, "HC", Heading);
			BoatData.TrueHeading = Heading+Variation+Deviation;
			BoatData.HeadingLastUpdate = now();
		}
//		if (result) NMEA0183_out.SendMessage (NMEA0183Msg);
		result = NMEA0183SetHDG ( NMEA0183Msg, "HC", Heading, Deviation, Variation);
		if (result) NMEA0183_out.SendMessage (NMEA0183Msg);
	} else {
		SERIALOUT.print("Failed to parse PGN: "); SERIALOUT.println(N2kMsg.PGN);
	}
}

//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
#if SENDACTISENSE
	N2kStream *S1;
	S1 = &SERIALOUT;
//	N2kMsg.Print(S1);
	N2kMsg.SendInActisenseFormat(S1);
#else
#if DISPLAYPGN
	SERIALOUT.print("PGN: "); SERIALOUT.println(N2kMsg.PGN);
#endif
	int iHandler;
	for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);

	if (NMEA2000Handlers[iHandler].PGN!=0) {
		NMEA2000Handlers[iHandler].Handler(N2kMsg);
		while (!NMEA0183_out.kick()) delay (100);  //clear buffer before processing next
	}
#endif
}

//*****************************************************************************


void loop() {

	ActisenseReader.ParseMessages();

}

