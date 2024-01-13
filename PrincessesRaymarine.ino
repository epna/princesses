

//===============================================================================
//
//            Les princesses version du 28/12/2023
//            Olivier Galland
//
//===============================================================================



#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2kMaretron.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#define N2K_SOURCE 15
#include <SoftwareSerial.h>
#include <NmeaParserV2.h>
#include <AIS.h>
// #include "NMEA0183AIStoNMEA2000.h"  // Contains class, global variables and code !!!


#define sp Serial.print
#define spl Serial.println

// entrée GPS
#define MYPORT_TX 33  // D40
#define MYPORT_RX 26  // D41


const char *ssid = "LesPrincesses";
const char *password = "";
const uint16_t PORT = 50000;
const uint16_t PORTNAVIONICS = 40000;
IPAddress ipBroadcast(192, 168, 4, 255);
JsonArray aisSession; 


typedef struct {
  String trType;
  long trIndex;
  String trLibelle;
} trame;

trame trames[] = {
  { "GPRMC", 7, "SOG" },
  { "GPRMC", 8, "COG" },
  { "GPGLL", 1, "lat" },
  { "GPGLL", 3, "long" },
  { "", 0, "" }
};

//============================================



//============================================



void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
using tN2kSendFunction = void (*)();
// Structure for holding message sending information
struct tN2kSendMessage {
  tN2kSendFunction SendFunction;
  const char *const Description;
  tN2kSyncScheduler Scheduler;

  tN2kSendMessage(tN2kSendFunction _SendFunction, const char *const _Description, uint32_t /* _NextTime */
                  ,
                  uint32_t _Period, uint32_t _Offset, bool _Enabled)
    : SendFunction(_SendFunction),
      Description(_Description),
      Scheduler(_Enabled, _Period, _Offset) {}
  void Enable(bool state);
};

extern tN2kSendMessage N2kSendMessages[];
extern size_t nN2kSendMessages;

static bool EnableForward = false;
static tN2kScheduler NextStatsTime;


static int offsetTimeToSend = 3000;
static unsigned long timeToSend = millis();

StaticJsonDocument<3500> doc;
WiFiUDP udp;

EspSoftwareSerial::UART myPort;
NmeaParserV2 parser(myPort);


// Forward declarations for functions
void CheckLoopTime();
void OnN2kOpen();
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);


void getDAtaGPSParser() {

  if (myPort.available()) {
    if (parser.valid()) {

      for (int iTrame = 0; (trames[iTrame].trType.length() != 0); iTrame++) {
        if (String(parser.getField(0)) == trames[iTrame].trType) {
          doc[trames[iTrame].trLibelle] = parser.getField(trames[iTrame].trIndex);
        }
      }
      if (String(parser.getField(0)) == "AIVDM") {
        setAIS(parser.getRawStatement());
      }
    }
  }
}

void initPort() {
  myPort.begin(38400, SWSERIAL_7E1, MYPORT_RX, MYPORT_TX, false);
  if (!myPort) {  // If the object did not initialize, then its configuration is invalid
    spl("Invalid EspSoftwareSerial pin configuration, check config");
    while (1) {  // Don't continue with invalid configuratione
      delay(1000);
      spl("Port non démarré");
    }
  }
  spl(" EspSoftwareSerial pin configuration OK !!!");
}

void initWIFI() {
  if (!WiFi.softAP(ssid, password)) {
    spl("Soft AP creation failed.");
    while (1)
      ;
  }
  IPAddress myIP = WiFi.softAPIP();
  sp("AP IP address: ");
  spl(myIP);
  udp.begin(PORT);  // start listening on port 50000
}
// ==================================================================================== envoi d'un paquet UDP
void sendPacket() {  // avec nbchar en INT

  udp.beginPacket(ipBroadcast, PORT);
  serializeJson(doc, udp);
  udp.println();
  udp.endPacket();
  doc.clear();
  JsonArray aisSession = doc.createNestedArray("AIS");

}
// *****************************************************************************
void setup() {
  // Initialize serial port.
  Serial.begin(115200);
  while (!Serial) { delay(100); }
  NMEA2000.SetN2kCANSendFrameBufSize(250);
  NMEA2000.SetProductInformation("00000001",                       // Manufacturer's Model serial code
                                 100,                              // Manufacturer's product code
                                 "Marine Application Generation",  // Manufacturer's Model ID
                                 "1.1.2.35 (2002-05-22)",          // Manufacturer's Software version code
                                 "1.1.2.0 (2002-05-22)"            // Manufacturer's Model version
  );
  // Set device information
  NMEA2000.SetDeviceInformation(1,    // Unique number. Use e.g. Serial number.
                                132,  // Device function=Analog to NMEA 2000 Gateway. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25,   // Device class=Inter/Intranetwork Device. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046  // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );
  // Uncomment 3 rows below to see, what device will send to bus
  NMEA2000.SetForwardStream(&Serial);             // PC output to default serial port
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);  // Show in clear text. Leave uncommented for default Actisense format.
  NMEA2000.SetForwardOwnMessages(false);          // Do not print own messages.
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndSend, 22);
  NMEA2000.EnableForward(EnableForward);  // Disable all msg forwarding to USB (=Serial)
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();
  doc.clear(); 
  aisSession = doc.createNestedArray("AIS");
  //
  initWIFI();
  initPort();


}

// *****************************************************************************
void SendN2kMsg(const tN2kMsg &N2kMsg) {
  NMEA2000.SendMsg(N2kMsg);
}
// *****************************************************************************
// Function check is it time to send message. If it is, message will be sent and
// next send time will be updated.
// Function always returns next time it should be handled.
int64_t CheckSendMessage(tN2kSendMessage &N2kSendMessage) {
  if (N2kSendMessage.Scheduler.IsDisabled()) return N2kSendMessage.Scheduler.GetNextTime();

  if (N2kSendMessage.Scheduler.IsTime()) {
    N2kSendMessage.Scheduler.UpdateNextTime();
    N2kSendMessage.SendFunction();
  }

  return N2kSendMessage.Scheduler.GetNextTime();
}

// *****************************************************************************
// Function send enabled messages from tN2kSendMessage structure according to their
// period+offset.
void SendN2kMessages() {
  static uint64_t NextSend = 0;
  uint64_t Now = N2kMillis64();

  if (NextSend < Now) {
    uint64_t NextMsgSend;
    NextSend = Now + 2000;
    for (size_t i = 0; i < nN2kSendMessages; i++) {
      NextMsgSend = CheckSendMessage(N2kSendMessages[i]);
      if (NextMsgSend < NextSend) NextSend = NextMsgSend;
    }
  }
}


//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  //int iHandler;


  if (N2kMsg.PGN == 128267) {
    unsigned char SID;
    double DepthBelowTransducer;
    double Offset;
    ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset); 
    doc["depth"] = DepthBelowTransducer;
  }
  if (N2kMsg.PGN == 130306) {
    unsigned char SID;
    double WindSpeed; 
    double WindAngle;
    tN2kWindReference WindReference; 
    ParseN2kPGN130306(N2kMsg, SID, WindSpeed,WindAngle,WindReference);
    doc["AWS"]=((int)(100*msToKnots(WindSpeed)))/100; 
    doc["AWD"]=((int)(100*RadToDeg(WindAngle)))/100; 
  }

  if (N2kMsg.PGN == 130310) {
    unsigned char SID;
    double WaterTemperature, OutsideAmbientAirTemperature, AtmosphericPressure; 
    ParseN2kPGN130310(N2kMsg, SID, WaterTemperature,OutsideAmbientAirTemperature, AtmosphericPressure);
    doc["water"]=KelvinToC(WaterTemperature);
  }

  if (N2kMsg.PGN == 127250) {
      unsigned char SID;
      double Heading, Deviation, Variation; 
      tN2kHeadingReference ref; 
      ParseN2kPGN127250(N2kMsg, SID,Heading, Deviation, Variation, ref);
      doc["HDG"]=((int)(100*RadToDeg(Heading)))/100;
  }

}
// *****************************************************************************
void loop() {
  SendN2kMessages();

  // envoi des messages standards vers raymarine (COG ,SOG, Latitude, Longitude, batterie etc ...   )
  NMEA2000.ParseMessages();

  // send data AIS
  getDAtaGPSParser();
  // send data to UDP 50000 vers application android (tel et tablette)
  if (timeToSend + offsetTimeToSend < millis()) {
    timeToSend = millis();
    if (!doc.isNull()) sendPacket();
  };
}
// *****************************************************************************
void SendN2kSystemTime() {
  tN2kMsg N2kMsg;
  SetN2kSystemTime(N2kMsg, 1, 17555, 62000);
  SendN2kMsg(N2kMsg);
}
// *****************************************************************************
void SendN2kProductInformation() {
  NMEA2000.SendProductInformation();
}
// *****************************************************************************
void SendN2kIsoAddressClaim() {
  // Note that sometime NMEA Reader gets grazy, when ISO Address claim will be sent periodically.
  // If that happens, reopen NMEA Reader.
  NMEA2000.SendIsoAddressClaim();
}
// *****************************************************************************
void SendN2kDCBatStatus1() {
  tN2kMsg N2kMsg;
  SetN2kDCBatStatus(N2kMsg, 1, 12.12);
  SendN2kMsg(N2kMsg);
}
// *****************************************************************************
void SendN2kDCBatStatus2() {
  /// Donnée Afficheé Fonctionne normalement
  tN2kMsg N2kMsg;
  static unsigned char sid = 0;
  //    SetN2kDCBatStatus(N2kMsg, 0, 12.45, 5.08, CToKelvin(27.15));
  SetN2kPGN127508(N2kMsg, 0, 12.7, 11.11, N2kDoubleNA, sid++);
  if (sid > 252) sid = 0;
  SendN2kMsg(N2kMsg);
}


void setAIS(String trame) {
  char buf[200];
  //spl(trame);
  String newtrame = extractDatatAIS(trame);
  newtrame.toCharArray(buf, newtrame.length() + 1);
  AIS ais(buf);
  //spl("test======================================");
  //testAISBextend(trame);
  //spl("apres test ====================================");
  if (isClassAAIS(trame)) SendN2kAISClassAPosition(trame);
  else sendN2kAISClassBPosition(trame);
  //if (!doc.containsKey("AIS")) JsonArray aisSession = doc.createNestedArray("AIS");
  
  
  JsonObject myAIS = aisSession.createNestedObject();
  myAIS["MMSI"] = ais.get_mmsi();
  myAIS["long"] = transpose(ais.get_longitude(), true);
  myAIS["lat"] = transpose(ais.get_latitude(), true);
  myAIS["COG"] = ais.get_COG() / 1000;
  myAIS["SOG"] = ais.get_SOG() / 1000;

  toNavionics(trame);
}


bool isClassAAIS(String maTrame) {
  int position = maTrame.indexOf(",");
  position = maTrame.indexOf(",", position + 1);
  position = maTrame.indexOf(",", position + 1);
  position = maTrame.indexOf(",", position + 1);

  return (maTrame.substring(position + 1, position + 2) == "A");
}

void toNavionics(String maTrame) {
  const char *content = maTrame.c_str();
  udp.beginPacket(ipBroadcast, PORTNAVIONICS);
  //Serial.print("Navionics : ");
  //Serial.printf(content);
  udp.printf(content);
  udp.endPacket();
}

// *****************************************************************************
void SendN2kAISClassAPosition(String inputTrame) {

  tN2kMsg N2kMsg;
  char buf[200];
  //inputTrame = inputTrame.substring(14);
  inputTrame = extractDatatAIS(inputTrame);

  inputTrame.toCharArray(buf, inputTrame.length() + 1);

  AIS ais(buf);
  tN2kAISNavStatus monStatus;

  tN2kAISNavStatus navStatus;
  switch (int(ais.get_navStatus())) {
    case tN2kAISNavStatus::N2kaisns_Under_Way_Motoring:
      navStatus = tN2kAISNavStatus::N2kaisns_Under_Way_Motoring;
      break;
    case tN2kAISNavStatus::N2kaisns_At_Anchor:
      navStatus = tN2kAISNavStatus::N2kaisns_At_Anchor;
      break;
    case tN2kAISNavStatus::N2kaisns_Not_Under_Command:
      navStatus = tN2kAISNavStatus::N2kaisns_Not_Under_Command;
      break;
    case tN2kAISNavStatus::N2kaisns_Restricted_Manoeuverability:
      navStatus = tN2kAISNavStatus::N2kaisns_Restricted_Manoeuverability;
      break;
    case tN2kAISNavStatus::N2kaisns_Constrained_By_Draught:
      navStatus = tN2kAISNavStatus::N2kaisns_Constrained_By_Draught;
      break;
    case tN2kAISNavStatus::N2kaisns_Moored:
      navStatus = tN2kAISNavStatus::N2kaisns_Moored;
      break;
    case tN2kAISNavStatus::N2kaisns_Aground:
      navStatus = tN2kAISNavStatus::N2kaisns_Aground;
      break;
    case tN2kAISNavStatus::N2kaisns_Fishing:
      navStatus = tN2kAISNavStatus::N2kaisns_Fishing;
      break;
    case tN2kAISNavStatus::N2kaisns_Under_Way_Sailing:
      navStatus = tN2kAISNavStatus::N2kaisns_Under_Way_Sailing;
      break;
    case tN2kAISNavStatus::N2kaisns_Hazardous_Material_High_Speed:
      navStatus = tN2kAISNavStatus::N2kaisns_Hazardous_Material_High_Speed;
      break;
    case tN2kAISNavStatus::N2kaisns_Hazardous_Material_Wing_In_Ground:
      navStatus = tN2kAISNavStatus::N2kaisns_Hazardous_Material_Wing_In_Ground;
      break;
    case tN2kAISNavStatus::N2kaisns_AIS_SART:
      navStatus = tN2kAISNavStatus::N2kaisns_AIS_SART;
      break;
    default:
      navStatus = tN2kAISNavStatus::N2kaisns_At_Anchor;
      break;
  }

  SetN2kAISClassAPosition(N2kMsg,
                          1,  //
                          tN2kAISRepeat::N2kaisr_First,
                          ais.get_mmsi(),
                          transpose(ais.get_latitude(), true),
                          transpose(ais.get_longitude(), true),
                          ais.get_posAccuracy_flag(),
                          ais.get_raim_flag(),
                          ais.get_timeStamp(),
                          ais.get_COG() / 10,
                          ais.get_SOG() / 10,
                          N2kaischannel_A_VDL_reception,
                          ais.get_HDG(),
                          ais.get_rot(),
                          navStatus

  );
  //SetN2kAISClassAPosition(N2kMsg, 1, tN2kAISRepeat::N2kaisr_First, 123456789, 26.396, -80.075, 1, 1, 1, 20, 20, N2kaischannel_A_VDL_reception, 30, 0, tN2kAISNavStatus::N2kaisns_At_Anchor);
  SendN2kMsg(N2kMsg);
}
// *****************************************************************************
void SendN2kCOGSOGRapid() {
  tN2kMsg N2kMsg;
  double mSOG = doc["SOG"];
  double mCOG = doc["COG"];
  if (mSOG > 0 && mCOG > 0) {
    double COG = doc["COG"];
    double SOG = doc["SOG"];
    SetN2kCOGSOGRapid(N2kMsg, 1, N2khr_true, DegToRad(COG), KnotsToms(SOG));
    //SetN2kCOGSOGRapid(N2kMsg, 1, N2khr_true, DegToRad(57.6), 2.57);
    SendN2kMsg(N2kMsg);
  }
}


double transpose(double entree, boolean AIS) {
  if (AIS) entree = entree / 600000;
  double partieEntiere = int(entree);
  double partieDecimale = entree - partieEntiere;
  double reponse;
  if (AIS) reponse = partieEntiere + (partieDecimale);
  else reponse = partieEntiere + (partieDecimale / .6);
  return reponse;
}


void SendN2kGNSS() {
  tN2kMsg N2kMsg;
  double latitude = doc["lat"];
  double longitude = doc["long"];
  if (latitude > 0 && longitude > 0) {
    SetN2kGNSS(N2kMsg, 1, 17555, 62000, transpose(latitude / 100, false), transpose(longitude / 100, false), 10.5, N2kGNSSt_GPS, N2kGNSSm_GNSSfix, 12, 0.8, 0.5, 15, 1, N2kGNSSt_GPS, 15, 2);
    SendN2kMsg(N2kMsg);
  }
}
// *****************************************************************************
void SendPosition() {
  tN2kMsg N2kMsg;
  if (!doc.isNull()) {
    double latitude = doc["lat"];
    double longitude = doc["long"];
    SetN2kPGN129025(N2kMsg, latitude, longitude);
    SendN2kMsg(N2kMsg);
  }
}
// We add 300 ms as default for each offset to avoid failed sending at start.
// Message sending is synchronized to open. After open there is 250 ms address claiming time when
// message sending fails.
//#define AddSendPGN(fn, NextSend, Period, Offset, Enabled)   { fn, #fn, NextSend, Period, Offset + 300, Enabled }
#define AddSendPGN(fn, NextSend, Period, Offset, Enabled) \
  { fn, #fn, NextSend, Period, Offset + 300, Enabled }
tN2kSendMessage N2kSendMessages[] = {
  AddSendPGN(SendN2kIsoAddressClaim, 0, 5000, 0, false),      // 60928 Not periodic
  AddSendPGN(SendN2kSystemTime, 0, 1000, 0, true),            // 126992
  AddSendPGN(SendN2kProductInformation, 0, 5000, 60, false),  // 126996 (20) Not periodic
  AddSendPGN(SendN2kDCBatStatus1, 0, 1500, 30, true),         // 127508
  AddSendPGN(SendN2kDCBatStatus2, 0, 1500, 31, true),         // 127508
  AddSendPGN(SendN2kCOGSOGRapid, 0, 200, 0, true),            // 129026
  //AddSendPGN(SendN2kAISClassAPosition, 0, 5000, 80, false),   // 129038 (4) Not periodic
  //AddSendPGN(SendPosition, 0, 500, 40, false),  // 129205
  AddSendPGN(SendN2kGNSS, 0, 1500, 75, true)  // 129029 (7)
};
size_t nN2kSendMessages = sizeof(N2kSendMessages) / sizeof(tN2kSendMessage);

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
// We initialize here all messages next sending time. Since we use tN2kSyncScheduler all messages
// send offset will be synchronized to libary.
void OnN2kOpen() {
  for (size_t i = 0; i < nN2kSendMessages; i++) {
    if (N2kSendMessages[i].Scheduler.IsEnabled()) N2kSendMessages[i].Scheduler.UpdateNextTime();
  }
}

// *****************************************************************************
void tN2kSendMessage::Enable(bool state) {
  if (Scheduler.IsEnabled() != state) {
    if (state) {
      Scheduler.UpdateNextTime();
    } else {
      Scheduler.Disable();
    }
  }
}



String extractDatatAIS(String inputData) {
  String res = inputData;
  int position = inputData.indexOf(",");
  position = inputData.indexOf(",", position + 1);
  position = inputData.indexOf(",", position + 1);
  position = inputData.indexOf(",", position + 1);
  position = inputData.indexOf(",", position + 1);
  inputData = inputData.substring(position + 1);

  return inputData;
}
void sendN2kAISClassBPosition(String inputTrame) {

  tN2kMsg N2kMsg;
  char buf[200];
  //inputTrame = inputTrame.substring(14);

  inputTrame = extractDatatAIS(inputTrame);
  inputTrame.toCharArray(buf, inputTrame.length() + 1);
  AIS ais(buf);
  SetN2kAISClassBPosition(N2kMsg, 0, tN2kAISRepeat::N2kaisr_First, ais.get_mmsi(),
                          transpose(ais.get_latitude(), true), transpose(ais.get_longitude(), true), ais.get_posAccuracy_flag(), ais.get_raim_flag(),
                          ais.get_timeStamp(), ais.get_COG() / 10, ais.get_SOG() / 10,
                          ais.get_HDG(), tN2kAISUnit::N2kaisunit_ClassB_CS, ais.get_display_flag(), ais.get_dsc_flag(), ais.get_band_flag(), ais.get_msg22_flag(), tN2kAISMode::N2kaismode_Autonomous,
                          0);
  SendN2kMsg(N2kMsg);
}


void testAISBextend(String trameAIVDM) {

  char buf[200];
  //inputTrame = inputTrame.substring(14);
  trameAIVDM = extractDatatAIS(trameAIVDM);
  spl("kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk");
  trameAIVDM.toCharArray(buf, trameAIVDM.length() + 1);
  spl("iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii");
  AIS ais(buf);

  onType19(ais.get_mmsi(), ais.get_SOG(), ais.get_posAccuracy_flag(), ais.get_longitude(), ais.get_latitude(), ais.get_COG(), ais.get_HDG(), ais.get_shipname(), ais.get_numeric_type(),
           ais.get_to_bow(), ais.get_to_stern(), ais.get_to_port(), ais.get_to_starboard(), ais.get_timeStamp(), ais.get_shiptype(), ais.get_dte_flag(), true, ais.get_repeat(), ais.get_raim_flag());
           spl("tttttttttttttttttttttt");
}

void onType19(unsigned int _uMmsi, unsigned int _uSog, bool _bPosAccuracy, int _iPosLon, int _iPosLat,
              int _iCog, int _iHeading, const std::string &_strName, unsigned int _uType,
              unsigned int _uToBow, unsigned int _uToStern, unsigned int _uToPort,
              unsigned int _uToStarboard, unsigned int _timestamp, unsigned int _fixtype,
              bool _dte, bool _assigned, unsigned int _repeat, bool _raim) {

  //Serial.println("19");
  tN2kMsg N2kMsg;

  // PGN129040

  char Name[21];
  strncpy(Name, _strName.c_str(), sizeof(Name) - 1);
  Name[20] = 0;
  for (int i = strlen(Name); i < 20; i++) Name[i] = 32;

  N2kMsg.SetPGN(129040UL);
  N2kMsg.Priority = 4;
  N2kMsg.AddByte((_repeat & 0x03) << 6 | (19 & 0x3f));
  N2kMsg.Add4ByteUInt(_uMmsi);
  N2kMsg.Add4ByteDouble(_iPosLon / 600000.0, 1e-07);
  N2kMsg.Add4ByteDouble(_iPosLat / 600000.0, 1e-07);
  N2kMsg.AddByte((_timestamp & 0x3f) << 2 | (_raim & 0x01) << 1 | (_bPosAccuracy & 0x01));
  N2kMsg.Add2ByteUDouble(decodeCog(_iCog), 1e-04);
  N2kMsg.Add2ByteUDouble(KnotsToms(_uSog) / 10.0, 0.01);
  N2kMsg.AddByte(0xff);  // Regional Application
  N2kMsg.AddByte(0xff);  // Regional Application
  N2kMsg.AddByte(_uType);
  N2kMsg.Add2ByteUDouble(decodeHeading(_iHeading), 1e-04);
  N2kMsg.AddByte(_fixtype << 4);
  N2kMsg.Add2ByteDouble(_uToBow + _uToStern, 0.1);
  N2kMsg.Add2ByteDouble(_uToPort + _uToStarboard, 0.1);
  N2kMsg.Add2ByteDouble(_uToStarboard, 0.1);
  N2kMsg.Add2ByteDouble(_uToBow, 0.1);
  N2kMsg.AddStr(Name, 20);
  N2kMsg.AddByte((_dte & 0x01) | (_assigned & 0x01) << 1);
  N2kMsg.AddByte(0x00);
  N2kMsg.AddByte(0xff);  // Sequence ID (Not Available)

  NMEA2000.SendMsg(N2kMsg);
}

double decodeRot(int iRot) {
  //see https://gpsd.gitlab.io/gpsd/AIVDM.html#_type_5_static_and_voyage_related_data
  //and https://opencpn.org/wiki/dokuwiki/doku.php?id=opencpn:supplementary_software:nmea2000
  double rot = N2kDoubleNA;
  if (iRot == 127) rot = 10;
  else if (iRot == -127) rot = -10;
  else if (iRot == 0) rot = 0;
  else if (1 <= iRot && iRot <= 126) rot = iRot * iRot / 22.401289;
  else if (iRot >= -126 && iRot <= -1) rot = iRot * iRot / -22.401289;
  //rot now in deg/minute
  rot = DegToRad(rot) / 60.0;  //N"K expects rot in radian/s
  return rot;
}
double decodeCog(int iCog) {
  double cog = N2kDoubleNA;
  if (iCog >= 0 && iCog < 3600) {
    cog = DegToRad(iCog) / 10.0;
  }
  return cog;
}
double decodeHeading(int iHeading) {
  double heading = N2kDoubleNA;
  if (iHeading >= 0 && iHeading <= 359) {
    heading = DegToRad(iHeading);
  }
  return heading;
}
