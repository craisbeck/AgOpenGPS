AU// Dualhead for AGopenGPS
// Private-use only! (you need to ask for a commercial-use)
// by Franz Husch  31.12.2020 in cooperation with Matthias Hammer
// and Valentin Ernst
// **** OTA is possible over Network  **** //
// Antennas cross to driveDirection on cabin symmetrical
// right Antenna is Rover (A) for position, left Antenna is MB (B) for heading
// Ntrip client for 5 Router or Handy hotspots or from AgopenGPS
// Progamm tries one time to connect,
// you can install a button from GND to PIN 4
// by pressing the button a new WiFi scan starts.
// PIN 2 you can add an LED for ntrip is received, have a look to the Photo
// in AgopenGPS  offset is ZERO
// WiFi_scan_Delay is the mount of sec, you will need start router or hotspot
// in Data Sources
///by Fix "OGI", Heading GPS "Dual", Heading Correction Source "None", Roll "OGI"
// in Neigung/Richtung
// Bei Fix "OGI", Richtung GPS "Dual", Richtungskorrecturquelle "keine", Neigung "OGI"
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   ++++++++++++++++++++++++++++++++++++to adjust++++++++++++++++++++++++++++++++++  */

byte Eth_myip[4] = { 192, 168, 1, 111 };//IP address to send UDP data to
//byte Eth_myip[4] = { 10, 0, 0, 22 };//IP address to send UDP data via router to tablett
int AntDistance = 0;       // distance between the two antennas in cm,+, 0 for automatic distance
double headingcorr = 90;     // right antenna A , left antenna B;
int tractorhight = 315;   // roll is in Position calculated, in AgOpenGPS mit 10cm
int WiFi_scan_Delay = 50;      // for router use 50 sec delay
//bool send_amatron_nmea = true;   // true for sending, false for not
bool send_amatron_nmea = false;    // true for sending, false for not
int send_Data_Via = 1;       // send Data via  0 = USB, 1 = Ethernet

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
//libraries -------------------------------

#include <math.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

//connection plan:
// ESP32--- Right F9P GPS pos --- Left F9P Heading-----Sentences
//  RX1----------TX1--------------------------------UBX-Nav-PVT out   (=position+speed)
//  TX1----------RX1--------------------------------RTCM in           (NTRIP comming from AOG to get absolute/correct postion
//  RX2--------------------------------TX1----------UBX-RelPosNED out (=position relative to other Antenna)
//  TX2--------------------------------RX1----------
//               RX2-------------------TX2----------RTCM 1005+1074+1084+1094+1124+1230 (activate in PVT F9P!! = NTRIP for relative positioning)

// TODO: UPDATE ABOVE
// 7    RX2 to TX1 of RelPosNED (Heading)
// 8    TX2 to RX1 of RelPosNED

// 28    RX1 to TX1 PVT (Position)
// 29    TX1 to RX1 PVT

//loop time variables in microseconds
long lastTime = 0;
float roll = 0;
byte XOR;
char c;
char b;
String t;
byte UDPPAOGIMsg[120];

// Ntrip
//NTRIPClient ntrip_c;
byte WiFi_netw_nr = 0; 
String RTCM_Packet;
unsigned long startSend_back_Time = millis();
unsigned long ntriptime_from_AgopenGPS = millis();
unsigned long WiFi_scan_Delay_Time = millis();
unsigned long Amatron_begin_Time = millis();
unsigned long GPSagetime = millis();
unsigned long Ntrip_begin_Time = millis();

bool OTA_update = false;
int wait = 30000, WiFi_scan_Versuch = 1;
int durchlauf_nord = 0, durchlauf_east = 0;
double nordWinkel_old, eastWinkel_old;

int ntrip_from_AgopenGPS = 0, ntriptime_from_ESP32 = 0;
bool network_found = false, NtripCheckTrue = false;
int buttonState = 0, Ntriphotspotoriginal = 1;
byte IPadress[4] = { 0, 0, 0, 0 };
int net_found = 0, Ntriphotspot_an = 0;
#define Button_ReScan 4  // pin 4  if button pressed, WiFi scan is starting
#define LED_ntrip_ON  2  // pin 2  if ntrip on without AGopenGPS


// Ethernet
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xB3, 0x1B}; // original

byte Eth_ipDest_ending = 100;//ending of IP address to send UDP data to
unsigned int portMy = 5544;             //this is port of this module: Autosteer = 5577 IMU = 5566 GPS =
unsigned int AOGNtripPort = 2233;       //port NTRIP data from AOG comes in
unsigned int portDestination = 9999;    //Port of AOG that listens
bool Ethernet_running = false;
char Eth_NTRIP_packetBuffer[512];// buffer for receiving and sending data
byte ReplyBufferPAOGI[120] = "";        // a string to send back
unsigned int Bytelaenge;
int m;

uint8_t udpNtripData[UDP_TX_PACKET_MAX_SIZE];  // Buffer For Receiving UDP Data
uint8_t udpData[UDP_TX_PACKET_MAX_SIZE];  // Buffer For Receiving UDP Data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
// An EthernetUDP instance to let us receive RTCM packets over UDP
EthernetUDP UdpRtcm;

IPAddress Eth_ipDestination;

// turn a round time
int Starttime = millis(), Countdurch = 0;
int Core1_time = millis(), Core1_break = 10000;

// Heading
double heading, headingUBX, headingzuvor = 0, headingzuvorVTG;
double headingUBXmin, headingUBXmax, headingVTGmin, headingVTGmax;
double speeed = 0, speeed1 = 0, headingnord, speeedzuvor = 0, speeedmin = 0.3, speeedmax = 0.3;


// roll
int rollaktiv = true;     // true roll activated
float rollCorrectionDistance = 0.00;
double rollnord = 0.0, rolleast = 0.0;
double rollnord_before = 0.0, rolleast_before = 0.0;
double relPosD, relPosDH;
double rollzuvor = 0;
double PI180 = 57.295791;
double baseline, baseline1, baselineHorizontal;

byte CK_A = 0, CK_B = 0;
byte incoming_char;

// NMEA erstellen
int inByte, start = 0, GGASats;
String nmea = "", GGAdaten = "", GGAdaten1 = "", VTGdaten = "", VTGspeed = "", VTGheadingnord = "";
String VTGSatz = "", GGASatz_old = "", GGAnord = "", GGAeast = "", GGAZeit = "", GGAWestEast = "", GGANordSued = "";
int j = 0, j2 = 0, jGGA = 0, jGGA2 = 0, jGGA3 = 0, jGGA4 = 0, jGGA5 = 0, jGGA6 = 0, jGGA7 = 0, jGGA78 = 0;
int jVTG1 = 0, jVTG2 = 0, jVTG3 = 0, jVTG4 = 0, jVTG5 = 0;
String GPSquali = "", WEcoordinaten, NScoordinaten, GGASat, GGAHDop;
String GGASatz = "", GGASatz_Korr, VTGSatz_Korr = "", GGASatz_send_back1 = "", GGASatz_send_back2 = "";
int GPSqualin1 = 0, GPSqualin2 = 1, GPSqualinzuvor = 1, GPSqualintime = 1, GGA_check = 0;
String ZDASatz = "", GGA_hDop, GGA_seahigh;
int  i = 0, ij = 0;
double GGAZeitNummerbevor, GGAZeitNummer;
double GGA_hDops, GGAage, GGA_seahighs;

unsigned long prev_PWM_Millis = 0;
byte velocityPWM_Pin = 12;      // Velocity (MPH speed) PWM pin
int velocityPWM_Multiplier = 10; // PWM (MPH * multiplier)

// PAOGI erstellen
bool Paogi_true = true;
String RollHeadingrest = "", RollHeadingrest_befor = "", BS =",";
int Paogi_Long, Coodinate_check1, Coodinate_check2, heading_check1 = 0;
int Paogi_Shit = 0;

// Chechsum controll
String checksum = "", checksum_GGA = "", checksum_GGA_send = "", checksum_VTG = "", checksum_VTG_send = "";
String check_headingroll = "";
int j_checksum_GGA = 0, j_checksum_VTG = 0;

union UBXMessage {
  struct {
    unsigned char HeaderA;
    unsigned char HeaderB;
    unsigned char cls;
    unsigned char id;
    unsigned short len;
    unsigned char reserved;
    unsigned char reserved1;
    unsigned short refStationId;
    unsigned long iTOW;
    long relPosN;
    long relPosE;
    long relPosD;
    long relPosLength;
    long relPosHeading;
    unsigned long reserved2;
    char relPosHPN;
    char relPosHPE;
    char relPosHPD;
    char relPosHPLength;
    unsigned long accN;
    unsigned long accE;
    unsigned long accD;
    unsigned long accLength;
    unsigned long accHeading;
    unsigned long reserved3;
    unsigned long flags;
  } relposned;
  byte rawBuffer[72];
} ubxmessage;

union UBXMessagePVT {
  struct {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    unsigned long iTOW;  //GPS time ms
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    unsigned long tAcc;
    long nano;
    uint8_t fixType;//0 no fix....
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV; //number of sats
    long lon;   //deg * 10^-7
    long lat;   //deg * 10^-7
    long height;
    long hMSL;  //heigt above mean sea level mm
    unsigned long hAcc;
    unsigned long vAcc;
    long velN;
    long velE;
    long velD;
    long gSpeed; //Ground Speed mm/s
    long headMot;
    unsigned long sAcc;
    unsigned long headAcc;
    uint16_t pDOP;
    uint8_t flags3;
    uint8_t reserved1[5];
    long headVeh;
    int16_t magDec;
    uint16_t magAcc;
    uint8_t CK0;
    uint8_t CK1;
  } PVT;
  byte rawBufferPVT[150];
} ubxmessagePVT;

//UBX
double speedUBXint;
String speedUBXstr;


// bool debugmode = true;  // GGA,VTG,
bool debugmode = false;
// bool debugmode1 = true;  // Heading
bool debugmode1 = false;
//  bool debugmode2 = true;  // Deviation
bool debugmode2 = false;
//  bool debugmode3 = true;  // roll
bool debugmode3 = false;
//bool debugmode4 = true;  //
bool debugmode4 = false;
//  bool debugProtokoll = true;  //Protocoll TestStation
bool debugProtokoll = false;
//bool debugmode_amatron = true;  //Protocoll Amatron
bool debugmode_amatron = false;

//AsyncUDP udpNtrip;
EthernetUDP Eth_udpPAOGI;
EthernetUDP Eth_udpNtrip;

int led = 13;

// Setup procedure ------------------------
void setup() {
  ubxmessage.rawBuffer[0] = 0xB5;
  ubxmessage.rawBuffer[1] = 0x62;
  ubxmessage.rawBuffer[2] = 0x01;
  ubxmessage.rawBuffer[3] = 0x3C;

  ubxmessagePVT.rawBufferPVT[0] = 0xB5;
  ubxmessagePVT.rawBufferPVT[1] = 0x62;
  ubxmessagePVT.rawBufferPVT[2] = 0x01;
  ubxmessagePVT.rawBufferPVT[3] = 0x07;

// TODO!!
 pinMode( velocityPWM_Pin, OUTPUT);
//   ledcSetup( 0, velocityPWM_Multiplier, 8 );
//   ledcAttachPin( velocityPWM_Pin, 0 );
//   ledcWrite(0, 128);

 pinMode(led, OUTPUT);

  delay(10);
  Serial.begin(115200);
  delay(10);
  Serial.println("Start setup");
  Serial7.begin(115200);
  delay(10);
  Serial2.begin(115200);
  Serial.println("Serial7 and Serial2 initialized");
//  Serial.println("Starting udp...");
//  // start UDP
//  Udp.begin(portMy);
//  Serial.println("Started udp");
  Serial.println("Starting Ethernet...");
  // start UdpRtcm
  Eth_Start();
  Serial.println("Started Ethernet...");
  Serial.println("End setup");
  Serial.println("");

  //debugmode = true;  // GGA,VTG,
  //debugmode1 = true;  // Heading
  //debugmode2 = true;  // Deviation
  //debugmode3 = true;  // roll
  //debugmode4 = true;  //
  //debugProtokoll = true;  //Protocoll TestStation
}


int cntr = 0;
int ledValue = HIGH;
int prevLedValue = LOW;


void loop() {
//  if (OTA_update) ArduinoOTA.handle();

   if (Serial.available()) {      // If RTCM3 comes in Serial (USB),
      char C = Serial.read();      // read a byte, then
      Serial7.write(C);            // send it out Serial7 from 16 to simpleRTK RX 1. Antenna = RTCM
      if (C != '  ') {             // if the byte is a newline character
        ntripcheck();
      }
      ntriptime_from_AgopenGPS = millis();
    }
    else {
      ntripcheck();
    }
        
    if (Serial7.available()) { // If anything comes in Serial7
      inByte = Serial7.read(); // read it and send for NMEA_PAOGI
      //Serial.println("Serial7 available");
      //Serial.write(inByte);
      NMEA_read();
    }

    if (Serial2.available()) {         // If anything comes in Serial2
      //Serial.println("Serial2 available");
      incoming_char = Serial2.read();  // ESP32 read RELPOSNED from F9P
      //Serial.write(incoming_char);
    
      if (i < 4 && incoming_char == ubxmessage.rawBuffer[i]) {
        i++;
      }
      else if (i > 3) {
        ubxmessage.rawBuffer[i] = incoming_char;
        i++;
      }
    }
    
    if (i > 71) {
      CK_A = 0;
      CK_B = 0;
      for (i = 2; i < 70 ; i++) {
        CK_A = CK_A + ubxmessage.rawBuffer[i];
        CK_B = CK_B + CK_A;
      }

      if (CK_A == ubxmessage.rawBuffer[70] && CK_B == ubxmessage.rawBuffer[71]) {
        rollundheading();
        PAOGI_builder();
      }
      else {
        // Serial.println("ACK Checksum Failure: ");
      }
      i = 0;
    }
    doEthUDPNtrip();
 }
