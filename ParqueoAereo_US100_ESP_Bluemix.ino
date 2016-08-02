// Librerias de ESP // MQTT/ JSON FORMAT data
#include <ESP8266WiFi.h>       //Libreira de ESPCORE ARDUINO
#include <PubSubClient.h>      // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <ArduinoJson.h>       // https://github.com/bblanchon/ArduinoJson/releases/tag/v5.0.7
//librerias de TIEMPO NTP
#include <TimeLib.h>           // TimeTracking
#include <WiFiUdp.h>           // UDP packet handling for NTP request
//Librerias de manejo de setup de redes 
#include <ESP8266WebServer.h>  //Libreira de html para ESP8266
#include <DNSServer.h>         //Libreria de DNS para resolucion de Nombres
#include <WiFiManager.h>       //https://github.com/tzapu/WiFiManager
//Librerias de Codigo de Sensor US100
#include <SoftwareSerial.h>    //Libreria de SoftwareSerial para recibir data del sensor
#include "settings.h"       //Libreria local que contiene valores configurables de conexion

ADC_MODE(ADC_VCC);

const int LEDPIN = 16;

int failed, sent, published; //variables de conteo de envios 

unsigned int HighLen = 0;       //variable para distancia maxima calculada
unsigned int LowLen  = 0;       //variable para distancia minima calculada
unsigned int Len_mm  = 0;       //variable para distancia promedio calculada
String Estado;                  //variable a la que se regresa el esatdo del sensor para envio en JSON

//connect RX (Pin 14 of Arduino digital IO) to Echo/Rx (US-100), TX (Pin 12 of Arduino digital IO) to Trig/Tx (US-100)
SoftwareSerial swSer(14, 12, false, 256);//se define el puerto serial Pin D5 (GPIO14) y D6 (GPIO12) en el board Rojo, la logica inversa (false) y el Buffer (256)

//Variables de Reloj para espera y envio de paquetes de MQTT
unsigned long Starttime;                //Variable que guarda el valor incial del reloj
unsigned long NEXTSENDTIME = 15*1000UL; //Variable que define el tiempo a trancurrir despues de inicializado el reloj para enviar el primer mensake de MQTT en microsegundos (10*1000UL)= 10segundos
unsigned long UPDATESENDTIME = 60*1000UL; //Variable que define el tiempo a trancurrir despues de inicializado el reloj para enviar el primer mensake de MQTT en microsegundos (10*1000UL)= 10segundos

//Variables de Servicio NTP
IPAddress timeServer(129, 6, 15, 28);           // direccion ip del time.nist.gov NTP server
const char* ntpServerName = "time.nist.gov";    //Nombre del servidor de NTP
boolean NTP = false;                            //Bandera que establece el estado inicial del valor de NTP
const int timeZone = -6;                        // valor de la zona Horaria de la cual tomamos la hora (-6) para centroamerica

WiFiUDP Udp;                    // Definicion de Cliente UDP
unsigned int localPort = 2390;  // Definicion de puerto para la ecucha de paquetes de udp (por defecto 2390) verificar que Gateway IP (AP) no tenga el puerto bloqueado en Firewall

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48;       // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE];   //buffer to hold incoming & outgoing packets

//------------Variables de SErvicio de Bluemix IBM---------------

char server[] = ORG ".messaging.internetofthings.ibmcloud.com";   //Servicio de IBM bluenix donde se envian los paquetes de mqtt
char authMethod[] = "use-token-auth";                             //forma de Autneticacion 
char token[] = TOKEN;                                             //Token de autenticacion (definir en Settins.h)
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;         //Cleinte de Atenticacion en servicio de Bluemix
int FWVERSION = 1;                                                //version del Firmware dentro del ESP (version Faltbox)
String NodeID = String(ESP.getChipId());                           //Variable Global que contiene la identidad del nodo (ChipID) o numero unico

//-------------- Variables de Codigo 
String ISO8601;                   //variable Global para almacenar la Fecha del NTP
String Latitud = "15.30";         //Variable Global para Configurar la Latitud del nodo
String Longitud = "-90.15";       //Variable Global para configurar la Longitud del nodo

//---------------------------------- Variables de medicion de parqueo
int MinDist = 2000;

//------------------se define una na funcion para identificar los mensajes de debugging -------------

#define DEBUGPRINT

#ifdef DEBUGPRINT
#define DEBUG_PRINT(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#endif


//-------- HandleUpdate function receive the data, parse and update the info --------//
void handleUpdate(byte* payload) { 
  StaticJsonBuffer<1024> jsonBuffer;                          //buffer para almacenar el Json
  JsonObject& root = jsonBuffer.parseObject((char*)payload);  
  if (!root.success()) {                                      
    DEBUG_PRINT (F("handleUpdate: payload parse FAILED"));  
    return;                                                   
  }
  DEBUG_PRINT(F("handleUpdate payload:"));                 
  root.prettyPrintTo(Serial);                                 
  DEBUG_PRINT();                                           
  JsonObject& d = root["d"];                                  
  JsonArray& fields = d["fields"];                            
  for (JsonArray::iterator it = fields.begin();              
       it != fields.end();                                    
       ++it) {                                                
    JsonObject& field = *it;                                 
    const char* fieldName = field["field"];                   
    if (strcmp (fieldName, "metadata") == 0) {                
      JsonObject& fieldValue = field["value"];
      if (fieldValue.containsKey("name")) {
        const char* nodeID = "";
        nodeID = fieldValue["name"];
        NodeID = String(nodeID);
        DEBUG_PRINT(F("NodeID:"));
        DEBUG_PRINT(NodeID);
      }
      if (fieldValue.containsKey("lat")) {
        const char* RLatSetting = "";
        RLatSetting = fieldValue["lat"];
        Latitud = String(RLatSetting);
        DEBUG_PRINT(F("Latitud:"));
        DEBUG_PRINT(Latitud);
      }
      if (fieldValue.containsKey("long")) {
        const char* RLongSetting = "";
        RLongSetting = fieldValue["long"];
        Longitud = String(RLongSetting);
        DEBUG_PRINT(F("Longitud:"));
        DEBUG_PRINT(Longitud);
      }
      if (fieldValue.containsKey("MinDist")) {
        MinDist = fieldValue["MinDist"];
        DEBUG_PRINT(F("MinDist:"));
        DEBUG_PRINT(MinDist);
      }
    }
    if (strcmp (fieldName, "deviceInfo") == 0) {
      JsonObject& fieldValue = field["value"];
      if (fieldValue.containsKey("fwVersion")) {
        FWVERSION = fieldValue["fwVersion"];
        DEBUG_PRINT(F("fwVersion:"));
        DEBUG_PRINT(FWVERSION);
      }
    }
  }
}

//-------- Callback function. Receive the payload from MQTT topic and translate it to handle  --------//

void callback(char* topic, byte* payload, unsigned int payloadLength) {
  DEBUG_PRINT(F("callback invoked for topic: "));
  DEBUG_PRINT(topic);
  if (strcmp (responseTopic, topic) == 0) {
    return; // just print of response for now
  }
  if (strcmp (rebootTopic, topic) == 0) {
    DEBUG_PRINT(F("Rebooting..."));
    ESP.restart();
  }
  if (strcmp (updateTopic, topic) == 0) {
    handleUpdate(payload);
  }
}

//-------- MQTT information --------//
WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);

//-------- mqttConnect fuunction. Connect to mqtt Server  --------//
void mqttConnect() {
  if (!!!client.connected()) {
    DEBUG_PRINT(F("Reconnecting MQTT client to "));
    DEBUG_PRINT(server);
    while (!!!client.connect(clientId, authMethod, token)) {
      DEBUG_PRINT(F("."));
      delay(500);
    }
    DEBUG_PRINT();
  }
}

//-------- initManageDevice function. suscribe topics, Send metadata and supports to bluemix --------//
void initManagedDevice() {
  if (client.subscribe("iotdm-1/response")) {
    DEBUG_PRINT(F("subscribe to responses OK"));
  }
  else {
   DEBUG_PRINT(F("subscribe to responses FAILED"));
  }
  if (client.subscribe(rebootTopic)) {
    DEBUG_PRINT(F("subscribe to reboot OK"));
  }
  else {
    DEBUG_PRINT(F("subscribe to reboot FAILED"));
  }
  if (client.subscribe("iotdm-1/device/update")) {
    DEBUG_PRINT(F("subscribe to update OK"));
  }
  else {
    DEBUG_PRINT(F("subscribe to update FAILED"));
  }
  
  int a = 0;
  
  if (a == 0) {
    StaticJsonBuffer<500> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonObject& d = root.createNestedObject("d");
    JsonObject& supports = d.createNestedObject("supports");
    supports["deviceActions"] = true;
    char buff[200];
    root.printTo(buff, sizeof(buff));
    DEBUG_PRINT(F("publishing device metadata:"));
    DEBUG_PRINT(buff);
    if (client.publish(manageTopic, buff)) {
      DEBUG_PRINT(F("device Publish ok"));
    }
    else {
      Serial.print(F("device Publish failed:"));
    }
    a = 1;
  }
  
  if (a == 1) {
    StaticJsonBuffer<1024> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonObject& d = root.createNestedObject("d");
    JsonObject& metadata = d.createNestedObject("metadata");
    metadata["name"] = NodeID;
    metadata["lat"] = Latitud;
    metadata["long"] = Longitud;
    metadata["MinDist"] = MinDist;
    char buff[1024];
    root.printTo(buff, sizeof(buff));
    DEBUG_PRINT(F("publishing device metadata:"));
    DEBUG_PRINT(buff);
    if (client.publish(manageTopic, buff)) {
      DEBUG_PRINT(F("device Publish ok"));
    } else {
      Serial.print(F("device Publish failed:"));
    }
    a = 2;
  }
}

//---------- timestamp functions-------//
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address){
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  ///Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  DEBUG_PRINT(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      DEBUG_PRINT(F("Receive NTP Response"));
      NTP = true;
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  DEBUG_PRINT(F("No NTP Response :-("));
  return 0; // return 0 if unable to get the time
}

void udpConnect() {
  DEBUG_PRINT(F("Starting UDP"));
  Udp.begin(localPort);
  Serial.print(F("Local port: "));
  DEBUG_PRINT(Udp.localPort());
  DEBUG_PRINT(F("waiting for sync"));
  setSyncProvider(getNtpTime);
}

void ISO8601TimeStampDisplay() {
  // digital clock display of the time
  ISO8601 = String (year(), DEC);
  if(month()<10){
     ISO8601 += "0";
  }
  ISO8601 += month();
  if(day()<10){
     ISO8601 += "0";
  }
  ISO8601 += day();
  ISO8601 += " ";
  if(hour()<10){
     ISO8601 += "0";
  }  
  ISO8601 += hour();
  ISO8601 += ":";
  if(minute()<10){
     ISO8601 += "0";
  }
  ISO8601 += minute();
  ISO8601 += ":";
  if(second()<10){
     ISO8601 += "0";
  }
  ISO8601 += second();
//  ISO8601 += "-06:00";
  DEBUG_PRINT(ISO8601);
}

time_t prevDisplay = 0; // when the digital clock was displayed

void checkTime () {
  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
      ISO8601TimeStampDisplay();
    }
  }
}

//--------  anager function. Configure the wifi connection if not connect put in mode AP--------//
void wifimanager() {
  WiFiManager wifiManager;
  DEBUG_PRINT(F("empezando"));
  if (!  wifiManager.autoConnect("flatwifi")) {
    if (!wifiManager.startConfigPortal("FlatWifi")) {
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  }
}

//------------------------------- setup  ---------------------------------------------//
void setup(){ 
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  //connect RX (Pin 0 of Arduino digital IO) to Echo/Rx (US-100), TX (Pin 1 of Arduino digital IO) to Trig/Tx (US-100) 
  Serial.begin(115200);  //set baudrate as 9600bps.
  DEBUG_PRINT(F("initializing Setup"));
  swSer.begin(9600);  //set baudrate as 9600bps.
  DEBUG_PRINT(F("Connection to Wifi"));
  
  while (WiFi.status() != WL_CONNECTED) {
    wifimanager();
  }
  DEBUG_PRINT(F("Connected to WiFi, Sync NTP time"));
  
  while (NTP == false) {
    udpConnect ();
  }
  
  DEBUG_PRINT(F("Time Sync, Connecting to mqtt sevrer"));
  mqttConnect();
  DEBUG_PRINT(F("Mqtt Connection Done!, sending Device Data"));
  initManagedDevice();
  DEBUG_PRINT(F("Finalizing Setup"));
  digitalWrite(LEDPIN, LOW);
  delay(100);  
}


void loop(){
  Starttime = millis();
  
  if (failed >= FAILTRESHOLD){
    failed =0;
    published =0;
    sent=0;    
    ESP.reset();
  }
  
  if (Starttime >= NEXTSENDTIME){
    mqttConnect();
    digitalWrite(LEDPIN, HIGH);
    DEBUG_PRINT(ReadUS100Sensor());//output the result to serial monitor
    Estado = ReadUS100Sensor();
    publishData(NodeID, ISO8601, Estado);
    NEXTSENDTIME = Starttime + 20*1000UL;
    digitalWrite(LEDPIN, LOW);     
  }
  
  if (Starttime >= UPDATESENDTIME){
    mqttConnect();
    String Msg = String ("MSGfailed" + failed); 
    DEBUG_PRINT(Msg);
    publishManageData(NodeID, published, failed);
    UPDATESENDTIME = Starttime + 30*60*1000UL;       
  }
  
}

//-------- publishData function. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//
void publishManageData (String Sid0, int env, int fail){
  float vdd = ESP.getVcc()/1000 ;
  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& metadata = d.createNestedObject("metadata");
  metadata["device_name"] = Sid0;
  metadata["Bateria"] = vdd;
  metadata["enviados"] = env;
  metadata["fallidos"] = fail;
  char buff[1024];
  root.printTo(buff, sizeof(buff));
  DEBUG_PRINT(F("publishing device metadata:"));
  DEBUG_PRINT(buff);
  if (client.publish(manageTopic, buff)) {
    DEBUG_PRINT(F("Manage Publish ok"));
     published ++;
     failed = 0; 
  }else {
    Serial.print(F("Manage Publish failed:"));
    failed ++;
  }
}


//-------- publishData function. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//

boolean publishData(String Sid, String tStamp, String Stado) {  
  checkTime();
  StaticJsonBuffer<200> jsonbuffer;
  JsonObject& root = jsonbuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& data = d.createNestedObject("data");
  data["device_name"] = Sid;
  data["estado"] = Stado;
  data["timestamp"] = ISO8601;
  char payload[100];
  root.printTo(payload, sizeof(payload));
  
  DEBUG_PRINT(F("publishing device metadata:"));
  DEBUG_PRINT(payload);
  sent ++;
  if (client.publish(publishTopic, payload, byte(sizeof(payload)))) {
    DEBUG_PRINT(F("Publish OK"));
    published ++;
    failed = 0; 
    return true;
  }
  else {
    DEBUG_PRINT(F("Publish FAILED"));
    failed ++;    
  }
}

String ReadUS100Sensor(){
  String estado;
  swSer.flush();     // clear receive buffer of serial port
  swSer.write(0X55); // trig US-100 begin to measure the distance
  delay(500);          //delay 500ms to wait result
  if(swSer.available() >= 2){     //when receive 2 bytes
    HighLen = swSer.read();                   //High byte of distance
    LowLen  = swSer.read();                   //Low byte of distance
    Len_mm  = HighLen*256 + LowLen;             //Calculate the distance
    if((Len_mm > 1) && (Len_mm < MinDist)){ //normal distance should between 1mm and 10000mm (1mm, 1m)
      estado = "ocupado";
      return estado;
    }else{
      estado = "desocupado";
      return estado;
    }
  }
}
