//-------- Customise these values-----------
//---------Bluemix IBM Settings-------------
#define ORG "qnu3pg"
#define DEVICE_TYPE "Parqueos"
#define DEVICE_ID "ParqueoAereo"
#define TOKEN "9KX51PvwDFfh*@e*+?"
#define FAILTRESHOLD 150

//-------- Customise the above values --------
//---------Blurmix Topics---------------------
const char publishTopic[] = "iot-2/evt/status/fmt/json";
const char averageTopic[] = "iot-2/evt/average/fmt/json";
const char responseTopic[] = "iotdm-1/response";
const char manageTopic[] = "iotdevice-1/mgmt/manage";
const char updateTopic[] = "iotdm-1/device/update";
const char rebootTopic[] = "iotdm-1/mgmt/initiate/device/reboot";

//-----------Variables de Sensor 
//-----------Variables de Configuracion de Sensor------------
#define SERIALNUMBER "ESP0001"
#define MANUFACTURER "flatbox"
#define MODEL "ESP8266"
#define DEVICECLASS "Wifi"
#define DESCRIPTION "Wifinode"
#define HWVERSION "IOTREDBOARD"
#define DESCRIPTIVELOCATION "CAMPUSTEC"




