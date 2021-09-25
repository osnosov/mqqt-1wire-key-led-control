#include <Arduino.h>
#include <UIPEthernet.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <EEPROM.h>

#define ONEWIRE_PIN  (8)

#define DS2413_FAMILY_ID    0xBA  //0x3A у китайского 0x85  //BA 35 D4 73 50 05 10 76
#define DS2413_ACCESS_READ  0xF5
#define DS2413_ACCESS_WRITE 0x5A
#define DS2413_ACK_SUCCESS  0xAA
#define DS2413_ACK_ERROR    0xFF
#define IOA 0x1
#define IOB 0x4

#define MACADDRESS 0xDE,0xAD,0xBE,0xAA,0xFE,0xAE
#define MYIPADDR 192,168,10,9
#define MYIPMASK 255,255,255,0
#define MYDNS 192,168,10,1
#define MYGW 192,168,10,1

struct Sensor
{
  byte addr[8];
  boolean valid;
  uint8_t power;
  uint8_t type; //0-key; 1-led;
};

Sensor AD[] = {
  {{0xBA,0x35,0xD4,0x73,0x50,0x05,0x10,0x76},false,true,1}, 
  {{0xBA,0x3E,0xD1,0x73,0x50,0x05,0x10,0x5C},false,true,1}, 
  {{0xBA,0x06,0xD1,0x73,0x50,0x05,0x10,0x10},false,true,1}, 
  {{0xBA,0x24,0xD3,0x73,0x50,0x05,0x10,0x4B},false,true,1}, 
  {{0xBA,0x44,0xC7,0x73,0x50,0x05,0x10,0xEB},false,true,1}, 
  {{0xBA,0x44,0xDA,0x73,0x50,0x05,0x10,0x7B},false,true,1},
  {{0xBA,0x97,0xDF,0x73,0x50,0x05,0x10,0x34},false,true,0},
  {{0xBA,0x76,0xCF,0x73,0x50,0x05,0x10,0x56},false,true,0},
  {{0xBA,0xEE,0xE1,0x73,0x50,0x05,0x10,0x1C},false,true,0},
  {{0xBA,0x27,0xDB,0x73,0x50,0x05,0x10,0x2C},false,true,0},
  {{0xBA,0xA3,0xD5,0x73,0x50,0x05,0x10,0xB8},false,true,0},
  {{0xBA,0x31,0xCA,0x73,0x50,0x05,0x10,0x74},false,true,1},
  {{0xBA,0xE1,0xDA,0x73,0x50,0x05,0x10,0xCC},false,true,1},
  {{0xBA,0x83,0xDC,0x73,0x50,0x05,0x10,0xFD},false,true,1},
  {{0xBA,0xA8,0xD2,0x73,0x50,0x05,0x10,0x11},false,true,1},
  {{0xBA,0x5E,0xE1,0x73,0x50,0x05,0x10,0x1B},false,true,0},
  {{0xBA,0xF8,0xC5,0x73,0x50,0x05,0x10,0x12},false,true,0},
  {{0xBA,0x53,0xC8,0x73,0x50,0x05,0x10,0x5A},false,true,0},
  {{0xBA,0xB8,0xDD,0x73,0x50,0x05,0x10,0x25},false,true,0}
};

uint8_t numberDevices;

uint8_t masKey[19*2];
uint8_t statusKeyBefore[19*2];
uint8_t statusKeyAfter[19*2];

OneWire oneWire(ONEWIRE_PIN);
uint8_t address[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

#define DEVICE_NAME       "1-wire key led controller"

// MQTT server information
//#define MQTT_SERVER "192.168.100.120"
//#define MQTT_PORT 1883
//#define MQTT_USER "Project"
//#define MQTT_PASS "Project"

#define MQTT_CLIENT_ID       "LightController"
#define INTERVAL_SEND_MQTT_DATA        3000 // 3 sec delay between publishing
#define INTERVAL_RECONECT_MQTT_SERVER        3000 // 3 sec delay between publishing
#define INTERVAL_SEND_DATA_SENSOR        3000

  uint8_t mac[6] = {MACADDRESS};
  uint8_t myIP[4] = {MYIPADDR};
  uint8_t myMASK[4] = {MYIPMASK};
  uint8_t myDNS[4] = {MYDNS};
  uint8_t myGW[4] = {MYGW};
  
IPAddress mqttServer(192,168,10,20);
uint16_t  mqttPort = 1883;

// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

EthernetClient ethClient;
PubSubClient mqttClient(mqttServer, mqttPort, callback, ethClient);

long lastTimeSendDataMqtt = 0;
long lastReconnectMqtt = 0;
long lastSendDataSensor = 0;

uint8_t drebezg = 3;

boolean stateEthernetConnected = false;
boolean statusConnectMqtt = false;

void printBytes(uint8_t* addr, uint8_t count, bool newline=0) 
{
  for (uint8_t i = 0; i < count; i++) 
  {
    Serial.print(addr[i]>>4, HEX);
    Serial.print(addr[i]&0x0f, HEX);
    Serial.print(" ");
  }
  if (newline)
  {
    Serial.println();
  }
}

boolean compare(uint8_t* addr1, uint8_t* addr2)
{
  boolean valid = true;
  for (uint8_t i = 0; i < 8; i++) 
  {
    if (addr1[i] != addr2[i]){
      valid = false;
    }
  }
  return valid;
}


byte read(uint8_t* addr)
{    
  // bool ok = false;
  uint8_t results;

  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(DS2413_ACCESS_READ);

  results = oneWire.read();                 /* Get the register results   */
  // ok = (!results & 0x0F) == (results >> 4); /* Compare nibbles            */
  results &= 0x0F;                          /* Clear inverted values      */

  oneWire.reset();
  
  // return ok ? results : -1;
  return results;
}

bool write(uint8_t* addr, uint8_t state)
{
  uint8_t ack = 0;
  /* Top six bits must '1' */
  state |= 0xFC;
  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(DS2413_ACCESS_WRITE);
  oneWire.write(state);
  oneWire.write(~state);                    /* Invert data and resend     */    
  ack = oneWire.read();                     /* 0xAA=success, 0xFF=failure */  
  if (ack == DS2413_ACK_SUCCESS)
  {
    oneWire.read();                          /* Read the status byte      */
  }
  oneWire.reset();
  return (ack == DS2413_ACK_SUCCESS ? true : false);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (uint8_t i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  String strTopic(topic);
//  String strTopic = String(topic);
  uint8_t power = 0;

  payload[length] = '\0';
  String strPayload = String((char*)payload);
  if (strPayload == "1") power = 1;
  else power = 0;

  if(strTopic == "led"){
    for (int i = 0; i < numberDevices; i++){
      if((AD[i].valid == true) && (AD[i].type == 1)){
        AD[i].power = power;
        printBytes(AD[i].addr, 8);
        Serial.print(F("callback. all Led: "));
        Serial.print(i);
        Serial.print(F(". Status: "));
        Serial.print(AD[i].power);
        Serial.print(F(" "));
        if (!write(AD[i].addr, !AD[i].power)) Serial.println(F("Wire failed write"));
        else Serial.println(F("Write Ok"));
      }
    }
  } else {
    
    for (int i = 0; i < numberDevices; i++){
      String myString = String(i);
      String myTopic = "led/" + myString;// + "\n";
      if((strTopic == myTopic) && (AD[i].valid == true) && (AD[i].type == 1)){
        AD[i].power = power;
        printBytes(AD[i].addr, 8);
        Serial.print(F("callback. on Led: "));
        Serial.print(i);
        Serial.print(F(". Status: "));
        Serial.print(AD[i].power);
        Serial.print(F(" "));
        if (!write(AD[i].addr, !AD[i].power)) Serial.println(F("Wire failed write"));
        else Serial.println(F("Write Ok"));
      }
    }

    for (int i = 0; i < numberDevices; i++){
      String myStringA = String(i*2);
      String myStringB = String(i*2+1);
      String myTopicA = "key/" + myStringA;
      String myTopicB = "key/" + myStringB;

      if((strTopic == myTopicA) && (AD[i].valid == true) && (AD[i].type == 0)){
        if (masKey[i*2] < drebezg) {
          if (strPayload == "1") {
            statusKeyAfter[i*2] = 1;
            statusKeyBefore[i*2] = 1;
          }
          if (strPayload == "0") {
            statusKeyAfter[i*2] = 0;
            statusKeyBefore[i*2] = 0;
          }
        } else {
          if (strPayload == "1") {
            statusKeyAfter[i*2] = 1;
            statusKeyBefore[i*2] = 0;
          } 
          if (strPayload == "0") {
            statusKeyAfter[i*2] = 0;
            statusKeyBefore[i*2] = 1;
          }
        }
      }

      if((strTopic == myTopicB) && (AD[i].valid == true) && (AD[i].type == 0)){
        if (masKey[i*2+1] < drebezg) {
          if (strPayload == "1") {
            statusKeyAfter[i*2+1] = 1;
            statusKeyBefore[i*2+1] = 1;
          }
          if (strPayload == "0") {
            statusKeyAfter[i*2+1] = 0;
            statusKeyBefore[i*2+1] = 0;
          }
        } else {
          if (strPayload == "1") {
            statusKeyAfter[i*2+1] = 1;
            statusKeyBefore[i*2+1] = 0;
          } 
          if (strPayload == "0") {
            statusKeyAfter[i*2+1] = 0;
            statusKeyBefore[i*2+1] = 1;
          }
        }
      }


    }

    
  } 
}

void sendDataMqtt(uint8_t t, uint8_t p) {
  String myString = String(t);
  String topic = "key/" + myString;
  String payload = String(p);
  
      Serial.print("sendDataMqtt topic=");
      Serial.print(topic);
      Serial.print(". payload=");
      Serial.print(payload);
      Serial.print(". ");

  char this_topic[topic.length() + 1];
  topic.toCharArray(this_topic, sizeof(this_topic));
  
  char this_payload[payload.length() + 1];
  payload.toCharArray(this_payload, sizeof(this_payload));

//  Serial.print("this_topic ");
//  Serial.println(this_topic);
//  Serial.print(F(" this_payload "));
//  Serial.println(this_payload);

  if (stateEthernetConnected) {
    if (mqttClient.connected()) {
      mqttClient.publish(this_topic, this_payload);
    } else Serial.println("Not Mqtt Server Connected");
  } else Serial.println("Not Ethernet Connected");
}


//void sendDataMqtt() {
//  Serial.println("Send data statusLed OPEN");
//  mqttClient.publish("statusLed", "OPEN");
//}

void sendAllDataMqtt() {
  for (int i = 0; i < numberDevices; i++){
    if (AD[i].type == 0 && AD[i].valid == true){ 
      sendDataMqtt(i*2, statusKeyAfter[i*2]);
      sendDataMqtt(i*2+1, statusKeyAfter[i*2+1]);
    }  
  }
}

void readSensor(){
  for (int i = 0; i < numberDevices; i++){
    if (AD[i].type == 0 && AD[i].valid == true) {
      //Serial.print(F("Чтение выключателя i = "));
      //Serial.print(i);
      //Serial.print(F("; stateEthernetConnected = "));
      //Serial.println(stateEthernetConnected);
      byte state = read(AD[i].addr);
      if (state == -1) {
        Serial.print(F("Failed reading the DS2413 nomber "));
        Serial.println(i);
      } else { 
        if ((state & IOA) == 0) {
          if (masKey[i*2] < drebezg) masKey[i*2]++;
          else {
            if (statusKeyBefore[i*2] == statusKeyAfter[i*2]) {
              if (statusKeyAfter[i*2]) statusKeyAfter[i*2] = 0;
              else statusKeyAfter[i*2] = 1;
              sendDataMqtt(i*2, statusKeyAfter[i*2]);
            }
          }
        } else {
          masKey[i*2] = 0;
          statusKeyBefore[i*2] = statusKeyAfter[i*2];
        }
        
        if ((state & IOB) == 0) {
          if (masKey[i*2+1] < drebezg) masKey[i*2+1]++;
          else {
            if (statusKeyBefore[i*2+1] == statusKeyAfter[i*2+1]) {
              if (statusKeyAfter[i*2+1]) statusKeyAfter[i*2+1] = 0;
              else statusKeyAfter[i*2+1] = 1;
              sendDataMqtt(i*2+1, statusKeyAfter[i*2+1]);
            }
          }
        } else {
          masKey[i*2+1] = 0;
          statusKeyBefore[i*2+1] = statusKeyAfter[i*2+1];
        }
        
      }
    }
    //else {
    //  Serial.print(F("i = "));
    //  Serial.print(i);
    //  Serial.print(F("; stateEthernetConnected = "));
    //  Serial.println(stateEthernetConnected);
    //}
  }  
}

void updateSensor(){
  long nowTime = millis();
  if (nowTime < lastSendDataSensor) lastSendDataSensor = nowTime;
  if (nowTime - lastSendDataSensor > INTERVAL_SEND_DATA_SENSOR) {
    lastSendDataSensor = nowTime;
    for (int i = 0; i < numberDevices; i++){
      if (AD[i].type == 1 && AD[i].valid == true){
        bool ok = false;
        printBytes(AD[i].addr, 8);
        Serial.print(F("Status "));
        Serial.print(AD[i].power);
        Serial.print(F(" "));
        ok = write(AD[i].addr, !AD[i].power);
        if (!ok) Serial.println(F("Wire failed write"));
        else Serial.println(F("Write Ok"));
      }
    }
  }
}

boolean connectMqtt() {
  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    mqttClient.publish("statusLed","Reconnect ..");
    mqttClient.subscribe("led/#");
    mqttClient.subscribe("key/#");
  }
  return mqttClient.connected();
}

void loopMQQT(){
    if (!mqttClient.connected()) {
      // MQTT Client not connected
      if (millis() - lastReconnectMqtt > INTERVAL_RECONECT_MQTT_SERVER) {
        Serial.print("Attempting MQTT connection...");
        if (connectMqtt()) {
          lastReconnectMqtt = 0;
          statusConnectMqtt = true;
          Serial.println("Successful Reconnect MQTT server");
        } else {
          lastReconnectMqtt = millis();
          statusConnectMqtt = false;
          Serial.println(mqttClient.state());
        }
      }
    } else {
      // MQTT Client connected
      if(millis() - lastTimeSendDataMqtt > INTERVAL_SEND_MQTT_DATA) {
        //sendAllDataMqtt(); //sendDataMqtt();
        lastTimeSendDataMqtt = millis();
      }
      mqttClient.loop();
    }  
}

void findDevicOnBus(){
  uint8_t numDevice = 0;
  Serial.println("Поиск устройств на шине...");
  oneWire.reset_search();
  delay(250);

  while (true) {
    if ( !oneWire.search(address)) {
      break;
    }
    Serial.print(F("Адрес устройства: "));
    printBytes(address, 8);
    // eeprom_update_block((void*)&address, 8, 8);
    for (int i = 0; i < numberDevices; i++){
      if(compare(AD[i].addr, address)) {
        AD[i].valid = true;
        AD[i].power = false;
        Serial.print(F("Порядковый номер в памяти i="));
        Serial.print(i);
      }
    }
    
    if (OneWire::crc8(address, 7) == address[7]) {
      if (address[0] == DS2413_FAMILY_ID) {
        Serial.println(F(". Определилось как DS2413."));
        numDevice++;
      }
      else {
        Serial.println(F(". Не DS2413!"));
      }
    }
    else {
      Serial.println(F(". Ошибка CRC!"));
    }
  }

  if ( numDevice == 0 ) {
    Serial.println(F("Не обнаружено устройств на шине"));
    oneWire.reset_search();
  } else {
    Serial.print(F("Всего найдено устройств: "));
    Serial.println(numDevice);
  }
}

void connectEthernet() {
//  Ethernet.begin(mac,myIP,myDNS,myGW,myMASK);
//  stateEthernetConnected = true;
  if(Ethernet.begin(mac) == 0) {
    stateEthernetConnected = false;
    Serial.println(F("Ошибка подключения по DHCP"));
  } else {
    stateEthernetConnected = true;
    Serial.println("Подключение по DHCP: ");
    Serial.print("localIP:     ");
    Serial.println(Ethernet.localIP());
    Serial.print("subnetMask:  ");
    Serial.println(Ethernet.subnetMask());
    Serial.print("gatewayIP:   ");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("dnsServerIP: ");
    Serial.println(Ethernet.dnsServerIP()); 
  }
}

void setup() {
  Serial.begin(115200);
  Serial.print("Start: ");
  Serial.println(DEVICE_NAME);

  //byte datchik[8];
  //while (addressEEPROM < EEPROM.length()) {     // Перебираем адреса, до тех пор, пока не перебирем все
  //while (addressEEPROM < 10) {     // Перебираем адреса, до тех пор, пока не перебирем все
  //  value = EEPROM.read(addressEEPROM);         // Считываем значение байта
  //  Serial.println("Address: "+String(addressEEPROM)+", value: "+String(value));  // Выводим полученное значение в монитор
  //  addressEEPROM++;                            // Наращиваем адрес
  //}
  // eeprom_read_block((void*)&datchik, 8, 8);
  // printBytes(datchik, 8, 1);

  numberDevices =  (sizeof(AD)/sizeof(Sensor));
  Serial.print("Количество сенсоров в памяти: ");
  Serial.println(numberDevices);
 
  findDevicOnBus();
  connectEthernet();

  if (stateEthernetConnected) {
    if (connectMqtt()) {
      statusConnectMqtt = true;
      Serial.println("Успешное подключение к MQTT серверу");
    } else {
      statusConnectMqtt = false;
      Serial.println("Ошибка подключения к MQTT серверу");
      Serial.println(mqttClient.state());
    }
  }

  
  lastReconnectMqtt = millis();
  lastSendDataSensor = millis();
  lastTimeSendDataMqtt = millis();
  
  sendAllDataMqtt();
  
}


void loop() {
  readSensor();
  //updateSensor();
  if (!stateEthernetConnected) connectEthernet();
  if (stateEthernetConnected) loopMQQT();  
  //else stateEthernetConnected = false;
}
