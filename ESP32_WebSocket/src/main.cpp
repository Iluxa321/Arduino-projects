#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>

// SDA - 21
// SCL - 22

#define MAX_CLIENTS 2
#define PIN 14
#define addressArduino  0x01


#define PWM_1   1
#define PWM_2   2

// Board Commands
enum BoardCommand{
  START_STOP = 1,
  MODE,
  READ_STATUS,
  SET_PWM,
  READ_VOLTAGE,
  READ_CURRENT
};

///////////////////////////////////
//  Board Status
enum BoardMode{
  MODE_IDLE = 0,
  MODE_PWM
};

enum PowerMode{
  POWER_OFF = 0,
  POWER_TURNS_OFF,
  POWER_TURNS_ON,
  POWER_ON
};
////////////////////////////////

////////////////////////////////
// Client data

typedef struct ClientData{
  uint8_t boardStatus;
  uint8_t voltage;
  uint8_t current;
}ClientData;


/////////////////////////////////


typedef enum bridgeState {
  IDLE,
  WAIT_BOARD,
  CONNECTING,
  DISCONNECTING,
  WAIT_MODE,
  PWM_MODE1,
  PWM_MODE2
}bridgeState;

typedef struct Settings {
  bool connectDisconnet;
  uint8_t mode;
  uint8_t numPwm;
  uint8_t pwmArr[20];
  uint16_t time;
}Settings;



typedef struct Bridge {
  // Осуществляет подключение клиента к борду
  uint8_t client;
  uint8_t addressBoard;
  Settings settings;
  bridgeState state;
  uint32_t timer;
}Bridge;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void bridgesInit(Bridge *bridges);
bool connectedToBridge(Bridge *bridges, uint8_t client);
void disconnectedToBridge(Bridge *bridges, uint8_t client);
void clearBridge(Bridge *bridge);
void clearBoardSettings(Bridge *bridge);
Bridge *getBridge(Bridge *bridges, uint8_t client);
void updateBridge(Bridge *bridge, StaticJsonDocument<1000> *doc);


const char *ssid = "MGTS_GPON_D08B";
const char *password = "kDkQMf33";

// const char *ssid = "Lampa";
// const char *password = "Soledad88";

WebSocketsServer webSocket = WebSocketsServer(81);
Bridge bridges[2];


void bridgesInit(Bridge *bridges) {
  for(int i = 0; i < MAX_CLIENTS; i++) {
    clearBridge(&bridges[i]);
  }
}

bool connectedToBridge(Bridge *bridges, uint8_t client){
  bool connectStatus = false;
  for(int i = 0; i < MAX_CLIENTS; i++) {
    if(bridges[i].client == 255) {
      bridges[i].client = client;
      connectStatus = true;
      break;
    }
  }
  return connectStatus;
}

void disconnectedToBridge(Bridge *bridges, uint8_t client) {
  for(int i = 0; i < MAX_CLIENTS; i++) {
    if(bridges[i].client == client) {
      bridges[i].client = 255;
      break;
    }
  }
}

void clearBridge(Bridge *bridge) {
  bridge->client = 255;
  bridge->addressBoard = 255;
  bridge->settings.connectDisconnet = 0;
  bridge->settings.mode = 0;
  memset(bridge->settings.pwmArr, 0, 20);
  bridge->settings.time = 0;
  bridge->state = IDLE;
  bridge->timer = millis();
}

void clearBoardSettings(Bridge *bridge) {
  bridge->addressBoard = 255;
  bridge->settings.connectDisconnet = 0;
  bridge->settings.mode = 0;
  memset(bridge->settings.pwmArr, 0, 20);
  bridge->settings.time = 1;
}


Bridge *getBridge(Bridge *bridges, uint8_t client) {
  for(int i = 0; i < MAX_CLIENTS; i ++) {
    if(bridges[i].client == client) {
      return &bridges[i];
    }
  }
  return NULL;
}

bool boardIsBusy(Bridge *bridges, uint8_t client, uint8_t addressBoard) {
  for(int i = 0; i < MAX_CLIENTS; i ++) {
    if(bridges[i].client != client && bridges[i].addressBoard == addressBoard) {
      return true;
    }
  }
  return false;
}

uint8_t readStatus(int addr) {
  uint8_t data;
  Wire.beginTransmission(addr);
  Wire.write(READ_STATUS);
  Wire.endTransmission();
  Wire.requestFrom(addr, 1);
  while(Wire.available()){
      data = Wire.read();
  }
  return data;
}

void writeBoardData(uint8_t addr, uint8_t cmd, int data){
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readBoardData(int addr, uint8_t cmd) {
  uint8_t data;
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.endTransmission();
  Wire.requestFrom(addr, 1);
  while(Wire.available()){
      data = Wire.read();
  }
  return data;
}

void sendClient(Bridge *bridge, ClientData data) {
  StaticJsonDocument<50> msg;
  char str[50];
  msg["boardState"] = data.boardStatus;
  msg["voltage"] = data.voltage;
  msg["current"] = data.current;
  serializeJson(msg, str);
  webSocket.sendTXT(bridge->client, str);
}

bridgeState disconnectingHandler(Bridge *bridge) {
  // Режим отключения от платы
  bridgeState state = DISCONNECTING;
  if(millis() - bridge->timer > 500) {
    uint8_t status = readBoardData(bridge->addressBoard, READ_STATUS);
    status = status & 0x03;
    if(status == POWER_ON) {

      // Рассмотреть случаей когда система не переходит во выключенное состояние

      writeBoardData(bridge->addressBoard, START_STOP, 0);
    }
    else if(status == POWER_OFF) {
      if(!bridge->settings.connectDisconnet) {
        bridge->addressBoard = 255;
      }
      bridge->settings.mode = 0;
      memset(bridge->settings.pwmArr, 0, 20);
      bridge->settings.time = 1;
      if(bridge->client != 255) {
        state = WAIT_BOARD;
      }
      else {
        state = IDLE;
      }
    }
  }
  return state;
}

bridgeState connectingHandler(Bridge *bridge) {
  // Режим подключения к плате
  bridgeState state = CONNECTING;
  if(!bridge->settings.connectDisconnet || bridge->client == 255) {
    bridge->settings.connectDisconnet = 0;
    // writeBoardData(bridge->addressBoard, START_STOP, bridge->settings.connectDisconnet);
    state = DISCONNECTING;
  }
  else if(millis() - bridge->timer > 500) {
    bridge->timer = millis();
    uint8_t status = readBoardData(bridge->addressBoard, READ_STATUS);
    status = status & 0x03;
    if(status == POWER_OFF) {

      // Рассмотреть случаей когда система не переходит во включенное состояние

      writeBoardData(bridge->addressBoard, START_STOP, 1);
    }
    else if(status == POWER_ON) {
      ClientData data = {1, 0, 0};
      sendClient(bridge, data);
      state = WAIT_MODE;
    }
  }
  return state;
}

bridgeState modeHandler(Bridge *bridge) {
  bridgeState state = WAIT_MODE;
  if(bridge->client == 255 || !bridge->settings.connectDisconnet) {
    bridge->settings.connectDisconnet = 0;
    // writeBoardData(bridge->addressBoard, START_STOP, bridge->settings.connectDisconnet);
    state = DISCONNECTING;
  }
  else if(bridge->settings.mode == PWM_1) {
    writeBoardData(bridge->addressBoard, MODE, PWM_1);
    writeBoardData(bridge->addressBoard, SET_PWM, bridge->settings.pwmArr[0]);
    state = PWM_MODE1;
  }

  return state;
}

bridgeState pwmMode1Handler(Bridge *bridge) {
  bridgeState state = PWM_MODE1;
  if(bridge->client == 255 || !bridge->settings.connectDisconnet) {
    bridge->settings.connectDisconnet = 0;
    state = DISCONNECTING;
  }
  else if(bridge->settings.mode != PWM_1) {
    bridgeState state = WAIT_MODE;
  }
  else if((millis() - bridge->timer) > (bridge->settings.time)) {
    bridge->timer = millis();
    ClientData data;
    writeBoardData(bridge->addressBoard, SET_PWM, bridge->settings.pwmArr[0]);
    data.voltage = readBoardData(bridge->addressBoard, READ_VOLTAGE);
    data.current = readBoardData(bridge->addressBoard, READ_CURRENT);
    data.boardStatus = 1;
    sendClient(bridge, data);
  }
  return state;
}

void bridgeHandler(Bridge *bridge) {
  switch (bridge->state)
  {
  case IDLE:
  //  Находимся здесь если к мосту не подключен клиент
    if(bridge->client != 255) {
      bridge->state = WAIT_BOARD;
    }
    else {
      bridge->state = IDLE;
    }
    break;
  case WAIT_BOARD:
  // Находимся здесь если есть клиент, но нету платы
    if(bridge->client == 255) {
      bridge->state = IDLE;
    }
    else if(bridge->addressBoard != 255 && bridge->settings.connectDisconnet) {
      // writeBoardData(bridge->addressBoard, START_STOP, bridge->settings.connectDisconnet);
      bridge->state = CONNECTING;
    }
    else{
      bridge->state = WAIT_BOARD;
    }
    break;
  case CONNECTING:
    bridge->state = connectingHandler(bridge);
    break;
  case DISCONNECTING:
    bridge->state = disconnectingHandler(bridge);
    break; 
  case WAIT_MODE:
    bridge->state = modeHandler(bridge);
    break;
  case PWM_MODE1: 
    bridge->state = pwmMode1Handler(bridge);
    break;
  }
  
}


void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  Serial.println();
  Serial.println("Configuring access point...");
  // You can remove the password parameter if you want the AP to be open.
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());

  bridgesInit(bridges);

  // Инициируем сервер
  Wire.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

}

void loop() {

  webSocket.loop();
  for(int i = 0; i < MAX_CLIENTS; i++) {
    bridgeHandler(&bridges[i]);
  }
  Serial.print("Bridge State: ");
  Serial.println(bridges[0].state);
  Serial.print("Mode: ");
  Serial.println(bridges[0].settings.mode);
  digitalWrite(2, !digitalRead(2));
  delay(500);
  
}



void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:{
          disconnectedToBridge(bridges, num);
          Serial.print("Disconnected: ");
          Serial.println(num);
          break;
        }       
        case WStype_CONNECTED: {
          bool isConnected = connectedToBridge(bridges, num);
          if(isConnected){
            Serial.print("Connected: ");
            Serial.println(num);
          }
          else {
            webSocket.disconnect(num);
            Serial.println("To many clients ");
          }
          
          break;
        }
            
        case WStype_TEXT:{
          StaticJsonDocument<200> doc; 
          deserializeJson(doc, (const char *)payload);
          serializeJson(doc, Serial);
          Bridge *brg = getBridge(bridges ,num);
          uint8_t addrBoard = (uint8_t) doc["addressBoard"];
          if(boardIsBusy(bridges, num, addrBoard)) {
            ClientData data = {2, 0, 0};
            sendClient(brg, data);
          }
          else {
            brg->addressBoard = addrBoard;
            brg->settings.connectDisconnet = doc["connectDisconnet"].as<bool>();
            brg->settings.mode = doc["mode"];
            brg->settings.numPwm = doc["numPwm"];
            brg->settings.time = doc["timeUpdate"];
            JsonArray arr = doc["pwmArr"];
            copyArray(arr, brg->settings.pwmArr, brg->settings.numPwm);
            Serial.print("addressBoard: ");
            Serial.print(brg->addressBoard);
            Serial.print("connectDisconnet: ");
            Serial.println(brg->settings.connectDisconnet);
            Serial.print("mode: ");
            Serial.println(brg->settings.mode);
            Serial.print("time Update: ");
            Serial.println(brg->settings.time);
            Serial.print("numPwm: ");
            Serial.println(brg->settings.numPwm);
            Serial.print("pwmArr: ");
            for (int i = 0; i < brg->settings.numPwm; i++) {
              Serial.print(brg->settings.pwmArr[i]);
              Serial.print(", ");
            }
            Serial.println();
          }


          break;
        }
            
        case WStype_BIN:{
          break;
        }
            
    case WStype_ERROR:      
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
    }

}
