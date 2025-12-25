/*
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUIDs (can be anything, but must match in nRF Connect)
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

// Example PLC variable
bool plcOutputState = false;

BLECharacteristic *pCharacteristic;

// Callback when phone writes data to PLC
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {
      Serial.print("Received from BLE: ");
      Serial.println(value.c_str());

      if (value == "ON") {
        plcOutputState = true;
      } 
      else if (value == "OFF") {
        plcOutputState = false;
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(I0_0, INPUT);
  pinMode(R0_0, OUTPUT);
  digitalWrite(R0_0, LOW);

  // Initialize BLE
  BLEDevice::init("ESP32_PLC14");

  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("PLC Ready");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE PLC Ready, waiting for connection...");
}

void loop() {
  // Example: reflect PLC output state
  if (plcOutputState) {
    Serial.println("PLC Output: ON");
    digitalWrite(R0_0, HIGH);
  } else {
    Serial.println("PLC Output: OFF");
    digitalWrite(R0_0, LOW);
  }

  if ( digitalRead(I0_0) == HIGH){
      pCharacteristic->setValue(" INPUT ON");
      Serial.println("input on");
  }
  else{
    pCharacteristic->setValue("INPUT OFF");
    Serial.println("INPUT OFF");
  }

  // Notify phone
  pCharacteristic->notify();

  delay(1000);
}
  */


#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>




#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RELAY_CHAR_UUID     "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define INPUT_CHAR_UUID     "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define SENSOR_CHAR_UUID    "6e400004-b5a3-f393-e0a9-e50e24dcca9e"


uint16_t sensorValue = 0;
bool deviceConnected = false;

// BLE Characteristics
BLECharacteristic *relayChar;
BLECharacteristic *inputChar;
BLECharacteristic *sensorChar;


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Mobile connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Mobile disconnected");

    // Restart advertising so mobile can reconnect
    delay(100);  
    BLEDevice::startAdvertising();
    Serial.println("Advertising restarted");
  }
};


class RelayCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value == "ON") {
      digitalWrite(R0_0, HIGH);
      Serial.println("Relay command: ON");
    } 
    else if (value == "OFF") {
      digitalWrite(R0_0, LOW);
      Serial.println("Relay command: OFF");
    }
  }
};

void setup() {
  Serial.begin(115200);

  pinMode(I0_0, INPUT);
  pinMode(R0_0, OUTPUT);
  digitalWrite(R0_0, LOW);

  // BLE Init
  BLEDevice::init("ESP32_PLC14");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  
  relayChar = pService->createCharacteristic(
    RELAY_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  relayChar->setCallbacks(new RelayCallbacks());

 
  inputChar = pService->createCharacteristic(
    INPUT_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  inputChar->addDescriptor(new BLE2902());

  
  sensorChar = pService->createCharacteristic(
    SENSOR_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  sensorChar->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE PLC advertising...");
}

void loop() {
  if (!deviceConnected) {
    delay(500);
    return;
  }

  // input state read
  bool inputState = digitalRead(I0_0);
  inputChar->setValue(inputState ? "ON" : "OFF");
  inputChar->notify();

  // sensor value read
  sensorValue++;   
  char buf[3];
  sprintf(buf, "%u", sensorValue); 

  Serial.print("Sensor value: ");
  Serial.println(sensorValue);

  sensorChar->setValue((uint8_t*)&sensorValue, sizeof(sensorValue));
  sensorChar->notify();

  delay(1000);
}

